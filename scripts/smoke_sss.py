#!/usr/bin/env python3
"""Exercise the actual nine ROS controllers against an isolated SSS fleet."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

from smoke_mock import (SmokeFailure, Probe, check_master_uri, finite_number,
                        formal_packet, validate_recording, validate_session_id)


def check_sss_graph(config, params, graph):
    ids = [rid for g in config['groups'] for rid in [g['leader']]+g['followers']]
    if set(ids) != {'ugv1','ugv2','ugv3'} | {'uav'+str(i) for i in range(1,7)}:
        raise SmokeFailure('Expected three UGVs and six UAVs')
    if params.get('/use_sim_time') is not True or params.get('/use_sss_sim_time') is not True:
        raise SmokeFailure('Both SSS simulation clocks must be enabled')
    pub, sub, srv = [dict((t,set(n)) for t,n in rows) for rows in graph]
    if any(n not in {'/supervisor','/recorder'} and not n.startswith('/record_')
           for n in sub.get('/command',set())):
        raise SmokeFailure('Unexpected /command subscriber')
    if any(not n.startswith('/ne_tvft_smoke_sss_') for n in pub.get('/command',set())):
        raise SmokeFailure('Another command publisher is active')
    for rid in ids:
        node = '/agent_'+rid
        if (params.get(node+'/backend') != 'sss' or params.get(node+'/mock') is not False
                or params.get(node+'/allow_hardware') is not False):
            raise SmokeFailure('Agent is not configured for SSS: '+rid)
        r=config['robots'][rid]
        if pub.get(r['command_topic'],set()) != {node} or '/nodelet_manager' not in sub.get(r['command_topic'],set()):
            raise SmokeFailure('SSS actuator path is incomplete or contested: '+rid)
        for key in ('pose_topic','velocity_topic'):
            if pub.get(r[key],set()) != {'/nodelet_manager'} or node not in sub.get(r[key],set()):
                raise SmokeFailure('SSS sensor publisher mismatch: '+rid)
        if r['kind']=='uav':
            if pub.get(r['extended_state_topic'],set()) != {'/nodelet_manager'}:
                raise SmokeFailure('SSS land-state patch is missing')
            for key in ('mode_service','arming_service'):
                if '/nodelet_manager' not in srv.get(r[key],set()):
                    raise SmokeFailure('SSS MAVROS service missing: '+rid)
    return ids


def validate_sss_inputs(directory, sid, t0, ids):
    """Check recorded control semantics independently of live readiness flags."""
    counts={rid:0 for rid in ids}
    max_dt=0.
    with (Path(directory)/'telemetry.jsonl').open() as stream:
        for line in stream:
            p=json.loads(line)
            if not formal_packet(p,sid,t0): continue
            if p.get('backend')!='sss' or p.get('communication_clock')!='ros':
                raise SmokeFailure('Formal telemetry has an unexpected backend or delay clock')
            if not finite_number(p.get('dt')) or p['dt']<=0 or p['dt']>.06+1e-9:
                raise SmokeFailure('Invalid formal control period')
            max_dt=max(max_dt,p['dt'])
            if p['id'].startswith('uav'):
                target=p.get('setpoint',{})
                if p.get('command_mode')!='acceleration' or target.get('type_mask')!=2111:
                    raise SmokeFailure('UAV formal control is not acceleration-only')
            counts[p['id']]+=1
    if any(not n for n in counts.values()): raise SmokeFailure('An agent has no formal control samples')
    return {'formal_input_samples':counts,'maximum_control_dt_s':max_dt,
            'uav_type_mask':2111,'communication_clock':'ros'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=80)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    args=parser.parse_args()
    report={'success':False,'backend':'sss','requested_duration_s':args.duration,
            'verification_scope':'SSS PX4 v1.13.3 modules and quadrotor dynamics; not full firmware SITL',
            'steps':[],'started_at':datetime.now(timezone.utc).isoformat()}
    ros=publisher=probe=None
    commanded=False
    ids=[]
    try:
        check_master_uri(os.environ.get('ROS_MASTER_URI',''))
        if not finite_number(args.duration) or not 1 <= args.duration <= 100:
            raise SmokeFailure('Duration must be between 1 and 100 seconds')
        if not args.report.is_absolute() or not args.output_root.is_absolute():
            raise SmokeFailure('Report and output paths must be absolute')
        import rospy as ros
        import rosgraph
        from std_msgs.msg import String,Int32
        ros.init_node('ne_tvft_smoke_sss',anonymous=True,disable_signals=True)
        probe=Probe(ros,String)
        publisher=ros.Publisher('/command',Int32,queue_size=5)
        probe.wait('initial SSS session',lambda s,r,a:s is not None and publisher.get_num_connections()>0,45)
        config=ros.get_param('/ne_tvft/config')
        ids=[rid for g in config['groups'] for rid in [g['leader']]+g['followers']]
        if Path(config['logging']['output_root']).resolve()!=args.output_root.resolve():
            raise SmokeFailure('Output path differs from launch')
        if config.get('simulation',{}).get('backend')!='sss' or args.duration+2>=config['runtime']['max_run_seconds']:
            raise SmokeFailure('SSS configuration/duration mismatch')
        master=rosgraph.Master(ros.get_name())
        def graph_check():
            keys=['/use_sim_time','/use_sss_sim_time']+[('/agent_'+rid+'/'+k) for rid in ids for k in ['backend','mock','allow_hardware']]
            params={k:ros.get_param(k,None) for k in keys}
            return check_sss_graph(config,params,master.getSystemState())
        def graph_ready(s,r,a):
            try: graph_check(); return True
            except SmokeFailure: return False
        probe.wait('SSS graph and land-state publishers',graph_ready,45,ids=ids)
        probe.wait('nine healthy state streams',lambda s,r,a:all(rid in a and a[rid][0].get('healthy') for rid in ids),30,ids=ids)
        def command(number,target):
            nonlocal commanded
            graph_check()
            commanded=True
            publisher.publish(Int32(data=number))
            state=probe.wait(target,lambda s,r,a:s is not None and s[0].get('phase')==target,90,ids=ids)
            report['steps'].append({'command':number,'completed_phase':target,'ros_stamp':state['stamp']})
            print('command {} -> {}'.format(number,target),flush=True)
            return state
        for n,phase in [(1,'READY'),(2,'ARMED'),(3,'AIRBORNE'),(4,'POSITIONED')]: command(n,phase)
        run=command(5,'RUNNING')
        sid,t0=validate_session_id(run['session_id']),run['t0']
        directory=args.output_root/sid
        report.update(session_id=sid,t0=t0,session_directory=str(directory))
        probe.wait('nine agents acknowledging RUNNING',lambda s,r,a:all(rid in a and a[rid][0].get('phase')=='RUNNING' for rid in ids),max(2,t0-ros.Time.now().to_sec()),ids=ids)
        def run_complete(s,r,a):
            if s[0].get('phase')!='RUNNING': raise SmokeFailure('Control left RUNNING early')
            if not r or not r[0].get('healthy'): raise SmokeFailure('Recorder unhealthy')
            return ros.Time.now().to_sec()>=t0+args.duration and all(formal_packet(a[rid][0],sid,t0) for rid in ids)
        probe.wait('formal control duration',run_complete,args.duration+20,require_fresh=True,ids=ids)
        command(6,'STOPPED')
        probe.wait('recording flush',lambda s,r,a:r is not None and r[0].get('closed') is True,35,ids=ids)
        command(7,'LANDED')
        report['recording_validation']=validate_recording(directory,sid,t0,ids,args.duration,config['runtime']['rate_hz'],backend='sss')
        report['control_validation']=validate_sss_inputs(directory,sid,t0,ids)
        _,_,telemetry=probe.snapshot()
        report['final_uav_states']={rid:{k:telemetry[rid][0].get(k) for k in ['position','fcu','landed_state']} for rid in ids if rid.startswith('uav')}
        report['success']=True
    except (Exception,KeyboardInterrupt) as exc:
        report['error']='{}: {}'.format(type(exc).__name__,exc)
        print(report['error'],file=sys.stderr,flush=True)
    finally:
        if commanded and not report['success'] and probe and not ros.is_shutdown():
            for n,phase in [(6,'STOPPED'),(7,'LANDED')]:
                try:
                    publisher.publish(Int32(data=n))
                    probe.wait('cleanup '+phase,lambda s,r,a:s and s[0].get('phase')==phase,25,ignore_fault=True)
                except Exception as exc: report.setdefault('cleanup_errors',[]).append(str(exc))
        report['finished_at']=datetime.now(timezone.utc).isoformat()
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        if ros is not None: ros.signal_shutdown('SSS integration test finished')
    return 0 if report['success'] else 1


if __name__=='__main__':
    sys.exit(main())
