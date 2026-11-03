#!/usr/bin/env python3
"""Read-only config check; --ros also compares configured topics to ROS types."""
import argparse
import json
from pathlib import Path
import sys
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from ne_tvft.core import MonitoringGame,validate_topology
from ne_tvft.protocol import agent_ids
from ne_tvft.safety import GroupSafety


def main():
    base=Path(__file__).resolve().parents[1]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=base/'config/experiment.yaml')
    p.add_argument('--topics',type=Path,default=base/'config/topics.yaml')
    p.add_argument('--ros',action='store_true')
    p.add_argument('--hardware',action='store_true')
    p.add_argument('--backend',choices=['mock','sss','hardware'])
    args=p.parse_args()
    backend=args.backend or ('hardware' if args.hardware else 'mock')
    if args.hardware and backend!='hardware':
        raise ValueError('--hardware conflicts with --backend')
    c=yaml.safe_load(args.config.read_text());c.update(yaml.safe_load(args.topics.read_text()))
    validate_topology(c);game=MonitoringGame(c)
    GroupSafety(c)
    ids=agent_ids(c)
    if set(ids)!=set(c['robots']):
        raise ValueError('Entity mapping and topic configuration differ')
    commands=[r['command_topic'] for r in c['robots'].values()]
    if len(commands)!=len(set(commands)):
        raise ValueError('Robots must not share an actuator command topic')
    for rid,r in c['robots'].items():
        for key in ['pose_topic','velocity_topic','command_topic']:
            if not r[key].startswith('/'):
                raise ValueError('Use absolute configured topic: '+rid+'/'+key)
        if r['kind']=='uav' and r['velocity_frame']=='body':
            raise ValueError('UAV body velocity requires a full-attitude converter upstream')
    if c['runtime']['rate_hz']<20 or c['runtime']['start_lead_seconds']<=2*c['runtime']['commit_margin']:
        raise ValueError('Invalid control rate or start barrier times')
    if backend=='hardware':
        for key in ['enabled','frames_verified','acceleration_interface_verified']:
            if not c['hardware'][key]:
                raise ValueError('Hardware configuration incomplete: '+key)
    if args.ros:
        import rospy
        rospy.init_node('ne_tvft_preflight',anonymous=True)
        if backend=='sss' and not (rospy.get_param('/use_sim_time',False) and rospy.get_param('/use_sss_sim_time',False)):
            raise ValueError('SSS clocks are not enabled')
        topics=dict(rospy.get_published_topics())
        prefix='/mock' if backend=='mock' else ''
        for rid in ids:
            robot=c['robots'][rid]
            for key,typ in [('pose_topic','geometry_msgs/'+robot['pose_type']),('velocity_topic','geometry_msgs/'+robot['velocity_type'])]:
                name=prefix+robot[key]
                if topics.get(name)!=typ:
                    raise ValueError('Topic absent/type mismatch: '+name+' expected '+typ)
        import rosgraph
        master=rosgraph.Master(rospy.get_name())
        publishers=dict(master.getSystemState()[0])
        for rid,r in c['robots'].items():
            names=publishers.get(prefix+r['command_topic'],[])
            if len(names)>1:
                raise ValueError('Multiple actuator publishers: '+prefix+r['command_topic']+' '+str(names))
    print(json.dumps({'configuration':'passed','agents':ids,'certificate':game.certificate()},indent=2))


if __name__=='__main__':
    main()
