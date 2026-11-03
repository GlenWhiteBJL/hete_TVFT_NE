#!/usr/bin/env python3
"""Run isolated mock or SSS random-delay trials on the delay branch only."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import math
import yaml

ROOT=Path(__file__).resolve().parents[1]


def save(path,value):
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    tmp.replace(path)


def git(*args):
    return subprocess.check_output(['git','--git-dir='+str(ROOT/'.git'),'--work-tree='+str(ROOT),*args],text=True).strip()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=80.)
    p.add_argument('--seeds',nargs='+',type=int,default=[11,29,47])
    p.add_argument('--backend',choices=['mock','sss'],default='mock')
    p.add_argument('--baseline-only',action='store_true',help='Run only one baseline; retain other batches separately')
    a=p.parse_args()
    if a.baseline_only: a.seeds=[]
    if git('branch','--show-current')!='experiment/communication-delay':
        raise RuntimeError('Run only on experiment/communication-delay')
    if not math.isfinite(a.duration) or not 1 <= a.duration <=100:
        raise ValueError('Duration must be in [1,100] seconds')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    base=yaml.safe_load((ROOT/'config'/('sss_experiment.yaml' if a.backend=='sss' else 'experiment.yaml')).read_text())
    if base['runtime']['signal_timeout'] is not False or base['hardware']['enabled']:
        raise RuntimeError('Require disabled neighbor timeout and disabled hardware')
    profile=yaml.safe_load((ROOT/'config/communication_normal250.yaml').read_text())
    state={'branch':git('branch','--show-current'),'base_commit':git('rev-parse','HEAD'),
           'backend':a.backend,
           'scope':('SSS mecanum UGVs and adapted PX4 modules + quadrotor dynamics; not full firmware SITL or hardware'
                    if a.backend=='sss' else 'ROS Noetic ideal mock, no PX4/SSS/hardware'),'signal_timeout_s':None,
           'planned_trials':1+len(a.seeds),'cases':[],'stage':'unit_tests'}
    save(out/'batch-status.json',state)
    (out/'source-changes.patch').write_text(git('diff'))
    save(out/'source-sha256.json',{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
        for folder in ['src','scripts','config','tests'] for f in (ROOT/folder).rglob('*') if f.is_file() and '__pycache__' not in f.parts})
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',
        PYTHONPATH=str(ROOT/'src')+os.pathsep+os.environ.get('PYTHONPATH',''),MPLBACKEND='Agg')
    with (out/'unit-tests.log').open('w') as log:
        subprocess.run([sys.executable,'-B','-m','unittest','discover','-s','tests','-v'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    for label,seed in [('baseline_no_delay',None)]+[('normal250_seed'+str(s),s) for s in a.seeds]:
        run=out/label;run.mkdir();(run/'output').mkdir();(run/'picture').mkdir();(run/'ros').mkdir()
        config=copy.deepcopy(base);comm=copy.deepcopy(profile)
        comm['communication']['enabled']=seed is not None
        comm['communication']['seed']=seed if seed is not None else 11
        if a.backend=='sss': comm['communication']['clock']='ros'
        for name,value in [('experiment.yaml',config),('communication.yaml',comm)]:
            (run/name).write_text(yaml.safe_dump(value,sort_keys=False))
        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        case_env=dict(env,ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_HOME=str(run/'ros'))
        case_env.pop('ROS_HOSTNAME',None)
        row={'trial':label,'seed':seed,'directory':label,'backend':a.backend,'duration_s':a.duration,'signal_timeout_s':None,'success':False}
        state['stage']=label;save(out/'batch-status.json',state)
        print('BEGIN '+label,flush=True)
        launch=None
        try:
            with (run/'roslaunch.log').open('w') as log:
                launch_args=(['sss.launch','open_rviz:=false'] if a.backend=='sss' else ['experiment.launch','mock:=true','allow_hardware:=false'])
                launch=subprocess.Popen(['roslaunch','ne_tvft_experiment']+launch_args+[
                    'config:='+str(run/'experiment.yaml'),'communication:='+str(run/'communication.yaml'),
                    'output_root:='+str(run/'output'),'picture_root:='+str(run/'picture')],env=case_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                with (run/'smoke.log').open('w') as smoke:
                    code=subprocess.run([sys.executable,'-B',str(ROOT/'scripts'/('smoke_sss.py' if a.backend=='sss' else 'smoke_mock.py')),'--duration',str(a.duration),
                        '--output-root',str(run/'output'),'--report',str(run/'smoke-report.json')],env=case_env,stdout=smoke,stderr=subprocess.STDOUT,timeout=a.duration+300).returncode
                row['exit_code']=code
                if code==0:
                    report=json.loads((run/'smoke-report.json').read_text())
                    with (run/'analysis.log').open('w') as analysis:
                        subprocess.run([sys.executable,'-B',str(ROOT/'scripts/analyze_experiment.py'),report['session_directory'],
                            '--picture-root',str(run/'picture')],env=case_env,stdout=analysis,stderr=subprocess.STDOUT,check=True,timeout=90)
                    row['success']=bool(report['success'])
        except Exception as exc:
            row['error']=str(exc)
        finally:
            if launch:
                for sig,wait in [(signal.SIGINT,15),(signal.SIGTERM,5),(signal.SIGKILL,3)]:
                    try: os.killpg(launch.pid,sig)
                    except ProcessLookupError: break
                    try: launch.wait(timeout=wait);break
                    except subprocess.TimeoutExpired: pass
            save(run/'run-status.json',row)
            state['cases'].append(row);save(out/'batch-status.json',state)
            print(('PASS ' if row['success'] else 'FAIL ')+label,flush=True)
    state['stage']='completed';state['success']=all(c['success'] for c in state['cases'])
    save(out/'batch-status.json',state)
    print(str(out),flush=True)
    return 0 if state['success'] else 1


if __name__=='__main__': sys.exit(main())
