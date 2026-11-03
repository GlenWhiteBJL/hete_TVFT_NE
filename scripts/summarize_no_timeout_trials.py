#!/usr/bin/env python3
"""Audit no-expiry trial recordings and keep failed attempts in the report."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def summarize(root):
    root=Path(root)
    batch=json.loads((root/'batch-status.json').read_text())
    rows=[]
    for case in batch['cases']:
        run=root/case['directory']
        row=dict(case)
        path=run/'smoke-report.json'
        smoke=json.loads(path.read_text()) if path.exists() else {}
        row['error']=smoke.get('error',case.get('error',''))
        row['ros_runtime_executed']=smoke.get('ros_runtime_executed',False)
        row['session_id']=smoke.get('session_id')
        if smoke.get('session_id'):
            session=run/'output'/smoke['session_id']
            meta=json.loads((session/'metadata.json').read_text())
            cfg=meta['config']
            if cfg['runtime']['signal_timeout'] is not False:
                raise ValueError('Actual neighbor expiry was enabled')
            row['runtime_used']=cfg['runtime']
            latest={};ages=[];over300=0;start=meta.get('t0') or 0.
            with (session/'telemetry.jsonl').open() as f:
                for line in f:
                    p=json.loads(line);latest[p['id']]=p
                    if p.get('phase')=='RUNNING' and p['stamp']>=start:
                        values=list(p.get('communication',{}).get('neighbor_source_age_s',{}).values())
                        ages.extend(values);over300+=sum(v>.3 for v in values)
            stats={}
            for p in latest.values(): stats.update(p.get('communication',{}).get('generated_per_link',{}))
            count=sum(s['count'] for s in stats.values())
            if count:
                mean=sum(s['sum_s'] for s in stats.values())/count
                var=max(0.,sum(s['sum_sq_s2'] for s in stats.values())/count-mean**2)
                row.update(generated_draws=count,injected_mean_s=mean,injected_std_s=math.sqrt(var))
            row.update(neighbor_age_samples=len(ages),neighbor_age_over_300ms=over300,
                neighbor_age_over_300ms_fraction=over300/len(ages) if ages else None,
                maximum_neighbor_source_age_s=max(ages,default=None),
                out_of_order=sum(p.get('communication',{}).get('out_of_order',0) for p in latest.values()))
            if row['success'] and smoke.get('success'):
                metrics=json.loads((run/'picture'/smoke['session_id']/'metrics.json').read_text())
                agents=metrics['agents'];counts=smoke['recording_validation']['formal_counts']
                if len(counts)!=9 or any(agents[k]['formal_samples']!=n for k,n in counts.items()):
                    raise ValueError('Analysis/raw recording sample counts differ')
                row.update(ne_tail_m=max(v['ne_error_m']['tail_max_abs'] for v in agents.values() if 'ne_error_m' in v),
                    position_tail_m=max(v['formation_position_error_m']['tail_max_abs'] for v in agents.values() if 'formation_position_error_m' in v),
                    velocity_tail_m_s=max(v['formation_velocity_error_m_s']['tail_max_abs'] for v in agents.values() if 'formation_velocity_error_m_s' in v),
                    max_control_sample_gap_s=max(v['maximum_sample_gap_s'] for v in agents.values()),
                    recording_complete=smoke['recording_validation']['recording_complete'],formal_counts=counts,
                    rosbag_sha256=hashlib.sha256((session/'raw.bag').read_bytes()).hexdigest())
        rows.append(row)
    report={'branch':batch['branch'],'scope':batch['scope'],'signal_timeout_s':None,
        'initial_set':'A','tail_seconds':10,'attempts':len(rows),
        'completed':sum(bool(r['success']) for r in rows),'cases':rows,
        'generated_distribution_includes_startup_and_out_of_order_packets':True}
    (root/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    lines=['# 250 ms 随机时延：取消邻居过期判定','',
        '分支：`experiment/communication-delay`。ROS Noetic 理想积分器 mock；没有运行 SSS、PX4 或实机。',
        '初值 A；时延均值参数 250 ms、标准差参数 50 ms、拒绝采样截断至 0–500 ms。仅 signal_timeout=false，其余定位、监督器、FCU、记录器、截止时间与执行限幅不变。','',
        '| 工况 | 完整运行 | NE 末段最大误差(m) | 编队位置末段最大误差(m) | 编队速度末段最大误差(m/s) | 超300ms邻居采样数 |',
        '|---|---|---:|---:|---:|---:|']
    for r in rows:
        values=[format(r[k],'.6g') if k in r else '--' for k in ['ne_tail_m','position_tail_m','velocity_tail_m_s']]
        lines.append('| {} | {} | {} | {} | {} | {} |'.format(r['trial'],'是' if r['success'] else '否',*values,r.get('neighbor_age_over_300ms','--')))
    lines+=['','误差统计为最后 10 秒、全部对应实体的最大范数；记录时间以 stamp-t0 为准。超过 300 ms 的数据未因年龄被拒绝，首次邻居数据仍须在启动前收到。',
        '取消此保护意味着邻居停止更新后仍可保持最后一个有效值，不再通过邻居数据年龄触发退出；定位、监督器和其他已有故障条件仍可使实验退出。',
        '注入分布统计包含启动与后续乱序丢弃的数据，不能当作端到端网络时延。单次种子结果不构成任意时延稳定性证明，理想模型残差不代表实物精度。','',
        '全部尝试、退出原因、实际配置、原始数据及 rosbag 均保留在对应工况目录；可检查 smoke-report.json、output/<session>/metadata.json 与 picture/<session>/metrics.json。']
    (root/'REPORT.md').write_text('\n'.join(lines)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    complete=[r for r in rows if r['success'] and 'position_tail_m' in r]
    fig,ax=plt.subplots(figsize=(9,4),constrained_layout=True)
    for field,label in [('ne_tail_m','Leader NE'),('position_tail_m','Formation position'),('velocity_tail_m_s','Formation velocity')]:
        ax.plot([r['trial'] for r in complete],[r[field] for r in complete],'o-',label=label)
    ax.set_yscale('log');ax.set_ylabel('Last-10-s maximum (m or m/s)');ax.legend();ax.grid(alpha=.25)
    fig.savefig(root/'comparison.png',dpi=180);plt.close(fig)
    print(root/'REPORT.md',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);summarize(p.parse_args().root)
