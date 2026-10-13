import argparse,json,numpy as np,rosbag2_py
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('experiment');p.add_argument('--output',required=True);args=p.parse_args();out=Path(args.experiment);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));names=['/odom','/ground_truth/poses','/route/nav_cmd_vel','/sim/cmd_vel','/sim/watchdog','/joint_states','/route/status'];times={k:[] for k in names}
while r.has_next():
 t,d,s=r.read_next()
 if t in times:times[t].append(s/1e9)
if Path(args.output).exists():raise RuntimeError('Refuse overwriting prior coverage evidence')
result={}
for name,t in times.items():
 gaps=np.diff(t);result[name]={'count':len(t),'first':t[0],'last':t[-1],'span_s':t[-1]-t[0],'maximum_receive_gap_s':float(max(gaps)),'p99_gap_s':float(np.percentile(gaps,99))}
result['note']='Transport loss notifications are not per-topic attributable here. Core timing coverage is measured, not a claim of mathematically continuous observation.'
Path(args.output).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
