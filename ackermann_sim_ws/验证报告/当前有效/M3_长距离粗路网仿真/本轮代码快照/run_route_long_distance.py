#!/usr/bin/env python3
import argparse,json,subprocess,signal,time,os,shutil
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
p=argparse.ArgumentParser();p.add_argument('--run-id',default=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'));p.add_argument('--dry-run',action='store_true');p.add_argument('--after-100',type=Path);args=p.parse_args();ws=Path(__file__).resolve().parents[1];source_report=ws/'验证报告/当前有效/M3_长距离粗路网仿真';cases=json.loads((source_report/'场景执行清单.json').read_text());logdir=ws/'验证报告/运行日志'/('长距离_'+args.run_id)
if args.dry_run:print(json.dumps(cases,ensure_ascii=False,indent=2));raise SystemExit
if not args.run_id.replace('_','').isalnum():raise SystemExit('Invalid run ID')
if args.after_100:
 for n in ['audit.json','supplemental_audit.json','long_metrics.json']:
  if not json.loads((args.after_100/'测试结果'/n).read_text())['all_pass']:raise SystemExit('100m gate has not passed: '+n)
 if json.loads((args.after_100/'测试结果/long_metrics.json').read_text())['graph_selected_length_m']<100:raise SystemExit('100m geometry not verified')
 cases=[c for c in cases if c['graph_route_length_m']>=300]
logdir.mkdir(parents=True,exist_ok=False);env=os.environ.copy();env.update(ROS_DOMAIN_ID='84',GZ_PARTITION='ackermann_route_TEST');runs=[];launch=None;gui=None;rviz=None;validator=None;handles=[]
def stop(proc):
 if proc and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=30)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
def start(cmd,log):
 f=log.open('w');handles.append(f);return subprocess.Popen(cmd,env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
 for case in cases:
  if shutil.disk_usage(ws).free<10*1024**3:raise RuntimeError('DISK_LOW')
  name='仿真_长距离粗路网_'+case['name']+'_'+args.run_id;out=ws/'实验记录/M3_导航/过程记录'/name;sf=logdir/(case['name']+'.json');sf.write_text(json.dumps(case,ensure_ascii=False,indent=2));ll=logdir/(case['name']+'_启动.log')
  print('START '+name,flush=True);launch=start(['bash',str(ws/'scripts/start_route.bash'),'start:=A','gui:=false','rviz:=false','roads:='+case['roads'],'graph:='+case['graph'],'route_params:='+case['route_params']],ll)
  deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   if launch.poll() is not None:raise RuntimeError('LAUNCH_EXIT')
   if 'Managed nodes are active' in ll.read_text():break
   time.sleep(.2)
  else:raise RuntimeError('LIFECYCLE_TIMEOUT')
  gui=start(['gz','sim','-g','--gui-config',str(ws/'src/ackermann_nav/config/gui.config')],logdir/(case['name']+'_GazeboGUI.log'))
  rviz=start(['rviz2','-d',str(ws/'src/ackermann_route/config/long_distance/rviz/ackermann.rviz'),'--ros-args','-p','use_sim_time:=true'],logdir/(case['name']+'_RViz.log'))
  time.sleep(8)
  validator=start(['python3',str(ws/'scripts/validate_route_long_distance.py'),'--spec',str(sf),'--output',str(out),'--launch-pid',str(launch.pid),'--extra-pids',str(gui.pid),str(rviz.pid)],logdir/(case['name']+'_验证.log'));code=validator.wait(timeout=case['timeout_s']+70);validator=None
  stop(rviz);rviz=None;stop(gui);gui=None;stop(launch);launch=None
  if code:raise RuntimeError('VALIDATOR_FAILED '+str(out))
  for tool in ['audit_route_long_distance.py','audit_route_boundaries.py','audit_long_metrics.py','analyze_route_limits.py']:
   cmd=['python3',str(ws/'scripts'/tool),str(out)]
   if tool=='analyze_route_limits.py':cmd+=['--output',str(out/'测试结果/limit-analysis.json')]
   with (logdir/(case['name']+'_'+tool+'.log')).open('w') as f:subprocess.run(cmd,env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=300)
  verdicts={n:json.loads((out/'测试结果'/n).read_text()) for n in ['audit.json','supplemental_audit.json','long_metrics.json']};passed=all(v['all_pass'] for v in verdicts.values());runs.append({'case':case,'output':str(out),'all_pass':passed,'results':verdicts});(logdir/'执行进度.json').write_text(json.dumps(runs,ensure_ascii=False,indent=2));print('DONE '+name+' all_pass='+str(passed),flush=True)
  if not passed:raise RuntimeError('CRITERIA_FAILED; stop before expanding '+name)
  for f in handles:f.close()
  handles=[];time.sleep(2)
finally:
 for proc in [validator,rviz,gui,launch]:stop(proc)
 for f in handles:f.close()
