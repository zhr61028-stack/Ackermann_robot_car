import os,json,subprocess,signal,time,argparse
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
parser=argparse.ArgumentParser(description='Fresh independent stability suite; never overwrite previous evidence');parser.add_argument('--run-id',default=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S'));parser.add_argument('--dry-run',action='store_true');args=parser.parse_args()
if not args.run_id.replace('_','').replace('-','').isalnum():raise SystemExit('run-id must contain letters, digits, underscores or hyphens')
ws=Path(__file__).resolve().parents[1];p=ws/'src/ackermann_route/config';report=ws/'验证报告/运行日志'/('稳定性复现_'+args.run_id);process=ws/'实验记录/M3_导航/过程记录';runs=[]

base=dict(roads=str(p/'roads.yaml'),graph=str(p/'local_test_graph.geojson'),route_params=str(p/'route_server.yaml'),map_updates=False)
def spec(name,start,goal,target,nodes,junctions,video=False,**changes):
 d=dict(base,name=name,start=start,goal=goal,target=target,expected_nodes=nodes,required_junctions=junctions,expected='FINISHED',video=video);d.update(changes);return d
cases=[spec('原路线A_复测1','A',4,[24,0,0],[1,2,3,4],[2,3]),spec('原路线B_复测1','B',5,[16,8,1.5707963267948966],[7,2,3,5],[2,3],True),spec('原路线B_复测2','B',5,[16,8,1.5707963267948966],[7,2,3,5],[2,3]),spec('近极限弯道_复测1','B',5,[16,8,1.5707963267948966],[7,2,3,5],[2,3],True,route_params=str(p/'stability/tight_route.yaml')),spec('近极限弯道_复测2','B',5,[16,8,1.5707963267948966],[7,2,3,5],[2,3],route_params=str(p/'stability/tight_route.yaml')),spec('连续路口分支_复测1','A',5,[16,6,0],[1,2,6,5],[2,6],True,roads=str(p/'stability/continuous_roads.yaml'),graph=str(p/'stability/continuous_graph.geojson')),spec('连续路口分支_复测2','A',5,[16,6,0],[1,2,6,5],[2,6],roads=str(p/'stability/continuous_roads.yaml'),graph=str(p/'stability/continuous_graph.geojson')),spec('途中封路_复测1','A',4,[24,0,0],[1,2,3,4],[2,3],True,map_updates=True,expected='FAILED'),spec('途中封路_复测2','A',4,[24,0,0],[1,2,3,4],[2,3],map_updates=True,expected='FAILED')]
if args.dry_run:
 print(json.dumps(cases,indent=2));raise SystemExit(0)
report.mkdir(parents=True,exist_ok=False)
(report/'场景执行清单.json').write_text(json.dumps(cases,indent=2));env=os.environ.copy();env.update(ROS_DOMAIN_ID='84',GZ_PARTITION='ackermann_route_TEST');launch=None
try:
 for case in cases:
  name='仿真_稳定性_'+case['name']+'_'+args.run_id;out=process/name
  if out.exists():raise RuntimeError('Refuse existing output '+str(out))
  spec_file=report/(case['name']+'.json');spec_file.write_text(json.dumps(case,indent=2));launch_log=report/(case['name']+'_启动.log');lf=launch_log.open('w');args=['bash',str(ws/'scripts/start_route.bash'),f"start:={case['start']}",f"gui:={str(case['video']).lower()}",f"rviz:={str(case['video']).lower()}",f"roads:={case['roads']}",f"graph:={case['graph']}",f"route_params:={case['route_params']}",f"map_updates:={str(case['map_updates']).lower()}"]
  print('START '+name,flush=True);launch=subprocess.Popen(args,env=env,cwd=ws,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True)
  deadline=time.monotonic()+25
  while time.monotonic()<deadline:
   if launch.poll() is not None:raise RuntimeError('Launch exited')
   if 'Managed nodes are active' in launch_log.read_text():break
   time.sleep(.2)
  else:raise RuntimeError('Lifecycle not active')
  log=report/(case['name']+'_验证.log')
  with log.open('w') as f:
   cmd=['python3',str(ws/'scripts/validate_route_stability.py'),'--spec',str(spec_file),'--goal',str(case['goal']),'--output',str(out)]
   if case['video']:cmd.append('--video')
   r=subprocess.run(cmd,env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,timeout=185)
  os.killpg(launch.pid,signal.SIGINT);launch.wait(timeout=25);lf.close();launch=None
  if r.returncode:raise RuntimeError('Validator failed '+str(log))
  audit_log=report/(case['name']+'_复核.log')
  with audit_log.open('w') as f:r=subprocess.run(['python3',str(ws/'scripts/audit_route_stability.py'),str(out)],env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,timeout=60)
  if r.returncode:raise RuntimeError('Audit failed '+str(audit_log))
  verdict=json.loads((out/'测试结果/audit.json').read_text());
  with (report/(case['name']+'_补充复核.log')).open('w') as f:subprocess.run(['python3',str(ws/'scripts/audit_route_boundaries.py'),str(out)],env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=60)
  supplement=json.loads((out/'测试结果/supplemental_audit.json').read_text());verdict['all_pass']=verdict['all_pass'] and supplement['all_pass']
  with (report/(case['name']+'_限幅分析.log')).open('w') as f:subprocess.run(['python3',str(ws/'scripts/analyze_route_limits.py'),str(out),'--output',str(out/'测试结果/limit-analysis.json')],env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=60)
  runs.append(dict(case=case,output=str(out),audit=verdict));(report/'执行进度.json').write_text(json.dumps(runs,indent=2));print('DONE '+name+' all_pass='+str(verdict['all_pass']),flush=True)
  if not verdict['all_pass']:raise RuntimeError('Criteria failed; stop before expanding '+name)
  time.sleep(1)
finally:
 if launch and launch.poll() is None:
  os.killpg(launch.pid,signal.SIGINT);launch.wait(timeout=25)
