import os,json,subprocess,signal,time
from pathlib import Path
ws=Path(__file__).resolve().parents[1];r=ws/'验证报告/当前有效/M3_仿真收尾';env=os.environ.copy();env.update(ROS_DOMAIN_ID='84',GZ_PARTITION='ackermann_route_TEST');results=[];proc=None
try:
 for i in [1,2]:
  out=ws/'实验记录/跨阶段_安全验证/过程记录'/f'仿真_310米场景完整负载指令断流_正式复测{i}_20261013';log=r/f'断流{i}_启动.log'
  with log.open('w') as f:
   proc=subprocess.Popen(['ros2','launch',str(ws/'scripts/route_interrupt.launch.py'),'roads:='+str(ws/'src/ackermann_route/config/long_distance/道路_310米.yaml'),'graph:='+str(ws/'src/ackermann_route/config/long_distance/路网_310米.geojson'),'gui:=true','rviz:=true'],env=env,cwd=ws,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
   deadline=time.monotonic()+40
   while time.monotonic()<deadline:
    if proc.poll() is not None:raise RuntimeError('LAUNCH_EXIT')
    if 'Managed nodes are active' in log.read_text():break
    time.sleep(.2)
   else:raise RuntimeError('NAV_NOT_ACTIVE')
   time.sleep(3)
   with (r/f'断流{i}_验证.log').open('w') as vf:subprocess.run(['python3',str(ws/'scripts/validate_route_interruption.py'),'--output',str(out),'--video'],env=env,cwd=ws,stdout=vf,stderr=subprocess.STDOUT,check=True,timeout=60)
   os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=30);proc=None
  with (r/f'断流{i}_复核.log').open('w') as af:subprocess.run(['python3',str(ws/'scripts/audit_route_interruption.py'),str(out)],env=env,cwd=ws,stdout=af,stderr=subprocess.STDOUT,check=True,timeout=60)
  results.append({'output':str(out),'audit':json.loads((out/'测试结果/audit.json').read_text())});(r/'断流复测汇总.json').write_text(json.dumps(results,ensure_ascii=False,indent=2));print('INTERRUPTION '+str(i)+' PASS',flush=True)
finally:
 if proc and proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=30)
