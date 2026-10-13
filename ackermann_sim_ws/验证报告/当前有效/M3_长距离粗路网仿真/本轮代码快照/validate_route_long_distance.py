#!/usr/bin/env python3
"""Observe one final Route Server task; never publish a driving command."""
import argparse,json,time,signal,subprocess,os,math
from pathlib import Path
from collections import deque
import rclpy
from std_msgs.msg import String
from validate_route_stability import Check
from validate_motion import Video

def resource_sample(groups):
 rows=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   s=(p/'stat').read_text().split(') ',1)[1].split()
   if int(s[2]) in groups:rows.append({'pid':int(p.name),'name':(p/'comm').read_text().strip(),'cpu_s':(int(s[11])+int(s[12]))/os.sysconf('SC_CLK_TCK'),'rss_bytes':int(s[21])*os.sysconf('SC_PAGE_SIZE')})
  except (OSError,ValueError,IndexError):pass
 mem={k:int(v.split()[0])*1024 for k,v in (l.split(':',1) for l in Path('/proc/meminfo').read_text().splitlines())};return dict(monotonic_s=time.monotonic(),processes=rows,aggregate_rss_bytes=sum(p['rss_bytes'] for p in rows),system_available_bytes=mem['MemAvailable'])

def main():
 a=argparse.ArgumentParser();a.add_argument('--spec',required=True);a.add_argument('--output',required=True);a.add_argument('--launch-pid',type=int,required=True);a.add_argument('--extra-pids',nargs='*',type=int,default=[]);args=a.parse_args();spec=json.loads(Path(args.spec).read_text());out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 for d in ['演示视频','轨迹或分析图','测试结果','原始数据']:(out/d).mkdir()
 (out/'测试结果/scenario.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2));(out/'测试结果/roads.yaml').write_text(Path(spec['roads']).read_text());(out/'测试结果/graph.geojson').write_text(Path(spec['graph']).read_text())
 rclpy.init();n=Check();n.data={k:deque(maxlen=500) for k in n.data};bag=None;video=None;resources=[];terminal=None;initial=None
 topics=['/clock','/odom','/ground_truth/poses','/route/request','/route/path','/route/selected','/route/local_goal','/route/status','/route/nav_cmd_vel','/cmd_vel','/sim/cmd_vel','/sim/watchdog','/plan','/tf','/tf_static','/map','/joint_states','/sim/stability_event']+['/sim/contacts/'+k for k in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']]
 (out/'测试结果/录制范围.json').write_text(json.dumps({'full_topics':topics,'excluded_repeated_auxiliary':['/local_costmap/costmap','/global_costmap/costmap','/trajectory/*','/received_global_plan','/lookahead_point'],'note':'关键topic全部消息；不抽样控制/真值/关节/接触；map静态一次；完整录包指全程关键数据而非所有ROS辅助topic'},ensure_ascii=False,indent=2))
 try:
  deadline=time.monotonic()+20
  while not n.data['truth'] and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
  if not n.data['truth']:raise RuntimeError('NO_TRUTH')
  log=(out/'测试结果/bag-record.log').open('w');bag=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(out/'原始数据/bag'),*topics],stdout=log,stderr=subprocess.STDOUT)
  wait=time.monotonic()+3
  while time.monotonic()<wait:rclpy.spin_once(n,timeout_sec=.005)
  if spec['video']:
   # Presentation-only existing Gazebo camera service; cannot steer the car.
   commands=[['gz','service','-s','/gui/follow','--reqtype','gz.msgs.StringMsg','--reptype','gz.msgs.Boolean','--timeout','3000','--req','data: "ackermann"'],['gz','service','-s','/gui/follow/offset','--reqtype','gz.msgs.Vector3d','--reptype','gz.msgs.Boolean','--timeout','3000','--req','x: -7, y: -10, z: 15']]
   responses=[]
   for command in commands:
    r=subprocess.run(command,capture_output=True,text=True,timeout=5);responses.append({'command':command,'code':r.returncode,'stdout':r.stdout,'stderr':r.stderr})
   (out/'测试结果/相机跟随.json').write_text(json.dumps(responses,indent=2));video=Video(out/'演示视频/demo.mp4')
  initial=dict(n.data['truth'][-1]);begin=time.monotonic();n.event_pub.publish(String(data=json.dumps({'name':'FINAL_GOAL_REQUEST','goal_id':spec['goal'],'monotonic_s':begin})));n.request.publish(String(data=json.dumps({'goal_id':spec['goal']})))
  last_frame=last_resource=last_status=0;stop_at=None
  while time.monotonic()-begin<spec['timeout_s']:
   rclpy.spin_once(n,timeout_sec=.001);now=time.monotonic()
   if video and now-last_frame>=.1:video.frame(n.states[-1]['state'] if n.states else 'WAIT',now-begin,n);last_frame=now
   if now-last_resource>=2:
    sample=resource_sample({args.launch_pid,os.getpid(),*args.extra_pids});sample.update(elapsed_s=now-begin,sim_time_s=n.data['truth'][-1]['sim']);resources.append(sample);last_resource=now
    if sample['system_available_bytes']<2*1024**3:raise RuntimeError('RESOURCE_MEMORY_LOW')
   if now-last_status>=15:
    st=n.states[-1] if n.states else {};print(json.dumps({'elapsed_s':round(now-begin,1),'state':st,'pose':dict(n.data['truth'][-1])},ensure_ascii=False),flush=True);last_status=now
   if n.states and n.states[-1]['state'] in ['FINISHED','FAILED'] and now-begin>2:
    if terminal is None:terminal=dict(n.states[-1]);stop_at=now
    if now-stop_at>2:break
  result=dict(terminal=terminal,routes=n.routes,goals=n.goals,initial=initial,final=dict(n.data['truth'][-1]),duration_wall_s=time.monotonic()-begin,navigation_terminal_wall_s=stop_at-begin if stop_at else None,failure_requested=False,manual_interventions=0,costmap_wall_confirmation={})
  (out/'测试结果/live.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False),flush=True)
 finally:
  if video:video.close()
  if bag:bag.send_signal(signal.SIGINT);bag.wait(timeout=30)
  (out/'测试结果/resources.json').write_text(json.dumps(resources,indent=2));n.destroy_node();rclpy.shutdown()
 if not terminal or terminal['state']!='FINISHED':raise RuntimeError('TASK_NOT_FINISHED')
if __name__=='__main__':main()
