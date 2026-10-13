#!/usr/bin/env python3
"""Request only destination IDs; never supplies intermediate poses or velocity."""
import argparse,json,time,signal,subprocess,math,yaml
from concurrent.futures import ThreadPoolExecutor
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from pathlib import Path
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile,DurabilityPolicy
from validate_motion import Recorder,Video
class Check(Recorder):
 def __init__(self):
  super().__init__();self.events=[];self.costmaps={};self.update=self.create_publisher(OccupancyGridUpdate,"/map_updates",10);self.event_pub=self.create_publisher(String,"/sim/stability_event",10);
  for name in ["local","global"]:self.create_subscription(OccupancyGrid,f"/{name}_costmap/costmap",lambda m,name=name:self.costmaps.update({name:m}),10)
  self.states=[];self.routes=[];self.goals=[];self.plans=[]
  self.request=self.create_publisher(String,'/route/request',10)
  self.create_subscription(String,'/route/status',lambda m:self.states.append(json.loads(m.data)),10)
  self.create_subscription(String,'/route/selected',lambda m:self.routes.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(PoseStamped,'/route/local_goal',lambda m:self.goals.append([m.pose.position.x,m.pose.position.y]),10)
  self.create_subscription(NavPath,'/plan',lambda m:self.plans.append([[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
def map_contains_wall(m):
 x=int((5-m.info.origin.position.x)/m.info.resolution);y=int((0-m.info.origin.position.y)/m.info.resolution)
 return 0<=x<m.info.width and 0<=y<m.info.height and m.data[y*m.info.width+x]>=99

def main():
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);a.add_argument('--goal',type=int,default=4);a.add_argument('--failure',action='store_true');a.add_argument('--spec',required=True);a.add_argument('--video',action='store_true');args=a.parse_args();spec=json.loads(Path(args.spec).read_text())
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 for d in ['演示视频','轨迹或分析图','测试结果','原始数据']:(out/d).mkdir()
 (out/'测试结果/scenario.json').write_text(json.dumps(spec,indent=2));(out/'测试结果/roads.yaml').write_text(Path(spec['roads']).read_text());rclpy.init();n=Check();video=None;bag=None;pool=ThreadPoolExecutor(max_workers=1);physical=None;injected=False
 def event(name,**data):
  e=dict(name=name,monotonic_s=time.monotonic(),**data);n.events.append(e);n.event_pub.publish(String(data=json.dumps(e)))
 try:
  begin=time.monotonic()
  while not n.data['truth'] and time.monotonic()-begin<15:rclpy.spin_once(n,timeout_sec=.05)
  if not n.data['truth']:raise RuntimeError('NO_TRUTH')
  log=(out/'测试结果/bag-record.log').open('w')
  topics=['/clock','/odom','/wheel/odom','/ground_truth/poses','/route/request','/route/path','/route/selected','/route/local_goal','/route/status','/route/nav_cmd_vel','/cmd_vel','/sim/cmd_vel','/sim/watchdog','/plan','/tf','/tf_static','/map','/joint_states','/map_updates','/local_costmap/costmap','/global_costmap/costmap','/sim/stability_event','/lookahead_point','/curvature_lookahead_point','/received_global_plan']+['/sim/contacts/'+k for k in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']]
  bag=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(out/'原始数据/bag'),*topics],stdout=log,stderr=subprocess.STDOUT)
  until=time.monotonic()+2
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
  if spec['expected']=='FAILED' and n.update.get_subscription_count()<2:raise RuntimeError('MAP_UPDATE_SUBSCRIBERS_MISSING')
  if args.video:video=Video(out/'演示视频/demo.mp4')
  initial=n.data['truth'][-1];event('FINAL_GOAL_REQUEST',goal_id=args.goal);n.request.publish(String(data=json.dumps({'goal_id':args.goal})));begin=time.monotonic();last_frame=0;failure_sent=False;terminal=None;stop_at=None
  while time.monotonic()-begin<160:
   rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
   if video and now-last_frame>=.1:video.frame(n.states[-1]['state'] if n.states else 'WAIT',now-begin,n);last_frame=now
   if spec['expected']=='FAILED' and physical is None and math.dist([initial['x'],initial['y']],[n.data['truth'][-1]['x'],n.data['truth'][-1]['y']])>2:
    event('PHYSICAL_WALL_REQUEST')
    sdf='<sdf version="1.9"><model name="blocking_wall"><static>true</static><pose>5 0 0.75 0 0 0</pose><link name="wall"><collision name="collision"><geometry><box><size>0.5 6 1.5</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.5 6 1.5</size></box></geometry><material><ambient>0.9 0.2 0.1 1</ambient><diffuse>0.9 0.2 0.1 1</diffuse></material></visual></link></model></sdf>'
    physical=pool.submit(subprocess.run,['gz','service','-s','/world/ackermann_test/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req','sdf: '+json.dumps(sdf)],capture_output=True,text=True,timeout=8)
   if physical is not None and physical.done() and not injected:
    response=physical.result();event('PHYSICAL_WALL_RESPONSE',returncode=response.returncode,response=response.stdout,stderr=response.stderr)
    if response.returncode or 'true' not in response.stdout:raise RuntimeError('PHYSICAL_WALL_CREATION_FAILED')
    roads=yaml.safe_load(Path(spec['roads']).read_text());r=roads['resolution_m'];xmin,ymin,_,_=roads['bounds_m'];u=OccupancyGridUpdate();u.header.frame_id='odom';u.header.stamp=n.get_clock().now().to_msg();u.x=round((4.75-xmin)/r);u.y=round((-3-ymin)/r);u.width=round(.5/r);u.height=round(6/r);u.data=[100]*(u.width*u.height);event('MAP_UPDATE_PUBLISHED',x=u.x,y=u.y,width=u.width,height=u.height);n.update.publish(u);injected=True;failure_sent=True
   if n.states and n.states[-1]['state'] in ['FINISHED','FAILED'] and now-begin>2:
    if terminal is None:terminal=dict(n.states[-1]);stop_at=now
    if now-stop_at>2:break
  result=dict(terminal=terminal,routes=n.routes,goals=n.goals,plans=n.plans,initial=initial,final=n.data['truth'][-1],duration_wall_s=time.monotonic()-begin,failure_requested=failure_sent,events=n.events,costmap_wall_confirmation={name:map_contains_wall(m) for name,m in n.costmaps.items()})
  (out/'测试结果/live.json').write_text(json.dumps(result,indent=2));(out/'测试结果/observations.json').write_text(json.dumps(n.data))
  print(json.dumps({k:v for k,v in result.items() if k not in ['goals','plans']},indent=2))
 finally:
  if video:video.close()
  if bag:bag.send_signal(signal.SIGINT);bag.wait(timeout=20)
  pool.shutdown(wait=True);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
