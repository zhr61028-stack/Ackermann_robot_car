#!/usr/bin/env python3
"""Request only destination IDs; never supplies intermediate poses or velocity."""
import argparse,json,time,signal,subprocess,math
from pathlib import Path
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile,DurabilityPolicy
from validate_motion import Recorder,Video
class Check(Recorder):
 def __init__(self):
  super().__init__();self.states=[];self.routes=[];self.goals=[];self.plans=[]
  self.request=self.create_publisher(String,'/route/request',10)
  self.create_subscription(String,'/route/status',lambda m:self.states.append(json.loads(m.data)),10)
  self.create_subscription(String,'/route/selected',lambda m:self.routes.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(PoseStamped,'/route/local_goal',lambda m:self.goals.append([m.pose.position.x,m.pose.position.y]),10)
  self.create_subscription(NavPath,'/plan',lambda m:self.plans.append([[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
def main():
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);a.add_argument('--goal',type=int,default=4);a.add_argument('--failure',action='store_true');a.add_argument('--video',action='store_true');args=a.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 for d in ['演示视频','轨迹或分析图','测试结果','原始数据']:(out/d).mkdir()
 rclpy.init();n=Check();video=None;bag=None
 try:
  begin=time.monotonic()
  while not n.data['truth'] and time.monotonic()-begin<15:rclpy.spin_once(n,timeout_sec=.05)
  if not n.data['truth']:raise RuntimeError('NO_TRUTH')
  log=(out/'测试结果/bag-record.log').open('w')
  topics=['/clock','/odom','/wheel/odom','/ground_truth/poses','/route/request','/route/path','/route/selected','/route/local_goal','/route/status','/route/nav_cmd_vel','/cmd_vel','/sim/cmd_vel','/sim/watchdog','/plan','/tf','/tf_static','/map','/joint_states']+['/sim/contacts/'+k for k in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']]
  bag=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(out/'原始数据/bag'),*topics],stdout=log,stderr=subprocess.STDOUT)
  until=time.monotonic()+2
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
  if args.video:video=Video(out/'演示视频/demo.mp4')
  initial=n.data['truth'][-1];n.request.publish(String(data=json.dumps({'goal_id':args.goal})));begin=time.monotonic();last_frame=0;failure_sent=False;terminal=None;stop_at=None
  while time.monotonic()-begin<160:
   rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
   if video and now-last_frame>=.1:video.frame(n.states[-1]['state'] if n.states else 'WAIT',now-begin,n);last_frame=now
   if args.failure and not failure_sent and math.dist([initial['x'],initial['y']],[n.data['truth'][-1]['x'],n.data['truth'][-1]['y']])>2:
    n.request.publish(String(data='{"goal_id":8}'));failure_sent=True
   if n.states and n.states[-1]['state'] in ['FINISHED','FAILED'] and now-begin>2:
    if terminal is None:terminal=dict(n.states[-1]);stop_at=now
    if now-stop_at>2:break
  result=dict(terminal=terminal,routes=n.routes,goals=n.goals,plans=n.plans,initial=initial,final=n.data['truth'][-1],duration_wall_s=time.monotonic()-begin,failure_requested=failure_sent)
  (out/'测试结果/live.json').write_text(json.dumps(result,indent=2));(out/'测试结果/observations.json').write_text(json.dumps(n.data))
  print(json.dumps({k:v for k,v in result.items() if k not in ['goals','plans']},indent=2))
 finally:
  if video:video.close()
  if bag:bag.send_signal(signal.SIGINT);bag.wait(timeout=20)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
