#!/usr/bin/env python3
"""TEST ONLY locked command relay; do not use as vehicle safety architecture."""
import argparse,json,time,signal,subprocess
from pathlib import Path
from collections import deque
import rclpy,yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from validate_route import Check
from validate_motion import Video
from bag_safety import audit
from rclpy.qos import QoSProfile
class TestGate(Check):
 def __init__(self):
  super().__init__();self.destroy_publisher(self.pub);self.set_parameters([rclpy.Parameter('use_sim_time',value=True)]);self.data={k:deque(maxlen=1000) for k in self.data};self.closed=False;self.forwarded=0;self.blocked=0;self.gate=self.create_publisher(Twist,'/test/guard_input',QoSProfile(depth=1));self.create_subscription(Twist,'/cmd_vel',self.relay,10)
 def relay(self,msg):
  if not self.closed:self.gate.publish(msg);self.forwarded+=1
  else:self.blocked+=1

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--video',action='store_true');args=p.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 for name in ['测试结果','原始数据','演示视频','轨迹或分析图']:(out/name).mkdir()
 ws=Path(__file__).resolve().parents[1];rclpy.init();n=TestGate();bag=None;video=None;result={};phase=n.create_publisher(String,'/sim/test_phase',10)
 try:
  deadline=time.monotonic()+20
  while not n.data['truth'] and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
  if not n.data['truth']:raise RuntimeError('NO_TRUTH')
  f=(out/'测试结果/bag-record.log').open('w');topics=['/clock','/odom','/ground_truth/poses','/route/request','/route/path','/route/selected','/route/local_goal','/route/status','/plan','/cmd_vel','/route/nav_cmd_vel','/test/guard_input','/sim/cmd_vel','/sim/watchdog','/sim/test_phase','/tf','/tf_static','/map','/joint_states'];bag=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(out/'原始数据/bag'),*topics],stdout=f,stderr=subprocess.STDOUT)
  until=time.monotonic()+3
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.002)
  topology={'guard_input_subscribers':[{'node':x.node_name,'namespace':x.node_namespace} for x in n.get_subscriptions_info_by_topic('/test/guard_input')],'guard_input_publishers':[{'node':x.node_name,'namespace':x.node_namespace} for x in n.get_publishers_info_by_topic('/test/guard_input')]}
  if len(topology['guard_input_publishers'])!=1 or len(topology['guard_input_subscribers'])!=1:raise RuntimeError('INPUT_NOT_ISOLATED')
  if args.video:video=Video(out/'演示视频/demo.mp4')
  n.request.publish(String(data='{"goal_id":6}'));begin=time.monotonic();moving_since=None;trip=None;last_frame=0
  while time.monotonic()-begin<30:
   rclpy.spin_once(n,timeout_sec=.001);now=time.monotonic()
   if video and now-last_frame>.1:video.frame('INPUT_CUT_LOCKED' if n.closed else 'AUTONOMOUS',now-begin,n);last_frame=now
   od=n.data['odom'][-1] if n.data['odom'] else None
   if od and od['v']>.35:
    if moving_since is None:moving_since=now
   elif trip is None:moving_since=None
   if moving_since is not None and trip is None and now-moving_since>=4:
    n.closed=True;trip=now;phase.publish(String(data='interruption'));result.update(trip_monotonic_s=trip,upstream_state_at_trip=n.states[-1],pose_at_trip=dict(n.data['truth'][-1]))
   if trip is not None and now-trip>=3:break
  if trip is None:raise RuntimeError('NO_STEADY_MOTION')
  result.update(topology=topology,gate_latched=n.closed,forwarded_messages=n.forwarded,blocked_upstream_messages=n.blocked,final_pose=dict(n.data['truth'][-1]),final_state=n.states[-1],scope='Only control input interruption with guard alive; not process crash/disconnect/MCU.',simulation_scene='310m graph; full Nav2/Gazebo/RViz launch; interrupted early during ongoing task');(out/'测试结果/live.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 finally:
  if video:video.close()
  if bag:bag.send_signal(signal.SIGINT);bag.wait(timeout=20)
  n.destroy_node();rclpy.shutdown()
 c=yaml.safe_load((ws/'src/ackermann_sim/config/vehicle.yaml').read_text());measurement=audit(out/'原始数据/bag',c);(out/'测试结果/stop-timing.json').write_text(json.dumps(measurement,ensure_ascii=False,indent=2));print(json.dumps(measurement,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
