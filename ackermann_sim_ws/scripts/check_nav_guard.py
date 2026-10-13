"""Verify isolated forward-only guard rejects reverse and in-place rotation."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
rclpy.init();n=Node('nav_guard_rejection_test');p=n.create_publisher(Twist,'/cmd_vel',1);poses=[];states=[]
def truth(m):
 for t in m.transforms:
  if t.child_frame_id=='ackermann':poses.append(t)
n.create_subscription(TFMessage,'/ground_truth/poses',truth,10);n.create_subscription(String,'/sim/watchdog',lambda m:states.append(json.loads(m.data)),10)
results={}
for name,v,w in [('reverse',-.4,0.),('pure_rotation',0.,1.)]:
 poses.clear();states.clear();until=time.monotonic()+1;next_pub=0
 while time.monotonic()<until:
  rclpy.spin_once(n,timeout_sec=.002)
  if time.monotonic()>=next_pub:
   m=Twist();m.linear.x=v;m.angular.z=w;p.publish(m);next_pub=time.monotonic()+.025
 reason='reverse_rejected' if name=='reverse' else 'pure_rotation_rejected'
 distance=((poses[-1].transform.translation.x-poses[0].transform.translation.x)**2+(poses[-1].transform.translation.y-poses[0].transform.translation.y)**2)**.5
 results[name]={'observed_expected_reason':any(s['reason']==reason for s in states),'outputs_zero':all(s['safe_speed']==0 and s['safe_yaw_rate']==0 for s in states),'position_change_m':distance}
p.publish(Twist());n.destroy_node();rclpy.shutdown()
results['all_pass']=all(s['observed_expected_reason'] and s['outputs_zero'] and s['position_change_m']<.01 for name,s in results.items())
Path('实验记录/跨阶段_安全验证/验收成果/仿真_完整负载断指令停车/guard-rejections.json').write_text(json.dumps(results,indent=2));print(json.dumps(results))
assert results['all_pass']
