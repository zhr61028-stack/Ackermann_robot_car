"""Positive control for the chassis contact sensor AFTER navigation; virtual only."""
import json,subprocess,time
from pathlib import Path
import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
out=Path('实验记录/跨阶段_安全验证/验收成果/仿真_车身接触检测正向测试');out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=Node('virtual_contact_positive_control');pose=[];contacts=[]
def p(m):
 for t in m.transforms:
  if t.child_frame_id=='ackermann':pose.append(t)
def c(m):
 for t in m.contacts:
  names=t.collision1.name+' '+t.collision2.name
  if 'contact_probe' in names:contacts.append(names)
n.create_subscription(TFMessage,'/ground_truth/poses',p,10);n.create_subscription(Contacts,'/sim/contacts/base_link',c,10)
until=time.monotonic()+5
while not pose and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
if not pose:raise RuntimeError('No simulator true pose')
t=pose[-1];q=t.transform.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));x=t.transform.translation.x+.4*math.cos(a);y=t.transform.translation.y+.4*math.sin(a)
sdf='<sdf version="1.9"><model name="contact_probe"><static>true</static><link name="probe"><collision name="box"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision><visual name="box"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></visual></link></model></sdf>'
try:
 run=subprocess.run(['ros2','run','ros_gz_sim','create','-world','ackermann_test','-name','contact_probe','-string',sdf,'-x',str(x),'-y',str(y),'-z','.20'],capture_output=True,text=True,timeout=15)
 (out/'spawn.log').write_text(run.stdout+run.stderr)
 until=time.monotonic()+5
 while not contacts and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 result={'body_sensor_detected_injected_virtual_contact':bool(contacts),'samples':contacts[:5],'spawn_returncode':run.returncode,'note':'AFTER successful navigation bag closed. Deliberate positive control, not part of no-collision navigation trajectory.'}
 (out/'results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
 if not contacts:raise AssertionError('Body contact positive control not observed')
finally:n.destroy_node();rclpy.shutdown()
