from pathlib import Path
import json,rosbag2_py
from rclpy.serialization import deserialize_message as d
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import argparse
a=argparse.ArgumentParser();a.add_argument('experiment');args=a.parse_args();out=Path(args.experiment)
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));request=None;zero=None;failed=None;stable=None;confirm=None
while r.has_next():
 topic,data,t=r.read_next();t=t/1e9
 if topic=='/route/request' and json.loads(d(data,String).data)['goal_id']==8:request=t
 if request is None:continue
 if topic=='/sim/cmd_vel' and zero is None:
  m=d(data,Twist)
  if abs(m.linear.x)<.001 and abs(m.angular.z)<.001:zero=t
 if topic=='/route/status' and failed is None and json.loads(d(data,String).data)['state']=='FAILED':failed=t
 if topic=='/odom' and confirm is None:
  m=d(data,Odometry)
  if abs(m.twist.twist.linear.x)<.02 and abs(m.twist.twist.angular.z)<.02:
   if stable is None:stable=t
   if t-stable>=.2:confirm=t
  else:stable=None
v=dict(request_bag_timestamp_s=request,first_zero_bag_timestamp_s=zero,failed_bag_timestamp_s=failed,actual_stop_bag_timestamp_s=stable,stop_confirmation_bag_timestamp_s=confirm,first_zero_after_request_s=zero-request,failure_report_after_request_s=failed-request,actual_stop_after_request_s=stable-request,confirmed_stop_after_request_s=confirm-request,note='Complete bag reception times; localization velocities from 50Hz truth. Includes transport and sample quantization; not MCU timing or watchdog trigger latency.')
(out/'测试结果/failure-timing.json').write_text(json.dumps(v,indent=2));print(json.dumps(v,indent=2))
