from pathlib import Path
import json,rosbag2_py
from rclpy.serialization import deserialize_message as dec
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
out=Path('/home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws/实验记录/M3_导航/过程记录/仿真_长距离粗路网_310米_验证2_20261013_310米正式');cfg=json.loads((out/'测试结果/topic-coverage-task.json').read_text());start=cfg['task_request_bag_time'];windows={'/odom':[start+2.30,start+2.51],'/joint_states':[start+57.56,start+57.71]};rows={k:[] for k in windows};r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
while r.has_next():
 topic,d,t=r.read_next();t/=1e9
 if topic in windows and windows[topic][0]<=t<=windows[topic][1]:
  if topic=='/odom':
   m=dec(d,Odometry);rows[topic].append({'wall_after_task_s':t-start,'sim':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'v':m.twist.twist.linear.x,'w':m.twist.twist.angular.z})
  else:
   m=dec(d,JointState);rows[topic].append({'wall_after_task_s':t-start,'sim':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'steering':{n:p for n,p in zip(m.name,m.position) if 'steering' in n}})
result={}
for topic,rs in rows.items():
 gaps=[{'receive_gap_s':b['wall_after_task_s']-a['wall_after_task_s'],'header_gap_s':b['sim']-a['sim'],'before':a,'after':b} for a,b in zip(rs,rs[1:])];entry=max(gaps,key=lambda s:s['receive_gap_s']);result[topic]=entry;print(topic,json.dumps(entry,indent=2))

(out/'测试结果/接收间隔与仿真采样核验.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
