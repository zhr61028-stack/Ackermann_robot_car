from pathlib import Path
import rosbag2_py,json,numpy as np
from rclpy.serialization import deserialize_message as dec
from std_msgs.msg import String
base=Path('/home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws/实验记录/M3_导航/过程记录')
for suffix in ['100米_验证1_20261013_正式长距离','310米_验证1_20261013_310米正式','310米_验证2_20261013_310米正式']:
 out=base/('仿真_长距离粗路网_'+suffix);old=json.loads((out/'测试结果/topic-coverage.json').read_text());topics=[k for k in old if k.startswith('/')];times={k:[] for k in topics};request=None;end=None;states=[];r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 while r.has_next():
  topic,d,t=r.read_next();t/=1e9
  if topic in times:times[topic].append(t)
  if topic=='/route/request':request=t
  if topic=='/route/status':
   state=json.loads(dec(d,String).data);states.append([t,state['state']])
   if state['state']=='FINISHED' and end is None:end=t
 result={}
 for name,t in times.items():
  gaps=np.diff(t);j=int(np.argmax(gaps));active=[x for x in t if request<=x<=end];activegap=np.diff(active)
  result[name]={'full_record_max_gap_s':float(gaps[j]),'full_max_gap_interval_relative_to_task_request_s':[t[j]-request,t[j+1]-request],'task_samples':len(active),'task_max_receive_gap_s':float(max(activegap)),'task_p99_gap_s':float(np.percentile(activegap,99))}
 result.update(task_request_bag_time=request,first_finished_bag_time=end,note='Task interval includes routing/planning/running, begins only at one final goal request. Startup record discovery and terminal hold are separately disclosed.')
 (out/'测试结果/topic-coverage-task.json').write_text(json.dumps(result,indent=2));print(suffix,json.dumps(result,indent=2))
