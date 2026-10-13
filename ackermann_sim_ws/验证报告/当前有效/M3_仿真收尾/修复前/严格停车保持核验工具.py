from pathlib import Path
import json,rosbag2_py
from rclpy.serialization import deserialize_message as dec
from geometry_msgs.msg import Twist
from std_msgs.msg import String
ws=Path('/home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws');summary=json.loads((ws/'验证报告/当前有效/M3_仿真收尾/断流复测汇总.json').read_text())
for item in summary:
 out=Path(item['output']);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));safe=[];first=None;phase=None
 while r.has_next():
  topic,d,t=r.read_next();t/=1e9
  if topic=='/sim/test_phase' and dec(d,String).data=='interruption':phase=t
  elif topic=='/sim/watchdog' and phase and t>=phase:
   m=json.loads(dec(d,String).data)
   if first is None and m['reason']=='timeout':first=t
  elif topic=='/sim/cmd_vel':m=dec(d,Twist);safe.append((t,m.linear.x,m.angular.z))
 nonzero=[x for x in safe if x[0]>=first and (abs(x[1])>.001 or abs(x[2])>.001)];result={'first_timeout_bag_epoch_s':first,'safe_samples_after_timeout':sum(x[0]>=first for x in safe),'nonzero_safe_samples_after_timeout':len(nonzero),'grace_s':0,'all_pass':not nonzero};(out/'测试结果/严格停车保持.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(out.name,json.dumps(result));assert result['all_pass']
