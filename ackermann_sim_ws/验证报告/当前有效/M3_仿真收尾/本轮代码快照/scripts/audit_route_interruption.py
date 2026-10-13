import argparse,json,bisect,statistics
from pathlib import Path
import rosbag2_py,numpy as np
from rclpy.serialization import deserialize_message as dec
from geometry_msgs.msg import Twist
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('experiment');a=p.parse_args();out=Path(a.experiment);r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));input=[];raw=[];safe=[];watch=[];phase=None;selected=[]
while r.has_next():
 topic,d,t=r.read_next();t/=1e9
 if topic in ['/test/guard_input','/cmd_vel','/sim/cmd_vel']:
  m=dec(d,Twist);{'/test/guard_input':input,'/cmd_vel':raw,'/sim/cmd_vel':safe}[topic].append([t,m.linear.x,m.angular.z])
 elif topic=='/sim/watchdog':watch.append([t,json.loads(dec(d,String).data)])
 elif topic=='/sim/test_phase' and dec(d,String).data=='interruption':phase=t
 elif topic=='/route/selected':selected.append(json.loads(dec(d,String).data))
timing=json.loads((out/'测试结果/stop-timing.json').read_text());live=json.loads((out/'测试结果/live.json').read_text());trigger=next(t for t,s in watch if t>=phase and s['reason']=='timeout');after=[s for t,s in watch if t>=trigger];upstream=[row for row in raw if row[0]>=trigger and abs(row[1])>.01];late_inputs=[row for row in input if row[0]>=trigger];moving_safe=[row for row in safe if row[0]>=trigger+.005 and (abs(row[1])>.001 or abs(row[2])>.001)];tickzero=next(t for t,v,w in safe if t>=trigger and abs(v)<.001 and abs(w)<.001)
checks={'route_310_task_selected':selected[-1]['nodes']==[1,2,3,4,5,6],'single_gate_publisher':len(live['topology']['guard_input_publishers'])==1,'single_guard_subscriber':sum(x['node']=='command_guard' for x in live['topology']['guard_input_subscribers'])==1,'gate_locked':live['gate_latched'],'upstream_kept_requesting':len(upstream)>30,'no_guard_input_after_timeout':not late_inputs,'no_input_recovery_in_native_status':len({s['last_rx_monotonic_s'] for s in after})==1,'safe_zero_after_timeout':not moving_safe,'native_trigger_within_budget':.06<=timing['trigger_latency_wall_s']<.3,'first_physical_stop_within_budget':timing['physical_stop_latency_wall_s']<=.3,'sustained_stop_recorded':timing['physical_stop_confirmed_latency_wall_s']>=timing['physical_stop_latency_wall_s']+.2-.005}
result={'all_pass':all(checks.values()),'checks':checks,'upstream_nonzero_after_timeout':len(upstream),'input_after_timeout':len(late_inputs),'nonzero_safe_after_timeout':len(moving_safe),'phase_to_timeout_bag_s':trigger-phase,'first_zero_after_timeout_bag_s':tickzero-trigger,'stop_times_from_last_guard_input_s':{'trigger':timing['trigger_latency_wall_s'],'first_stop':timing['physical_stop_latency_wall_s'],'confirmed_stop':timing['physical_stop_confirmed_latency_wall_s']},'scope':live['scope'],'transport_error_note':'Bag receive time quantization and median monotonic/epoch alignment, not hardware hard realtime.'};(out/'测试结果/audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2));assert result['all_pass']
