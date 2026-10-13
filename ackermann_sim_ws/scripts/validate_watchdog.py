"""Repeat receive-time watchdog tests at maximum assumed vehicle speed."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bag_safety import audit
a=argparse.ArgumentParser();a.add_argument('--output',default='实验记录/跨阶段_安全验证/过程记录/仿真_停车保护80ms方案未通过');out=Path(a.parse_args().output);out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=Node('watchdog_retest');p=n.create_publisher(Twist,'/cmd_vel',1);phase=n.create_publisher(String,'/sim/test_phase',10)
f=(out/'bag-record.log').open('w');bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),'/clock','/sim/cmd_vel','/sim/watchdog','/ground_truth/poses','/sim/test_phase'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
try:
 time.sleep(2)
 for i,v in enumerate([.6,.6,.6,.4,.4]):
  phase.publish(String(data='drive'+str(i)))
  until=time.monotonic()+1.5
  while time.monotonic()<until:
   m=Twist();m.linear.x=v;p.publish(m);rclpy.spin_once(n,timeout_sec=.001);time.sleep(.024)
  phase.publish(String(data='interruption'+str(i)));time.sleep(1)
finally:
 p.publish(Twist());os.killpg(bag.pid,signal.SIGINT);bag.wait(timeout=10);f.close();n.destroy_node();rclpy.shutdown()
c=yaml.safe_load(Path('src/ackermann_sim/config/vehicle.yaml').read_text());results=[]
for i in range(5):
 x=audit(out/'bag',c,phase_name='interruption'+str(i));results.append(x)
report={'assumptions':c,'trials':results,'all_pass':all(x['trigger_latency_wall_s']<.3 and x['physical_stop_latency_wall_s']<.3 for x in results),'note':'Simulation evidence only, physical stop threshold .02 m/s and .02 rad/s held .2s; receive timestamp and sampled model pose, not hardware hard real time.'}
(out/'results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print(json.dumps(report,indent=2,ensure_ascii=False))
if not report['all_pass']:raise AssertionError('Stop budget exceeded')
