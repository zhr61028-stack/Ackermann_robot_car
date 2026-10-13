"""M3 preliminary isolated Nav2 goals. TEST ground truth localization only."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String
from validate_motion import Recorder,Video
from audit_nav import audit

def main():
 a=argparse.ArgumentParser();a.add_argument('--output',required=True);a.add_argument('--video',action='store_true');a.add_argument('--limit',type=int,default=3);args=a.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
 scene=yaml.safe_load(Path('src/ackermann_nav/config/test_scene.yaml').read_text());vehicle=yaml.safe_load(Path('src/ackermann_sim/config/vehicle.yaml').read_text())
 rclpy.init();n=Recorder();n.set_parameters([Parameter('use_sim_time',value=True)]);client=ActionClient(n,NavigateToPose,'/navigate_to_pose');bag=None;f=None;video=None;goals=[];active=None;frames=0
 try:
  if not client.wait_for_server(timeout_sec=20):raise RuntimeError('Nav2 action not available')
  until=time.monotonic()+10
  while not n.data['truth'] and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
  if not n.data['truth']:raise RuntimeError('No truth pose')
  initial=n.data['truth'][-1]
  if math.hypot(initial['x'],initial['y'])>.1 or abs(initial['yaw'])>.05:raise RuntimeError('Restart start_nav.bash: test requires a fresh vehicle at (0,0,0).')
  f=(out/'bag-record.log').open('w');topics=['/clock','/cmd_vel','/sim/cmd_vel','/sim/watchdog','/odom','/wheel/odom','/ground_truth/poses','/joint_states','/tf','/tf_static','/map','/plan','/local_costmap/costmap','/sim/test_phase']+['/sim/contacts/'+s for s in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']]
  bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),*topics],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  if args.video:video=Video(out/'demo.mp4')
  overall=time.monotonic();next_frame=overall
  def spin(phase):
   nonlocal next_frame
   rclpy.spin_once(n,timeout_sec=.001);now=time.monotonic()
   if video and now>=next_frame:video.frame(phase,now-overall,n);next_frame=now+.1
  settle=time.monotonic()+2
  while time.monotonic()<settle:spin('TEST truth localization')
  for goal in scene['goals'][:args.limit]:
   name=goal['name'];n.phase_pub.publish(String(data=name));print('GOAL',name,flush=True)
   msg=NavigateToPose.Goal();msg.pose=PoseStamped();msg.pose.header.frame_id='odom';msg.pose.header.stamp=n.get_clock().now().to_msg()
   msg.pose.pose.position.x=goal['x'];msg.pose.pose.position.y=goal['y'];msg.pose.pose.orientation.z=math.sin(goal['yaw']/2);msg.pose.pose.orientation.w=math.cos(goal['yaw']/2)
   start=time.monotonic();future=client.send_goal_async(msg)
   while not future.done() and time.monotonic()-start<10:spin(name)
   active=future.result()
   if not active.accepted:raise RuntimeError('Goal rejected')
   result=active.get_result_async()
   while not result.done() and time.monotonic()-start<scene['validation']['maximum_goal_duration_s']:spin(name)
   if not result.done():
    active.cancel_goal_async();raise RuntimeError('Goal timeout: '+name)
   res=result.result();end=time.monotonic()+1
   while time.monotonic()<end:spin(name+' STOP')
   p=n.data['truth'][-1];yawerr=abs(math.atan2(math.sin(p['yaw']-goal['yaw']),math.cos(p['yaw']-goal['yaw'])))
   item=dict(goal=goal,status=res.status,error_code=res.result.error_code,error_msg=res.result.error_msg,duration_wall_s=time.monotonic()-start,final_truth_pose=p,endpoint_error_m=math.hypot(p['x']-goal['x'],p['y']-goal['y']),endpoint_yaw_error_rad=yawerr)
   goals.append(item);(out/'goal-results.json').write_text(json.dumps(goals,indent=2));print(json.dumps(item),flush=True)
   if res.status!=4:raise RuntimeError('Navigation failed: '+name)
  tf=n.buffer.lookup_transform('odom','base_footprint',rclpy.time.Time());p=n.data['truth'][-1]
  tf_error=math.hypot(tf.transform.translation.x-p['x'],tf.transform.translation.y-p['y'])
  for frame in ['base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']:n.buffer.lookup_transform('odom',frame,rclpy.time.Time())
  report={'goals':goals,'scene':scene,'assumptions':vehicle,'tf_truth_position_error_m':tf_error,'localization':'TEST_ONLY_GAZEBO_TRUTH','video':{'frames':video.frames,'fps':10,'duration_s':video.frames/10} if video else 'not requested'}
  (out/'results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
 finally:
  if active:
   cancel=active.cancel_goal_async();until=time.monotonic()+1
   while not cancel.done() and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
  n.pub.publish(Twist())
  if video:video.close()
  if bag:os.killpg(bag.pid,signal.SIGINT);bag.wait(timeout=15)
  if f:f.close()
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
 if (out/'results.json').exists():
  report=json.loads((out/'results.json').read_text());report.update(audit(out/'bag',vehicle,scene))
  tol=scene['validation'];report['checks'].update(goals_succeeded=all(g['status']==4 for g in goals),endpoint_error=all(g['endpoint_error_m']<=tol['endpoint_position_tolerance_m'] and g['endpoint_yaw_error_rad']<=tol['endpoint_yaw_tolerance_rad'] for g in goals),tf_consistent=tf_error<.03)
  report['all_pass']=all(report['checks'].values());(out/'trajectory.json').write_text(json.dumps(report.pop('trajectory')));report['trajectory_file']='trajectory.json';(out/'results.json').write_text(json.dumps(report,indent=2,ensure_ascii=False));print('ALL_PASS',report['all_pass'],flush=True)
  if not report['all_pass']:raise AssertionError('At least one navigation check failed')
if __name__=='__main__':main()
