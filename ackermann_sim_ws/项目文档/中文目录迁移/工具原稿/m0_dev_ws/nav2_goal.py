import signal, time, json, math
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener
signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError("Nav2 verification time limit")))
signal.alarm(150)
rclpy.init()
nav=BasicNavigator()
nav.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
buf=Buffer();tf=TransformListener(buf,nav)
pose=PoseStamped();pose.header.frame_id="map";pose.pose.position.x=-2.0;pose.pose.position.y=-0.5;pose.pose.orientation.w=1.0
pose.header.stamp=nav.get_clock().now().to_msg()
nav.setInitialPose(pose)
nav.waitUntilNav2Active()
print("NAV2_ACTIVE",flush=True)
start=None
for _ in range(30):
 rclpy.spin_once(nav,timeout_sec=0.1)
 try:
  t=buf.lookup_transform("map","base_link",rclpy.time.Time());start=[t.transform.translation.x,t.transform.translation.y];break
 except Exception:pass
print("INITIAL_TF",start,flush=True)
goal=PoseStamped();goal.header.frame_id="map";goal.header.stamp=nav.get_clock().now().to_msg();goal.pose.position.x=-1.0;goal.pose.position.y=-0.5;goal.pose.orientation.w=1.0
nav.goToPose(goal)
begin=time.monotonic();i=0
while not nav.isTaskComplete():
 if time.monotonic()-begin>90:nav.cancelTask();raise TimeoutError("Goal timeout")
 feedback=nav.getFeedback();i+=1
 if feedback and i%20==0:print("DISTANCE_REMAINING",feedback.distance_remaining,flush=True)
result=nav.getResult()
for _ in range(10):rclpy.spin_once(nav,timeout_sec=0.1)
t=buf.lookup_transform("map","base_link",rclpy.time.Time());final=[t.transform.translation.x,t.transform.translation.y]
report={"robot":"TurtleBot3 Waffle differential drive","result":str(result),"success":result==TaskResult.SUCCEEDED,"initial_xy":start,"goal_xy":[-1.0,-0.5],"final_tf_xy":final,"position_error_m":math.hypot(final[0]+1,final[1]+0.5),"elapsed_wall_s":time.monotonic()-begin}
Path("records/nav2-goal-result.json").write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
nav.destroy_node();rclpy.shutdown();signal.alarm(0)
raise SystemExit(0 if report["success"] else 1)
