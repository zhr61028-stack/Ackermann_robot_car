import argparse,json,math,bisect
from pathlib import Path
import numpy as np,rosbag2_py,yaml
from rclpy.serialization import deserialize_message as dec
from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from audit_nav import yaw

def main():
 a=argparse.ArgumentParser();a.add_argument('experiment');a.add_argument('--reference-target',nargs=3,type=float);a.add_argument('--output');args=a.parse_args();
 if args.reference_target and not args.output:a.error('Reference analysis requires a separate --output path')
 out=Path(args.experiment);spec={'expected':'FINISHED','target':args.reference_target} if args.reference_target else json.loads((out/'测试结果/scenario.json').read_text());v=yaml.safe_load((Path(__file__).resolve().parents[1]/'src/ackermann_sim/config/vehicle.yaml').read_text())['vehicle'];front=max(v['body_center_x_m']+v['body_length_m']/2,v['wheelbase_m']+v['wheel_radius_m']);half=max(v['body_width_m']/2,v['track_m']/2+math.hypot(v['wheel_radius_m'],v['wheel_width_m']/2));r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));odom=[];safe=[];state=[];maps={'local':[],'global':[]};update=[];events=[];route=[];plans=[];goals=[]
 while r.has_next():
  topic,d,t=r.read_next();t/=1e9
  if topic=='/odom':
   m=dec(d,Odometry);p=m.pose.pose;odom.append([t,p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z])
  elif topic=='/sim/cmd_vel':
   m=dec(d,Twist);safe.append([t,m.linear.x,m.angular.z])
  elif topic=='/route/status':state.append([t,json.loads(dec(d,String).data)])
  elif topic=='/map_updates':update.append(t)
  elif topic=='/sim/stability_event':events.append([t,json.loads(dec(d,String).data)])
  elif topic in ['/local_costmap/costmap','/global_costmap/costmap']:
   m=dec(d,OccupancyGrid);x=int((5-m.info.origin.position.x)/m.info.resolution);y=int(-m.info.origin.position.y/m.info.resolution);value=m.data[y*m.info.width+x] if 0<=x<m.info.width and 0<=y<m.info.height else -1;maps['local' if topic.startswith('/local') else 'global'].append([t,value])
  elif topic=='/route/path':route=[[p.pose.position.x,p.pose.position.y] for p in dec(d,NavPath).poses]
  elif topic=='/plan':plans.append([t,[[p.pose.position.x,p.pose.position.y] for p in dec(d,NavPath).poses]])
 route=np.array(route);first_motion=next(o[0] for o in odom if o[4]>.05);failure=spec['expected']=='FAILED';until=update[0] if failure and update else float('inf');begin=None;pauses=[]
 for t,x,y,a,v,w in odom:
  eligible=t>=first_motion and t<until and math.dist([x,y],spec['target'][:2])>1
  if eligible and abs(v)<.02:
   if begin is None:begin=t
  elif begin is not None:
   if t-begin>.2:pauses.append([begin,t,t-begin])
   begin=None
 route_errors=[float(np.min(np.linalg.norm(route-[o[1],o[2]],axis=1))) for o in odom];path_errors=[];ptimes=[p[0] for p in plans]
 for t,x,y,a,v,w in odom:
  i=bisect.bisect_right(ptimes,t)-1
  if i>=0 and plans[i][1]:path_errors.append(float(np.min(np.linalg.norm(np.array(plans[i][1])-[x,y],axis=1))))
 changes=[];last=[]
 for t,s in state:
  if s['visited_junctions']!=last:changes.append(dict(t=t,junctions=s['visited_junctions']));last=s['visited_junctions']
 result=dict(no_intermediate_pause=not pauses,intermediate_pauses=pauses,maximum_route_point_distance_m=max(route_errors),maximum_current_plan_point_distance_m=max(path_errors),junction_transitions=changes)
 if failure:
  injected=update[0];zero=next(t for t,v,w in safe if t>=injected and abs(v)<.001 and abs(w)<.001);report_time=next(t for t,s in state if s['state']=='FAILED');stable=None;confirm=None
  for t,x,y,a,v,w in odom:
   if t<zero:continue
   if abs(v)<.02 and abs(w)<.02:
    if stable is None:stable=t
    if t-stable>=.2:confirm=t;break
   else:stable=None
  acknowledgements={k:next((t for t,value in rows if t>=injected and value>=99),None) for k,rows in maps.items()};motion_after_fail=[(t,v,w) for t,v,w in safe if t>=report_time and (abs(v)>.001 or abs(w)>.001)];min_clearance=min(4.75-(x+front*math.cos(a)+half*abs(math.sin(a))) for t,x,y,a,v,w in odom if t>=injected)
  result.update(map_update_bag_time=injected,first_zero_bag_time=zero,first_stop_bag_time=stable,confirmed_stop_bag_time=confirm,failure_report_bag_time=report_time,first_zero_after_update_s=zero-injected,first_stop_after_zero_s=stable-zero if stable else None,stop_confirmation_after_zero_s=confirm-zero if confirm else None,stop_after_update_s=stable-injected if stable else None,costmap_acknowledgements=acknowledgements,costmaps_accepted=all(v is not None for v in acknowledgements.values()),motion_commands_after_failure=len(motion_after_fail),failure_latched=not motion_after_fail,minimum_front_wall_clearance_m=min_clearance,stop_performance_pass=bool(stable) and stable-zero<=.3,wall_clearance_pass=min_clearance>0)
 result['all_pass']=result['no_intermediate_pause'] and (not failure or all(result[k] for k in ['costmaps_accepted','failure_latched','stop_performance_pass','wall_clearance_pass']))
 (Path(args.output) if args.output else out/'测试结果/supplemental_audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
