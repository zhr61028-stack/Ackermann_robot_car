#!/usr/bin/env python3
import argparse,json,math
from pathlib import Path
from collections import Counter
import numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message as dec
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist,PoseStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts
from audit_nav import yaw,wrap
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
 a=argparse.ArgumentParser();a.add_argument('experiment');args=a.parse_args();out=Path(args.experiment);ws=Path(__file__).resolve().parents[1]
 spec=json.loads((out/'测试结果/scenario.json').read_text());cfg=yaml.safe_load((out/'测试结果/roads.yaml').read_text());v=yaml.safe_load((ws/'src/ackermann_sim/config/vehicle.yaml').read_text())['vehicle'];live=json.loads((out/'测试结果/live.json').read_text());R=v['wheelbase_m']/math.tan(v['center_steering_limit_rad'])
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(out/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 truth=[];odom=[];safe=[];raw=[];status=[];goals=[];route=[];plans=[];joints=[];contacts=Counter();bad_contacts=[];frames=Counter();watch=[]
 while reader.has_next():
  t,d,ns=reader.read_next();wall=ns/1e9
  if t=='/ground_truth/poses':
   for p in dec(d,TFMessage).transforms:
    if p.child_frame_id=='ackermann':truth.append([wall,p.header.stamp.sec+p.header.stamp.nanosec*1e-9,p.transform.translation.x,p.transform.translation.y,yaw(p.transform.rotation)])
  elif t=='/odom':
   m=dec(d,Odometry);odom.append([wall,m.twist.twist.linear.x,m.twist.twist.angular.z]);frames[(m.header.frame_id,m.child_frame_id)]+=1
  elif t in ['/sim/cmd_vel','/route/nav_cmd_vel']:
   m=dec(d,Twist);(safe if t=='/sim/cmd_vel' else raw).append([wall,m.linear.x,m.angular.z])
  elif t=='/route/status':status.append([wall,json.loads(dec(d,String).data)])
  elif t=='/sim/watchdog':watch.append(json.loads(dec(d,String).data))
  elif t=='/route/local_goal':
   m=dec(d,PoseStamped);goals.append([wall,m.pose.position.x,m.pose.position.y])
  elif t=='/route/path':route=[[p.pose.position.x,p.pose.position.y] for p in dec(d,NavPath).poses]
  elif t=='/plan':plans.append([[p.pose.position.x,p.pose.position.y] for p in dec(d,NavPath).poses])
  elif t=='/joint_states':
   m=dec(d,JointState);joints.extend(abs(p) for n,p in zip(m.name,m.position) if 'steering_joint' in n)
  elif t.startswith('/sim/contacts/'):
   contacts[t]+=1
   for c in dec(d,Contacts).contacts:
    names=c.collision1.name+' '+c.collision2.name
    if 'ground_plane' not in names:bad_contacts.append(names)
 rear=min(v['body_center_x_m']-v['body_length_m']/2,-v['wheel_radius_m']);front=max(v['body_center_x_m']+v['body_length_m']/2,v['wheelbase_m']+v['wheel_radius_m']);half=max(v['body_width_m']/2,v['track_m']/2+math.hypot(v['wheel_radius_m'],v['wheel_width_m']/2))
 # Exact SAT overlap against rectangular cells of the complement of the road union.
 xs=sorted(set([cfg['bounds_m'][0],cfg['bounds_m'][2]]+[z for r in cfg['road_rectangles'] for z in [r[0],r[2]]]))
 ys=sorted(set([cfg['bounds_m'][1],cfg['bounds_m'][3]]+[z for r in cfg['road_rectangles'] for z in [r[1],r[3]]]))
 forbidden=[]
 for x1,x2 in zip(xs,xs[1:]):
  for y1,y2 in zip(ys,ys[1:]):
   x=(x1+x2)/2;y=(y1+y2)/2
   if not any(a<=x<=c and b<=y<=d for a,b,c,d in cfg['road_rectangles']):forbidden.append((x,y,(x2-x1)/2,(y2-y1)/2))
 outside=0;local_center=(rear+front)/2;lh=(front-rear)/2
 for _,_,x,y,angle in truth:
  c=math.cos(angle);s=math.sin(angle);center=np.array([x+c*local_center,y+s*local_center]);axes=[np.array([1.,0.]),np.array([0.,1.]),np.array([c,s]),np.array([-s,c])]
  overlap=False
  for bx,by,hx,hy in forbidden:
   delta=np.array([bx,by])-center
   if all(abs(delta@axis)<lh*abs(np.dot([c,s],axis))+half*abs(np.dot([-s,c],axis))+hx*abs(axis[0])+hy*abs(axis[1])-1e-8 for axis in axes):overlap=True;break
  outside+=overlap
 curvature=max([abs(w/s) for _,s,w in odom if s>.1],default=0)
 safe_bad=sum(abs(w)>abs(s)/R+1e-6 or abs(s)>v['max_speed_mps']+1e-6 for _,s,w in safe);raw_bad=sum(abs(w)>abs(s)/R+1e-6 for _,s,w in raw)
 failure=live['failure_requested'];terminal=live['terminal'];last=truth[-1];target=spec['target'];position=math.dist(last[2:4],target[:2]);angle=abs(wrap(last[4]-target[2]))
 # Count sustained uncommanded intermediate stops using complete truth-derived odometry.
 stops=[];run=None;truth_times=np.array(truth)[:,0];local_extents=[]
 for tm,gx,gy in goals:
  k=min(len(truth)-1,int(np.searchsorted(truth_times,tm)));p=truth[k];local_extents.append(max(abs(gx-p[2]),abs(gy-p[3])))
 for tm,s,w in odom:
  near=truth[min(len(truth)-1,int(np.searchsorted(truth_times,tm)))];dist=math.dist(near[2:4],target[:2]);active=next((st['state']=='RUNNING' for t,st in reversed(status) if t<=tm),False)
  if active and dist>1 and abs(s)<.02:
   if run is None:run=tm
  elif run is not None:
   if tm-run>.2:stops.append(tm-run)
   run=None
 stop={}
 if failure:
  failtime=next(t for t,st in status if st['state']=='FAILED');zero=next(t for t,s,w in safe if t>=failtime and abs(s)<.001 and abs(w)<.001)
  stable=None
  for tm,s,w in odom:
   if tm<failtime:continue
   if abs(s)<.02 and abs(w)<.02:
    if stable is None:stable=tm
    if tm-stable>=.2:stop=dict(command_zero_after_report_s=zero-failtime,actual_stop_after_report_s=stable-failtime,stop_confirmed_after_report_s=tm-failtime);break
   else:stable=None
 checks=dict(wheel_contacts_present=all('/sim/contacts/'+k in contacts for k in ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']),expected_terminal=bool(terminal) and terminal['state']==('FAILED' if failure else 'FINISHED'),road_footprint_inside=outside==0,no_non_ground_contact=not bad_contacts,command_constraints=safe_bad==0,actual_curvature=curvature<=1/R+.03,forward_only=all(s>=-.02 for _,s,w in odom),no_in_place_rotation=not any(abs(s)<.002 and abs(w)>.02 for _,s,w in odom),steering_limits=max(joints,default=0)<=v['wheel_joint_limit_rad']+.001,local_goals_within_window=max(local_extents,default=0)<=5.,odom_frame=set(frames)=={('odom','base_footprint')},no_intermediate_stops=not stops)
 if failure:checks.update(map_update_accepted=all(live['costmap_wall_confirmation'].get(k,False) for k in ['local','global']),failure_reason=bool(terminal) and ('CONTROLLER_FAILED' in terminal['reason'] or 'LOCAL_PLANNING_FAILED' in terminal['reason']),failure_stopped=bool(stop))
 else:checks.update(endpoint_position=position<=.25,endpoint_yaw=angle<=.25,junction_order=terminal['visited_junctions']==spec['required_junctions'])
 if not failure:checks['route_selection']=live['routes'][-1]['nodes']==spec['expected_nodes']
 report=dict(all_pass=all(checks.values()),checks=checks,route=live['routes'],terminal=terminal,endpoint_position_error_m=None if failure else position,endpoint_yaw_error_rad=None if failure else angle,road_outside_pose_samples=outside,road_footprint_method='Oriented conservative vehicle rectangle SAT versus exact rectangular complement of road union',maximum_local_goal_axis_extent_m=max(local_extents,default=0),truth_samples=len(truth),minimum_actual_radius_m=1/curvature if curvature else None,physical_minimum_radius_m=R,safe_command_violations=safe_bad,raw_nav_constraint_violations=raw_bad,max_steering_joint_rad=max(joints,default=0),non_ground_contact_count=len(bad_contacts),contact_messages=dict(contacts),intermediate_stops_s=stops,failure_stop=stop,local_goal_count=len(goals),plan_count=len(plans),actual_path_length_m=sum(math.dist(a[2:4],b[2:4]) for a,b in zip(truth,truth[1:])),localization='TEST_GAZEBO_TRUTH',road_boundary='KNOWN_TEST_OCCUPANCY_MAP',measurement='Bag reception times for failure delays; odometry velocities from simulated truth pose differences, not hardware timing.')
 (out/'测试结果/audit.json').write_text(json.dumps(report,indent=2))
 fig,ax=plt.subplots(figsize=(10,7))
 from matplotlib.patches import Rectangle
 for x1,y1,x2,y2 in cfg['road_rectangles']:ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,color='#dddddd'))
 for i,path in enumerate(plans):
  if i%3==0:ax.plot(*np.array(path).T,color='salmon',alpha=.45,label='Smac paths' if i==0 else None)
 if route:ax.plot(*np.array(route).T,color='deepskyblue',linewidth=3,label='Route Server')
 ax.plot(np.array(truth)[:,2],np.array(truth)[:,3],color='green',linewidth=1.5,label='Actual TEST truth')
 if goals:ax.scatter(np.array(goals)[:,1],np.array(goals)[:,2],s=14,color='orange',label='Local goals')
 ax.set(xlim=(-3,27),ylim=(-11,12),xlabel='odom x / m',ylabel='odom y / m',title='SIMULATION / local metric graph / known road boundaries');ax.set_aspect('equal');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'轨迹或分析图/route_and_trajectory.png');plt.close(fig)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
