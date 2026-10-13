import json,math,bisect
from pathlib import Path
import numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message as dec
from nav_msgs.msg import Odometry,Path as PathMsg
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import argparse
a=argparse.ArgumentParser();a.add_argument('experiment');a.add_argument('--output',required=True);args=a.parse_args();ws=Path(__file__).resolve().parents[1];exp=Path(args.experiment);v=yaml.safe_load((ws/'src/ackermann_sim/config/vehicle.yaml').read_text())['vehicle'];bound=math.tan(v['center_steering_limit_rad'])/v['wheelbase_m']
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(exp/'原始数据/bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));odom=[];raw=[];plans=[];goals=[];watch=[];route=[]
while r.has_next():
 topic,d,t=r.read_next();t/=1e9
 if topic=='/odom':
  m=dec(d,Odometry);q=m.pose.pose.orientation;p=m.pose.pose.position;odom.append([t,p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z),m.twist.twist.linear.x,m.twist.twist.angular.z])
 elif topic=='/route/nav_cmd_vel':
  m=dec(d,Twist);raw.append([t,m.linear.x,m.angular.z])
 elif topic=='/plan':plans.append((t,dec(d,PathMsg)))
 elif topic=='/route/path':route=np.array([[p.pose.position.x,p.pose.position.y] for p in dec(d,PathMsg).poses])
 elif topic=='/sim/watchdog':watch.append(json.loads(dec(d,String).data))
ots=[o[0] for o in odom];pts=[p[0] for p in plans];events=[];curvs=[]
for t,s,w in raw:
 if abs(s)<.001:continue
 k=abs(w/s)
 if k<=bound+1e-6:continue
 o=odom[min(len(odom)-1,bisect.bisect_left(ots,t))];plan=plans[max(0,bisect.bisect_right(pts,t)-1)][1];xy=np.array([[p.pose.position.x,p.pose.position.y] for p in plan.poses]);i=int(np.argmin(np.linalg.norm(xy-o[1:3],axis=1)));a=max(0,i-2);b=min(len(xy)-1,i+2);dxy=np.linalg.norm(np.diff(xy[a:b+1],axis=0),axis=1);q=[p.pose.orientation for p in plan.poses[a:b+1]];angles=np.unwrap([math.atan2(2*q.w*q.z,1-2*q.z*q.z) for q in q]);pk=max(abs(np.diff(angles)/np.maximum(dxy,1e-6)),default=0)
 j=max(0,i-8);k2=min(len(xy)-1,i+8);aa=xy[j];bb=xy[i];cc=xy[k2];u=bb-aa;vv=cc-aa;den=np.linalg.norm(bb-aa)*np.linalg.norm(cc-bb)*np.linalg.norm(cc-aa);geometry=2*abs(u[0]*vv[1]-u[1]*vv[0])/den if den>1e-9 else 0
 events.append(dict(plan_geometric_curvature_per_m=geometry,t=t,x=o[1],y=o[2],yaw=o[3],speed=s,raw_yaw=w,raw_curvature=k,physical_bound=bound,excess_percent=(k/bound-1)*100,delta_yaw=abs(w)-abs(s)*bound,plan_nearby_curvature=pk,cross_track_plan_m=float(np.linalg.norm(xy[i]-o[1:3])),cross_track_route_m=float(np.min(np.linalg.norm(route-o[1:3],axis=1))),seconds_after_plan=t-plans[max(0,bisect.bisect_right(pts,t)-1)][0]))
groups=[]
for e in events:
 if not groups or e['t']-groups[-1][-1]['t']>.04:groups.append([])
 groups[-1].append(e)
summary=dict(raw_messages=len(raw),limited_raw_messages=len(events),fraction=len(events)/len(raw),physical_bound=bound,groups=[dict(begin=g[0]['t'],end=g[-1]['t'],duration_s=g[-1]['t']-g[0]['t']+.025,count=len(g),start_xy=[g[0]['x'],g[0]['y']],max_excess_percent=max(e['excess_percent'] for e in g),max_delta_yaw_radps=max(e['delta_yaw'] for e in g),max_plan_orientation_difference_curvature=max(e['plan_nearby_curvature'] for e in g),max_plan_geometric_curvature=max(e['plan_geometric_curvature_per_m'] for e in g),max_cross_track_plan_m=max(e['cross_track_plan_m'] for e in g),max_cross_track_route_m=max(e['cross_track_route_m'] for e in g)) for g in groups],events=events)
Path(args.output).write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='events'},indent=2))
