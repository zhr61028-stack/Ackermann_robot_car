"""Complete-bag checks independent of video/live subscription sampling."""
import json,math
from collections import Counter
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message as decode
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,Path
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ros_gz_interfaces.msg import Contacts

def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def polygon_distance(a,b):
 overlap=True
 for poly in [a,b]:
  for p,q in zip(poly,np.roll(poly,-1,axis=0)):
   axis=np.array([-(q-p)[1],(q-p)[0]]);axis/=np.linalg.norm(axis)
   aa=a@axis;bb=b@axis
   if max(aa)<min(bb) or max(bb)<min(aa):overlap=False
 if overlap:return 0.0
 distances=[]
 for points,edges in [(a,b),(b,a)]:
  for p in points:
   for c,d in zip(edges,np.roll(edges,-1,axis=0)):
    u=d-c;t=np.clip(np.dot(p-c,u)/np.dot(u,u),0,1);distances.append(np.linalg.norm(p-c-t*u))
 return float(min(distances))
def audit(bag,vehicle,scene):
 r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 v=vehicle['vehicle'];minimum=v['wheelbase_m']/math.tan(v['center_steering_limit_rad']);raw=[];safe=[];odom=[];truth=[];joints=[];contacts=Counter();obstacle_contacts=[];phases=[];statuses=[];plans=[]
 while r.has_next():
  topic,data,t=r.read_next()
  if topic in ['/cmd_vel','/sim/cmd_vel']:
   m=decode(data,Twist);(raw if topic=='/cmd_vel' else safe).append((t/1e9,m.linear.x,m.angular.z))
  elif topic=='/odom':
   m=decode(data,Odometry);odom.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.twist.twist.linear.x,m.twist.twist.angular.z))
  elif topic=='/ground_truth/poses':
   for m in decode(data,TFMessage).transforms:
    if m.child_frame_id=='ackermann':truth.append((t/1e9,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.transform.translation.x,m.transform.translation.y,yaw(m.transform.rotation)))
  elif topic=='/joint_states':
   m=decode(data,JointState)
   joints.extend(abs(a) for n,a in zip(m.name,m.position) if 'steering_joint' in n)
  elif topic.startswith('/sim/contacts/'):
   m=decode(data,Contacts);contacts[topic]+=1
   for c in m.contacts:
    names=c.collision1.name+' '+c.collision2.name
    if any(b['name'] in names for b in scene['obstacles']):obstacle_contacts.append(names)
  elif topic=='/sim/test_phase':phases.append((t/1e9,decode(data,String).data))
  elif topic=='/sim/watchdog':statuses.append(json.loads(decode(data,String).data))
  elif topic=='/plan':
   m=decode(data,Path);plans.append([(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in m.poses])
 rear=min(v['body_center_x_m']-v['body_length_m']/2,-v['wheel_radius_m']);front=max(v['body_center_x_m']+v['body_length_m']/2,v['wheelbase_m']+v['wheel_radius_m']);half=max(v['body_width_m']/2,v['track_m']/2+math.hypot(v['wheel_radius_m'],v['wheel_width_m']/2))
 local=np.array([[rear,-half],[front,-half],[front,half],[rear,half]])
 boxes=[np.array([[b['x']+sx*b['size_x']/2,b['y']+sy*b['size_y']/2] for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]) for b in scene['obstacles']]
 clearance=min(polygon_distance(local@np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])+[x,y],b) for _,_,x,y,a in truth for b in boxes)
 actual=[abs(w/s) for _,s,w in odom if s>.10];worst=max(actual)
 command_violations=sum(abs(w)>abs(s)/minimum+1e-6 for _,s,w in safe)
 raw_violations=sum(abs(w)>abs(s)/minimum+1e-6 for _,s,w in raw)
 stationary_rotation=sum(abs(s)<.001 and abs(w)>.001 for _,s,w in safe)
 reverse=sum(s<-.001 for _,s,w in safe)
 moving_timeouts=sum(s['reason']=='timeout' and s['age_s']<.2 for s in statuses)
 # Keep independent actual truth trajectory for plots/reproduction.
 segments={}
 for i,(begin,name) in enumerate(phases):
  end=phases[i+1][0] if i+1<len(phases) else float('inf');samples=[p for p in truth if begin<=p[0]<end]
  segments[name]={'actual_path_length_m':sum(math.hypot(b[2]-a[2],b[3]-a[3]) for a,b in zip(samples,samples[1:])),'min_y':min(p[3] for p in samples),'max_y':max(p[3] for p in samples)}
 return {'bag_audit':{'samples':{'truth':len(truth),'command':len(raw),'safe':len(safe),'truth_odom':len(odom)},'minimum_obstacle_clearance_m':clearance,'obstacle_contact_count':len(obstacle_contacts),'contact_messages':dict(contacts),'max_measured_curvature_per_m_above_0_1mps':worst,'physical_curvature_bound_per_m':1/minimum,'minimum_measured_radius_m':1/worst,'max_steering_joint_rad':max(joints),'safe_constraint_violations':command_violations,'raw_nav_constraint_violations':raw_violations,'safe_in_place_rotation_commands':stationary_rotation,'safe_reverse_commands':reverse,'timeout_samples_age_under_200ms_including_normal_goal_stops':moving_timeouts,'segments':segments,'localization':'Gazebo model pose, TEST ONLY'},'checks':{'no_obstacle_contact':not obstacle_contacts,'no_geometric_overlap':clearance>0,'wheel_ground_contact_streams_present':all('/sim/contacts/'+name in contacts for name in ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']),'safe_commands_feasible':command_violations==0,'no_in_place_rotation':stationary_rotation==0,'forward_only':reverse==0,'actual_forward_only':all(s>=-.02 for _,s,w in odom),'actual_no_in_place_rotation':not any(abs(s)<.002 and abs(w)>.02 for _,s,w in odom),'physical_curvature':worst<=1/minimum+scene['validation']['actual_curvature_tolerance_per_m'],'steering_limits':max(joints)<=v['wheel_joint_limit_rad']+.001},'trajectory':{'truth':truth,'plans':plans}}
