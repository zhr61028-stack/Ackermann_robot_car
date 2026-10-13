"""Local metric TEST route executor; route selection is owned by Nav2 Route Server."""
import json,math,time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path as NavPath
from nav2_msgs.action import ComputeRoute,ComputePathToPose,FollowPath

class Generator(Node):
 def __init__(self):
  super().__init__('local_goal_generator')
  self.declare_parameter('roads_file','');self.declare_parameter('graph_file','');self.declare_parameter('vehicle_file','')
  self.cfg=yaml.safe_load(Path(self.get_parameter('roads_file').value).read_text());self.settings=self.cfg['local_goal']
  graph=json.loads(Path(self.get_parameter('graph_file').value).read_text());self.nodes={f['properties']['id']:f['geometry']['coordinates'] for f in graph['features'] if f['geometry']['type']=='Point'}
  v=yaml.safe_load(Path(self.get_parameter('vehicle_file').value).read_text())['vehicle'];self.rear=min(v['body_center_x_m']-v['body_length_m']/2,-v['wheel_radius_m']);self.front=max(v['body_center_x_m']+v['body_length_m']/2,v['wheelbase_m']+v['wheel_radius_m']);self.half=max(v['body_width_m']/2,v['track_m']/2+math.hypot(v['wheel_radius_m'],v['wheel_width_m']/2));self.footprint=[(self.rear+(self.front-self.rear)*i/24,-self.half+2*self.half*j/14) for i in range(25) for j in range(15)]
  self.state='IDLE';self.reason='';self.pose=None;self.last_odom=0.;self.epoch=0;self.plan_busy=False;self.follow=None;self.seq=0;self.progress=0.;self.last_plan_progress=-99.;self.last_plan_time=0.;self.failures=0;self.visited=[];self.points=[]
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.status=self.create_publisher(String,'/route/status',10);self.selected=self.create_publisher(String,'/route/selected',q);self.path_pub=self.create_publisher(NavPath,'/route/path',q);self.goal_pub=self.create_publisher(PoseStamped,'/route/local_goal',10);self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
  self.create_subscription(Odometry,'/odom',self.odom,10);self.create_subscription(Twist,'/route/nav_cmd_vel',self.forward,10);self.create_subscription(String,'/route/request',self.request,10)
  self.router=ActionClient(self,ComputeRoute,'compute_route');self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose');self.controller=ActionClient(self,FollowPath,'follow_path')
  self.create_timer(.05,self.tick)
 def odom(self,msg):
  p=msg.pose.pose;self.pose=(p.position.x,p.position.y,math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y**2+p.orientation.z**2)));self.last_odom=time.monotonic()
 def forward(self,msg):
  if self.state=='RUNNING' and time.monotonic()-self.last_odom<self.settings['odom_timeout_s']:self.cmd.publish(msg)
 def on_road(self,x,y):
  return any(a<=x<=c and b<=y<=d for a,b,c,d in self.cfg['road_rectangles'])
 def body_on_road(self,pose):
  x,y,a=pose;c=math.cos(a);s=math.sin(a)
  return all(self.on_road(x+c*u-s*v,y+s*u+c*v) for u,v in self.footprint)
 def fail(self,reason):
  # Invalidate pending callbacks before closing command forwarding.
  self.epoch+=1;self.plan_busy=False
  self.state='FAILED';self.reason=reason;self.cmd.publish(Twist())
  if self.follow:self.follow.cancel_goal_async()
  self.get_logger().error(reason)
 def request(self,msg):
  try:goal=int(json.loads(msg.data)['goal_id'])
  except (ValueError,KeyError,TypeError,json.JSONDecodeError):self.fail('INVALID_REQUEST');return
  if self.pose is None or not self.router.server_is_ready():self.fail('LOCALIZATION_OR_ROUTE_SERVER_NOT_READY');return
  self.epoch+=1;epoch=self.epoch;self.state='ROUTING';self.reason='';self.cmd.publish(Twist())
  if self.follow:self.follow.cancel_goal_async()
  self.plan_busy=False;self.visited=[];self.failures=0
  start=min(self.nodes,key=lambda n:math.hypot(self.nodes[n][0]-self.pose[0],self.nodes[n][1]-self.pose[1]))
  g=ComputeRoute.Goal();g.start_id=start;g.goal_id=goal;g.use_start=True;g.use_poses=False
  future=self.router.send_goal_async(g);future.add_done_callback(lambda f:self.route_accepted(f,epoch))
 def route_accepted(self,f,epoch):
  if epoch!=self.epoch:return
  h=f.result()
  if not h.accepted:self.fail('ROUTE_REQUEST_REJECTED');return
  h.get_result_async().add_done_callback(lambda f:self.route_result(f,epoch))
 def route_result(self,f,epoch):
  if epoch!=self.epoch:return
  response=f.result();r=response.result
  if response.status!=4 or r.error_code:self.fail(f'ROUTE_UNREACHABLE error_code={r.error_code}');return
  self.ids=[n.nodeid for n in r.route.nodes];self.required=[n for n in self.ids if n in self.cfg['junction_ids']]
  self.points=[(p.pose.position.x,p.pose.position.y) for p in r.path.poses]
  if len(self.points)<2:self.fail('EMPTY_ROUTE');return
  self.arc=[0.]
  for a,b in zip(self.points,self.points[1:]):self.arc.append(self.arc[-1]+math.dist(a,b))
  self.progress=0.;self.last_plan_progress=-99.;self.last_plan_time=0.;self.path_pub.publish(r.path)
  self.selected.publish(String(data=json.dumps(dict(nodes=self.ids,edges=[e.edgeid for e in r.route.edges],cost=r.route.route_cost,frame=r.path.header.frame_id))))
  self.state='PLANNING';self.plan()
 def target(self):
  distance=min(self.arc[-1],self.progress+self.settings['lookahead_m']);i=min(range(len(self.arc)),key=lambda i:abs(self.arc[i]-distance));a=self.points[max(0,i-1)];b=self.points[min(len(self.points)-1,i+1)];yaw=math.atan2(b[1]-a[1],b[0]-a[0])
  g=PoseStamped();g.header.frame_id='odom';g.header.stamp=self.get_clock().now().to_msg();g.pose.position.x,g.pose.position.y=self.points[i];g.pose.orientation.z=math.sin(yaw/2);g.pose.orientation.w=math.cos(yaw/2)
  return g,i==len(self.points)-1
 def plan(self):
  if self.plan_busy or not self.planner.server_is_ready():return
  goal,final=self.target();self.goal_pub.publish(goal);g=ComputePathToPose.Goal();g.goal=goal;g.planner_id='GridBased';g.use_start=False
  self.plan_busy=True;self.last_plan_time=time.monotonic();epoch=self.epoch;origin=self.pose
  self.planner.send_goal_async(g).add_done_callback(lambda f:self.plan_accepted(f,epoch,final,origin))
 def plan_accepted(self,f,epoch,final,origin):
  if epoch!=self.epoch:return
  h=f.result()
  if not h.accepted:self.plan_busy=False;self.fail('PLANNER_REJECTED');return
  h.get_result_async().add_done_callback(lambda f:self.plan_result(f,epoch,final,origin))
 def plan_result(self,f,epoch,final,origin):
  if epoch!=self.epoch or self.state=='FAILED':return
  self.plan_busy=False;r=f.result()
  if r.status!=4 or r.result.error_code:
   self.failures+=1
   if self.failures>=self.settings['max_plan_failures']:self.fail(f'LOCAL_PLANNING_FAILED error_code={r.result.error_code}')
   return
  path=r.result.path
  if not path.poses:self.fail('EMPTY_LOCAL_PATH');return
  if any(max(abs(p.pose.position.x-origin[0]),abs(p.pose.position.y-origin[1]))>self.settings['max_plan_axis_extent_m'] for p in path.poses):self.fail('PATH_OUTSIDE_LOCAL_RANGE');return
  if any(not self.on_road(p.pose.position.x,p.pose.position.y) for p in path.poses):self.fail('PATH_LEAVES_ROAD');return
  self.failures=0;self.last_plan_progress=self.progress;self.seq+=1;seq=self.seq
  g=FollowPath.Goal();g.path=path;g.controller_id='FollowPath';g.goal_checker_id='goal_checker';g.progress_checker_id='progress_checker'
  self.controller.send_goal_async(g).add_done_callback(lambda f:self.follow_accepted(f,epoch,seq,final))
 def follow_accepted(self,f,epoch,seq,final):
  if epoch!=self.epoch or seq!=self.seq:return
  h=f.result()
  if not h.accepted:self.fail('CONTROLLER_REJECTED');return
  self.follow=h;self.state='RUNNING';h.get_result_async().add_done_callback(lambda f:self.follow_result(f,epoch,seq,final))
 def follow_result(self,f,epoch,seq,final):
  if epoch!=self.epoch or seq!=self.seq or self.state=='FAILED':return
  r=f.result()
  if r.status!=4:self.fail(f'CONTROLLER_FAILED error_code={r.result.error_code}');return
  if final:
   if self.visited!=self.required:self.fail('MISSED_REQUIRED_JUNCTION');return
   self.state='FINISHED';self.cmd.publish(Twist())
  else:self.last_plan_progress=-99.;self.state='PLANNING'
 def tick(self):
  now=time.monotonic()
  if self.state in ['RUNNING','PLANNING']:
   if now-self.last_odom>self.settings['odom_timeout_s']:self.fail('ODOM_TIMEOUT')
   elif not self.body_on_road(self.pose):self.fail('VEHICLE_LEAVES_TEST_ROAD')
   elif self.points:
    candidates=[i for i,s in enumerate(self.arc) if self.progress-.5<=s<=self.progress+1.5]
    if candidates:
     i=min(candidates,key=lambda i:math.dist(self.points[i],self.pose[:2]));self.progress=max(self.progress,self.arc[i])
     if math.dist(self.points[i],self.pose[:2])>2.:self.fail('ROUTE_CORRIDOR_DEVIATION');return
    if len(self.visited)<len(self.required):
     n=self.required[len(self.visited)]
     if math.dist(self.nodes[n],self.pose[:2])<self.settings['junction_capture_m']:self.visited.append(n)
    if now-self.last_plan_time>=self.settings['update_min_s'] and (self.progress-self.last_plan_progress>=self.settings['update_progress_m'] or self.failures):self.plan()
  if self.state!='RUNNING':self.cmd.publish(Twist())
  self.status.publish(String(data=json.dumps(dict(state=self.state,reason=self.reason,progress_m=self.progress,visited_junctions=self.visited,local_path_sequence=self.seq,monotonic_s=now))))

def main():
 rclpy.init();n=Generator()
 try:rclpy.spin(n)
 finally:n.cmd.publish(Twist());n.destroy_node();rclpy.shutdown()
