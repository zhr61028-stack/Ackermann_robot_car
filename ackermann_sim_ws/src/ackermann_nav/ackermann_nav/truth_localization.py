"""TEST ONLY. Direct simulator pose in odom; no localization algorithm validated."""
import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
class TruthLocalization(Node):
 def __init__(self):
  super().__init__('TEST_truth_localization');self.previous=None
  self.pub=self.create_publisher(Odometry,'/odom',10);self.tf=TransformBroadcaster(self)
  self.create_subscription(TFMessage,'/ground_truth/poses',self.receive,20)
  self.get_logger().warning('TEST MODE: Gazebo true pose drives odom + TF. NOT real localization.')
 def receive(self,m):
  for tr in m.transforms:
   if tr.child_frame_id!='ackermann':continue
   p=tr.transform.translation;q=tr.transform.rotation
   yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));t=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9
   o=Odometry();o.header.stamp=tr.header.stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint'
   o.pose.pose.position.x=p.x;o.pose.pose.position.y=p.y;o.pose.pose.orientation=q
   if self.previous:
    pt,px,py,pa=self.previous;dt=t-pt
    if dt<=0:continue
    vx=(p.x-px)/dt;vy=(p.y-py)/dt
    o.twist.twist.linear.x=math.cos(yaw)*vx+math.sin(yaw)*vy
    o.twist.twist.linear.y=-math.sin(yaw)*vx+math.cos(yaw)*vy
    o.twist.twist.angular.z=math.atan2(math.sin(yaw-pa),math.cos(yaw-pa))/dt
   self.previous=(t,p.x,p.y,yaw)
   tf=TransformStamped();tf.header=o.header;tf.child_frame_id=o.child_frame_id
   tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.rotation=q
   self.pub.publish(o);self.tf.sendTransform(tf)
def main():
 rclpy.init();n=TruthLocalization()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
