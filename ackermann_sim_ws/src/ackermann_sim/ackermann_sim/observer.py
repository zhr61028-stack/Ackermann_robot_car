"""Draw wheel odometry and independent simulator model-pose paths in RViz."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage


class Observer(Node):
    def __init__(self):
        super().__init__('path_observer')
        self.paths = {key: Path() for key in ['odom', 'truth']}
        self.path_publishers = {key:self.create_publisher(Path,'/trajectory/'+key,10) for key in self.paths}
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(TFMessage,'/ground_truth/poses',self.truth,10)
        self.create_timer(0.1,self.publish_paths)

    def add(self,key,pose):
        path=self.paths[key]
        path.header.frame_id='odom';path.header.stamp=pose.header.stamp
        if not path.poses or pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9-(path.poses[-1].header.stamp.sec+path.poses[-1].header.stamp.nanosec*1e-9)>=0.09:
            path.poses.append(pose)
            path.poses=path.poses[-2000:]

    def odom(self,msg):
        p=PoseStamped();p.header=msg.header;p.pose=msg.pose.pose;self.add('odom',p)

    def truth(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='ackermann':
                p=PoseStamped();p.header=t.header;p.header.frame_id='odom'
                p.pose.position.x=t.transform.translation.x;p.pose.position.y=t.transform.translation.y
                p.pose.orientation=t.transform.rotation
                self.add('truth',p)

    def publish_paths(self):
        for key,path in self.paths.items():self.path_publishers[key].publish(path)


def main():
    rclpy.init();node=Observer()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
