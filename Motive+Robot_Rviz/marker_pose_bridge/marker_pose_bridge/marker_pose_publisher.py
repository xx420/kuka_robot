import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies


class MarkerPosePublisher(Node):
    def __init__(self):
        super().__init__('marker_pose_publisher')
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rigid_bodies_callback,
            10
        )
        self.pose_pub = self.create_publisher(PoseStamped, '/marker_pose', 10)

    def rigid_bodies_callback(self, msg):
        for rb in msg.rigidbodies:
            if rb.rigid_body_name == '2':
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = rb.pose
                self.pose_pub.publish(pose_msg)
                self.get_logger().info('Published /marker_pose')

def main():
    rclpy.init()
    node = MarkerPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

