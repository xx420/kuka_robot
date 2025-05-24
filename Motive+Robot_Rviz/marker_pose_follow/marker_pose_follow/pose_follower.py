# pose_follower.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MoveGroupInterface
from moveit.planning_interface_planning_scene_monitor import PlanningSceneMonitor
from moveit_msgs.msg import MoveItErrorCodes


class PoseFollower(Node):
    def __init__(self):
        super().__init__('pose_follower')
        self.moveit = MoveItPy(node_name="moveit_py_node")
        self.move_group = MoveGroupInterface(self.moveit, 'iiwa_arm')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/tracked_marker_pose',  # From marker_pose_publisher.py
            self.pose_callback,
            10
        )
        self.get_logger().info("PoseFollower initialized and subscribed to /tracked_marker_pose")

    def pose_callback(self, pose_stamped):
        self.get_logger().info("Received marker pose, planning motion...")
        self.move_group.set_pose_target(pose_stamped.pose)

        plan_result = self.move_group.plan()
        if plan_result:
            result = self.move_group.execute()
            if result == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Motion executed successfully!")
            else:
                self.get_logger().warn(f"Execution failed with error code: {result}")
        else:
            self.get_logger().warn("Planning failed.")


def main(args=None):
    rclpy.init(args=args)
    node = PoseFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

