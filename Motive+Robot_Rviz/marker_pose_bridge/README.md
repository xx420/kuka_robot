marker\_pose\_follow ROS 2 Package
==================================

Overview
--------

This ROS 2 package listens to the `/rigid_bodies` topic (published by mocap4r2) and republishes the pose of a specific tracked marker as a `geometry_msgs/msg/PoseStamped` message on the `/marker_pose` topic. This pose can be visualized in RViz and optionally used for motion planning.

Directory Structure
-------------------

ros2_ws/src/marker\_pose\_follow/  
├── marker\_pose\_follow/  
│ ├── \_\_init\_\_.py  
│ └── marker\_pose\_publisher.py  
├── package.xml  
└── setup.py

marker\_pose\_publisher.py
--------------------------

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from mocap4r2_msgs.msg import RigidBodies
    
    class MarkerPosePublisher(Node):
        def __init__(self):
            super().__init__('marker_pose_publisher')
            self.publisher = self.create_publisher(PoseStamped, '/marker_pose', 10)
            self.subscription = self.create_subscription(RigidBodies, '/rigid_bodies', self.rigid_body_callback, 10)
            self.get_logger().info("marker_pose_publisher node has started.")
    
        def rigid_body_callback(self, msg):
            for body in msg.rigid_bodies:
                if body.name == 'marker':  # ← Replace with your actual marker name
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.pose = body.pose
                    self.publisher.publish(pose_msg)
                    self.get_logger().info("Published /marker_pose")
    
    def main(args=None):
        rclpy.init(args=args)
        node = MarkerPosePublisher()
        rclpy.spin(node)
        rclpy.shutdown()
    
    if __name__ == '__main__':
        main()
      

setup.py
--------

    from setuptools import setup
    
    package_name = 'marker_pose_follow'
    
    setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='rosuser',
        maintainer_email='imtiajsayem5@gmail.com',
        description='Publishes PoseStamped messages based on mocap rigid body data',
        license='MIT',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'marker_pose_publisher = marker_pose_follow.marker_pose_publisher:main',
            ],
        },
    )

Build and Run
-------------

### Build

    cd ~/ros2_ws
    colcon build --packages-select marker_pose_follow
    source install/setup.bash

### Run the Node

    ros2 run marker_pose_follow marker_pose_publisher

### Expected Output

    [INFO] [marker_pose_publisher]: Published /marker_pose
    [INFO] [marker_pose_publisher]: Published /marker_pose
    ...

Visualize in RViz
-----------------

*   Open RViz (via your MoveIt launch or standalone)
*   Add a **Pose → PoseStamped** display
*   Set topic to `/marker_pose`
*   Set the correct **Fixed Frame** (e.g. `map` or `world`)

Static Transform in a new Terminal (Optional)
---------------------------------------------

If needed, to bridge frames for TF:

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world

Replace `map` and `world` with actual frames as required by your setup.
