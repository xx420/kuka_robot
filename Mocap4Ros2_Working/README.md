# mocap4r2_ws - OptiTrack Motion Capture Integration for ROS2

This workspace integrates the [mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack) package into a ROS2 environment for streaming OptiTrack motion capture data.

## Workspace Setup

### 1. Create and initialize the workspace

```
mkdir -p ~/mocap4r2_ws/src
cd ~/mocap4r2_ws/src
```

2. Clone the required repository
```
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
```
3. Install dependencies

Make sure you have ros-humble-desktop or equivalent ROS2 version installed.

```cd ~/mocap4r2_ws
rosdep install --from-paths src --ignore-src -r -y
```
4. Build the workspace
```
source /opt/ros/humble/setup.bash
colcon build
```
5. Source the workspace
```
source install/setup.bash
```
You can add the above line to your .bashrc or create a shell alias.
Configuration
```
echo 'source ~/mocap4r2_ws/install/setup.bash' >> ~/.bashrc
```
Then source it
```
source ~/.bashrc
```
Edit the configuration file (usually a .yaml file) to match your OptiTrack setup.
Example Configuration (used in this project)
```
mocap4r2_optitrack_driver_node:

  ros__parameters:

    # Connection settings

    connection_type: "Multicast"  # Multicast / Unicast

    server_address: "172.31.1.146"  # Motive PC

    local_address: "172.31.1.149"   # ROS2 machine

    multicast_address: "239.255.42.99"

    server_command_port: 1510

    server_data_port: 1511
 
    # Rigid body settings (you can customize as needed)

    rigid_body_name: "ground"

    lastFrameNumber: 0

    frameCount: 0

    droppedFrameCount: 0

    n_markers: 0

    n_unlabeled_markers: 0
 
    # QoS settings

    qos_history_policy: "keep_all"         # keep_all / keep_last

    qos_reliability_policy: "best_effort"  # best_effort / reliable

    qos_depth: 10                          # 10 / 100 / 1000
 
    # Verbose settings (optional additions from optitrack_wrapper_node)

    verbose_data_description: false

    verbose_frame: false

    topic_frame_data: "frame_data"
```
Make sure the server_address and local_address match your network setup.
Running the Node
```
ros2 run mocap4ros2_optitrack mocap4ros2_optitrack_driver_node --ros-args --params-file path/to/your_config.yaml
```

Launching the OptiTrack System

To launch the driver using ROS2 launch files:
```
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```
Check that the OptiTrack configuration works and the system connects.

Since the driver node is a lifecycle node, you need to activate it after launch:
```
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
Visualizing in RViz

You can visualize marker data in RViz using:
```
ros2 launch mocap4r2_marker_viz mocap4r2_marker_viz.launch.py mocap4r2_system:=optitrack
```



## Troubleshooting Tips

- **Cannot connect to server:**  
  Ensure both your machine and the OptiTrack server are on the same network. Also, verify that ports **1510** (command) and **1511** (data) are open and not blocked by a firewall.

- **No data received:**  
  Make sure **Motive** is streaming data using the **NatNet** protocol.

- **Multicast not working:**  
  Try switching the `connection_type` to `"Unicast"` and confirm that the `server_address` and `local_address` are set correctly.



