Marker Pose Follow (ROS 2)
==========================

This ROS 2 package enables a **KUKA iiwa** robot to track a marker in 3D space using motion capture data (from OptiTrack). It subscribes to marker pose messages and optionally broadcasts a TF transform for visualization and planning. Integration with MoveIt2 enables the robot to follow the marker pose.

🛠️ Setup
---------

### Clone the required repositories into your workspace:

    cd ~/test_ws/src
    # Assuming these are already cloned:
    # - mocap4r2
    # - mocap4ros2_optitrack
    # - iiwa_ros2
    # - marker_pose_follow
    

### Build your workspace:

    cd ~/test_ws
    colcon build
    source install/setup.bash
    

### Install required dependencies:

    sudo apt update
    sudo apt install \
      ros-humble-moveit-common \
      ros-humble-moveit-configs \
      ros-humble-moveit-ros-planning-interface \
      python3-moveit
    

✅ Fixes Applied
---------------

*   🐛 **Fixed missing static TF between `map → world`**:  
    Without this, RViz and MoveIt2 were confused about marker frame alignment.  
    **Solution:** Run this in a separate terminal:
    
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world
    
*   🧩 Integrated marker pose tracking by broadcasting `/tracked_marker_pose` as a TF frame.
*   ⚙️ Added custom node `pose_to_tf.py` to convert `geometry_msgs/PoseStamped` → TF broadcast.
*   🔧 Updated `setup.py` to include `pose_follower` and `pose_to_tf` entry points.

🔄 Nodes
--------
| Node            | Description                                                                   |
| --------------- | ----------------------------------------------------------------------------- |
| `pose_follower` | Uses MoveIt to follow the marker pose (manual or auto).                       |
| `pose_to_tf`    | Converts `/tracked_marker_pose` into a broadcasted TF frame (`marker_frame`). |


🧪 Usage
--------

Run the following in **separate terminals**:

### ① Launch motion capture + marker tracking:

    ros2 launch mocap4r2_marker_viz marker_viz.launch.py

### ② Launch robot with MoveIt2 RViz support:

    ros2 launch lbr_bringup bringup.launch.py

### ③ (Fix) Publish missing static transform (`map → world`):

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world

### ④ Run the TF broadcaster node:

    ros2 run marker_pose_follow pose_to_tf

### ⑤ (Optional) Run the pose follower node:

    ros2 run marker_pose_follow pose_follower

📡 RViz Configuration
---------------------

*   Add Display: **TF** → Confirm `marker_frame` appears.
*   Add Display: **PoseStamped** → Set topic to `/tracked_marker_pose` (if visualizing pose directly).

Use the **Motion Planning Panel**:

*   Set **Planning Group**: `iiwa_arm`
*   Set **Goal State**: Interactive or from RViz marker
*   Use **Plan and Execute** to follow the pose
