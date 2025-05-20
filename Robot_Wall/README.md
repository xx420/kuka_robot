# iiwa14 URDF Modification and Visualization

## Overview

This repository contains an updated URDF description of the KUKA iiwa14 robot integrated into the `lbr_fri_ros2_stack`. The main change is attaching the robot to a **wall** that is fixed to the ground, simulating the robot being mounted on a wall (e.g., ceiling or elevated structure), rather than directly to the ground.

## Changes Made

- Added a new `<link>` named `wall` representing a fixed wall structure with defined mass and inertia.

- Created a fixed joint `joint_wall_ground` to attach the wall link to the `world` link (ground).

- Modified the robot's base joint from `robot_name_world_joint` to `robot_name_wall_joint` to attach the robot's base link (`robot_name_link_0`) to the wall instead of directly to the ground.

- Positioned and oriented the robot to be centered on the wall's front face and parallel to the ground (using appropriate `xyz` and `rpy` in the joint origin).

- Maintained inclusion of the original iiwa14 robot macro for model details and ros2_control config.

### Key positioning details in the new joint:

```xml
<joint name="$(arg robot_name)_wall_joint" type="fixed">
  <parent link="wall" />
  <child link="$(arg robot_name)_link_0" />
  <origin xyz="0 0.05 1.0" rpy="1.5708 0 0" /> <!-- Rotated 90 degrees on X to be parallel and offset above ground -->
</joint>
```
This sets the robot 1 meter above ground, slightly offset in Y, and rotated to face forward horizontally.

## Build Instructions

Make sure your ROS2 workspace (`ros2_ws`) is properly sourced and includes the `lbr_fri_ros2_stack` package.

```bash
cd ~/ros2_ws
colcon build --packages-select lbr_description
source install/setup.bash
```

## Visualization

To visualize the updated robot model in RViz:

1. Launch your ROS2 environment:

```bash
ros2 launch lbr_description display.launch.py
```
2. Confirm that the robot is attached to a vertical wall fixed to the ground, positioned roughly 1m above the ground level, facing forward horizontally.

## Notes

- 🧱 The `wall` link acts as an intermediate fixed frame simulating a mounting surface.

- 🔗 The original base link is no longer attached to `world` but to `wall`.

- ⚙️ Adjust `xyz` and `rpy` in the joint as needed for your exact mounting scenario.
