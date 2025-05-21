# MoveIt with RViz for iiwa14

This guide explains how to run MoveIt using RViz for the **iiwa14** robot model with the `lbr_bringup` package.

![image](https://github.com/user-attachments/assets/8719ee2a-cfb9-4250-b1fd-4950c01b1bc4)

**IIWA 7 R800 in RViz**
<br>

## Prerequisites

Make sure you have:

- ROS 2 installed and sourced
- The `lbr_bringup` package available in your workspace
- Proper ROS environment configuration

---

## 1. Run the Mock Setup

```
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa14
```
## 2. Launch MoveIt with RViz

```
ros2 launch lbr_bringup move_group.launch.py mode:=mock rviz:=true model:=iiwa14
```
## Usage

Once RViz is open:

- Use the `MotionPlanning` panel in RViz
- Plan and execute trajectories for the **iiwa14** robot
- Interact using goal pose markers or set joint values manually
