---
title: "Lab (Module 2): Gazebo Digital Twin Bringup"
---

# Lab (Module 2): Gazebo Digital Twin Bringup

## Prerequisites

- Gazebo Classic integration installed (`ros-humble-gazebo-ros-pkgs`).
- You completed Chapter 4 and have `q2_minibot_description` (URDF/Xacro).

## Goal

Spawn a robot in Gazebo, drive it with `/cmd_vel`, and verify odometry on `/odom` with correct simulation time behavior.

## Deliverable

- `q2_minibot_description` contains a Gazebo-ready Xacro and a Gazebo spawn launch file.
- Evidence that:
  - `/cmd_vel` controls the robot
  - `/odom` publishes while the robot moves
  - `use_sim_time:=True` is in effect for sim nodes

## Steps

Follow Chapter 5 to:

1. Create `minibot.gazebo.urdf.xacro` with a diff-drive plugin.
2. Add `spawn_minibot_gazebo.launch.py` to bring up Gazebo + spawn the robot.
3. Build your workspace and launch the sim.

## Verification

In a new terminal, confirm sim time and topics:

```bash
ros2 topic list | rg -e cmd_vel -e odom -e clock
ros2 topic echo /odom --once
ros2 topic echo /clock --once
```

Confirm you can drive the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
ros2 topic echo /odom --once
```

## Assessment

- Describe one failure mode that causes “robot spawns but doesn’t move” and how you would diagnose it using ROS tools.

