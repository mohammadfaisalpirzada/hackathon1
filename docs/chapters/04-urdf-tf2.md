---
title: "Chapter 4 — URDF/Xacro + TF2 (Robot Model and Frames)"
---

# Chapter 4 - URDF/Xacro + TF2 (Robot Model and Frames)

## Learning Objectives

- Build a minimal URDF/Xacro robot model with links and joints.
- Explain common TF frame conventions (`map`, `odom`, `base_link`, sensor frames).
- Visualize the robot model and frames in RViz and diagnose TF issues.

## Key Terms

- URDF, Xacro, link, joint, TF2, transform, frame, robot_state_publisher

## Prerequisites

- ROS 2 Humble Desktop installed (includes `rviz2`).
- Install helpers (if missing):

```bash
sudo apt update
sudo apt install -y ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

## Concepts

### URDF vs Xacro

- **URDF**: XML robot description; links + joints + visuals/collisions/inertials.
- **Xacro**: “URDF with macros”; use it to avoid copy/paste and parameterize dimensions.

### TF2 (why frames are a first-class problem)

Your robot is many coordinate frames. TF2 answers:

- “Where is `base_link` relative to `map` right now?”
- “How do I transform a point from camera frame to base frame?”

Common frames you’ll see:

```text
map -> odom -> base_link -> base_footprint -> sensors (camera_link, lidar_link, ...)
```

## Hands-on Lab: build a minimal "minibot" URDF and visualize in RViz

### 1) Create a description package

```bash
cd ~/q2_ws/src
ros2 pkg create q2_minibot_description --build-type ament_cmake
mkdir -p q2_minibot_description/urdf q2_minibot_description/launch
```

### 2) Create a Xacro file

Create `~/q2_ws/src/q2_minibot_description/urdf/minibot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="minibot">
  <xacro:property name="wheel_r" value="0.05"/>
  <xacro:property name="wheel_sep" value="0.30"/>

  <link name="base_link">
    <visual>
      <geometry><box size="0.35 0.25 0.12"/></geometry>
      <material name="blue"><color rgba="0.15 0.39 0.92 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.35 0.25 0.12"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="left_wheel_link">
    <visual><geometry><cylinder length="0.02" radius="${wheel_r}"/></geometry></visual>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 ${wheel_sep/2} -0.05" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="right_wheel_link">
    <visual><geometry><cylinder length="0.02" radius="${wheel_r}"/></geometry></visual>
  </link>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -${wheel_sep/2} -0.05" rpy="0 1.5708 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="lidar_link">
    <visual><geometry><cylinder length="0.03" radius="0.04"/></geometry></visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.08" rpy="0 0 0"/>
  </joint>
</robot>
```

### 3) Add a launch file to publish robot_description + TF

Create `~/q2_ws/src/q2_minibot_description/launch/view_minibot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("q2_minibot_description"), "urdf", "minibot.urdf.xacro"]
    )
    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(package="rviz2", executable="rviz2"),
        ]
    )
```

### 4) Wire CMake install rules

Edit `~/q2_ws/src/q2_minibot_description/CMakeLists.txt` (add at end):

```cmake
install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)
```

### 5) Build and run

```bash
cd ~/q2_ws
colcon build --symlink-install
source ~/q2_ws/install/setup.bash
ros2 launch q2_minibot_description view_minibot.launch.py
```

In RViz:

- Add “RobotModel”
- Set "Fixed Frame" to `base_link`

## Lab Deliverable

- A package `q2_minibot_description` whose robot renders in RViz with a coherent TF tree and consistent fixed frame selection.

## Assessment Item

- Quiz: explain why `map -> odom -> base_link` is preferred over `map -> base_link` directly in most navigation setups.

## Troubleshooting

- `xacro: command not found`
  - Install: `sudo apt install ros-humble-xacro`
- Robot doesn’t show up in RViz
  - You didn’t add “RobotModel”, or “Fixed Frame” is wrong.
- TF tree is confusing
  - Use: `ros2 run tf2_tools view_frames` (generates a PDF in the current directory).

## Quick Quiz

1. Why use `odom -> base_link` instead of `map -> base_link` directly?
2. What is the difference between a fixed joint and a continuous joint?
3. What ROS node turns URDF joint states into TF transforms?
