---
title: "Lab (Module 1): ROS 2 Primitives Mini-System"
---

# Lab (Module 1): ROS 2 Primitives Mini-System

## Prerequisites

- ROS 2 Humble installed and sourced.
- A workspace exists at `~/q2_ws`.

## Goal

Build a tiny ROS 2 “system” that demonstrates the four primitives you will use throughout the course:

- Topic: heartbeat stream
- Service: reset counter (request/response)
- Parameters: publish rate
- Action: understand the interface shape (we’ll use Nav2 actions later)

## Deliverable

- A package named `q2_ros2_basics` in `~/q2_ws/src/` with:
  - a heartbeat publisher node
  - a reset service node
  - evidence (commands + outputs) showing the topic publishes and the service resets state

## Steps

### 1) Create the package

```bash
cd ~/q2_ws/src
ros2 pkg create q2_ros2_basics --build-type ament_python --dependencies rclpy std_msgs std_srvs
```

### 2) Implement the heartbeat publisher

Follow Chapter 2’s heartbeat node pattern in your package:

- publish to `/q2/heartbeat`
- use a parameter `publish_rate_hz`

### 3) Implement the reset service

Follow Chapter 2’s reset service pattern:

- provide `/q2/reset_counter` of type `std_srvs/srv/Trigger`

### 4) Register console scripts

Update `~/q2_ws/src/q2_ros2_basics/setup.py` to register both nodes as entry points.

### 5) Build and run

```bash
cd ~/q2_ws
colcon build --symlink-install
source ~/q2_ws/install/setup.bash
```

Run in two terminals:

```bash
ros2 run q2_ros2_basics heartbeat --ros-args -p publish_rate_hz:=5.0
```

```bash
ros2 run q2_ros2_basics reset_service
```

## Verification

Confirm the topic exists and is publishing:

```bash
ros2 topic list | rg q2
ros2 topic echo /q2/heartbeat --once
```

Confirm the service exists and can be called:

```bash
ros2 service list -t | rg reset_counter
ros2 service call /q2/reset_counter std_srvs/srv/Trigger "{}"
```

## Assessment

- Explain in 3 sentences: when would you use an action instead of a service?

