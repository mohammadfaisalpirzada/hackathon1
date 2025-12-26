---
title: "ROS 2 Cheat Sheet (Humble)"
---

# ROS 2 Cheat Sheet (Humble)

## Prerequisites

- ROS 2 installed and sourced.

## Concepts

- Most debugging is “what exists right now?” and “what is it publishing right now?”.

## Hands-on Lab: the 90-second debug routine

```bash
ros2 node list
ros2 topic list
ros2 service list -t
ros2 action list -t
```

Inspect a topic:

```bash
ros2 topic info /topic_name -v
ros2 topic echo /topic_name --once
ros2 topic hz /topic_name
```

Inspect interfaces:

```bash
ros2 interface list | rg nav2
ros2 interface show geometry_msgs/msg/Twist
```

Parameters:

```bash
ros2 param list /node_name
ros2 param get /node_name some_param
ros2 param set /node_name some_param 123
```

## Troubleshooting

- “It runs but nothing happens”
  - You’re publishing to the wrong topic name or namespace.
  - QoS mismatch (especially for sensor topics).
  - Time mismatch (`use_sim_time`).

## Quick Quiz

1. What command shows you a topic’s QoS settings?
2. How do you confirm a node actually exists under a namespace?
3. What’s the difference between `ros2 topic echo` and `ros2 topic hz`?

