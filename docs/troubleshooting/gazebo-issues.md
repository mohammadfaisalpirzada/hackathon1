---
title: "Troubleshooting: Gazebo Classic (ROS 2 Humble)"
---

# Troubleshooting: Gazebo Classic (ROS 2 Humble)

## Symptom: Gazebo is blank/slow

Common causes:

- integrated GPU struggling with rendering
- running under a compositor that reduces performance

Mitigations:

- reduce rendering settings / FPS display
- close other GPU-heavy applications
- prefer native Ubuntu install over VM

## Symptom: robot spawns but doesn’t move

Diagnosis steps:

```bash
ros2 topic list | rg cmd_vel
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once
```

Common causes:

- diff-drive plugin not loaded (check Gazebo console output)
- wrong joint names in plugin config
- `/cmd_vel` topic name mismatch (namespace issues)

## Symptom: odometry/TF looks wrong

Common causes:

- wrong wheel separation/diameter values
- inverted wheel joint axis
- `use_sim_time` mismatch between nodes

Checks:

```bash
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo odom base_link
```

