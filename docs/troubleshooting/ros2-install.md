---
title: "Troubleshooting: ROS 2 Humble Installation"
---

# Troubleshooting: ROS 2 Humble Installation

## Symptom: `ros2: command not found`

Likely causes:

- ROS 2 is not installed.
- You installed ROS 2 but did not source the environment.

Fix:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

If sourcing fixes it, persist it:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Symptom: `colcon: command not found`

Install colcon tooling:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions
```

## Symptom: `rosdep init` fails

You need admin permissions for `rosdep init`:

```bash
sudo rosdep init
rosdep update
```

## Symptom: talker/listener don’t see each other

Quick checks:

```bash
echo $ROS_DOMAIN_ID
ros2 node list
ros2 topic list
```

Common causes:

- different `ROS_DOMAIN_ID` between terminals
- DDS blocked by firewall/network policies (rare on single machine)

