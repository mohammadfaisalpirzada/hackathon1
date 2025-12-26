---
title: "Setup: Ubuntu 22.04 + ROS 2 Humble + Simulation"
sidebar_position: 2
---

# Setup: Ubuntu 22.04 + ROS 2 Humble + Simulation

## Prerequisites

- A machine you can install packages on (admin access).
- Ubuntu 22.04 (native install strongly preferred for simulation performance).

## Hardware requirements (explicit)

Minimum (works for ROS 2 + RViz + basic Gazebo scenes):

- CPU: 4 cores (x86_64)
- RAM: 16 GB
- Disk: 30 GB free
- GPU: integrated is OK (RViz may be slower)

Recommended (smooth simulation + Isaac Sim experiments):

- CPU: 8+ cores
- RAM: 32–64 GB
- Disk: 100 GB free (datasets + caches + containers)
- GPU: NVIDIA RTX-class, 8+ GB VRAM (12-16 GB strongly recommended for Isaac Sim)

See also:

- Hardware + lab architectures (on-prem vs Ether lab): `/reference/hardware-lab-architectures`
- On-prem BOM: `/reference/on-prem-lab-bom`
- Ether lab BOM/cost model: `/reference/ether-lab-bom`

## Concepts

- ROS 2 is installed system-wide under `/opt/ros/humble`.
- Your project code lives in a workspace, e.g. `~/q2_ws`.
- You will use `colcon` to build ROS packages.

## Hands-on Lab: install ROS 2 Humble (apt)

Official packages assume Ubuntu Jammy (22.04).

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

Tooling:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
```

Shell setup:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Hands-on Lab: create your ROS workspace

```bash
mkdir -p ~/q2_ws/src
cd ~/q2_ws
colcon build
echo "source ~/q2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Hands-on Lab: install Gazebo (classic) integration

Assumption: we use Gazebo Classic here because it’s the simplest path for Humble labs.

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

Quick test:

```bash
gazebo --version
```

## Troubleshooting

- `rosdep init` fails with permissions
  - You didn’t use `sudo` for `rosdep init`.
- `colcon: command not found`
  - Install: `sudo apt install python3-colcon-common-extensions`
- Gazebo opens but is blank/slow
  - Laptop GPU: reduce rendering (Gazebo “View → FPS”, lower resolution), close other apps.
  - On Wayland issues: try Xorg session.

## Quick Quiz

1. What’s the difference between `/opt/ros/humble` and `~/q2_ws/install`?
2. Why do we source two setup files (ROS + workspace)?
3. What’s the fastest way to confirm Gazebo is installed correctly?
