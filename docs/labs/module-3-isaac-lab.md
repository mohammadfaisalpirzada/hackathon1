---
title: "Lab (Module 3): Isaac Track (Synthetic Data / VSLAM Pipeline)"
---

# Lab (Module 3): Isaac Track (Synthetic Data / VSLAM Pipeline)

## Prerequisites

- ROS 2 Humble.
- For the optional Isaac track: access to a GPU-capable environment (local NVIDIA GPU or Ether lab) and a working Isaac Sim installation.

## Goal

Produce a localization-like stream suitable for downstream navigation, using either:

- a baseline ROS pipeline (runnable everywhere), or
- an Isaac-enhanced pipeline (optional/advanced).

## Deliverable

Choose one:

- **Baseline deliverable (required)**: a running camera pipeline with measurable topic rate and a recorded bag file (or logged evidence) showing topic activity.
- **Isaac deliverable (optional)**: a documented run producing a pose estimate stream and a short note on latency/throughput observations.

## Baseline track (runnable): camera + processing + evidence

### 1) Publish a camera stream

```bash
source /opt/ros/humble/setup.bash
ros2 run image_tools cam2image --ros-args -p frequency:=10.0
```

### 2) Add a standard processing node

```bash
source /opt/ros/humble/setup.bash
ros2 run image_proc rectify_node --ros-args -r image:=/image -r camera_info:=/camera_info
```

### 3) Measure rates and record evidence

```bash
ros2 topic hz /image
ros2 topic list | rg image
```

Optional: record a short bag:

```bash
mkdir -p ~/q2_ws/bags
cd ~/q2_ws/bags
ros2 bag record -o isaac-track-baseline /image /camera_info
```

## Isaac track (optional): swap in accelerated nodes / synthetic sources

If you have Isaac Sim + Isaac ROS available, keep the message contract stable:

- inputs remain image-like topics (plus camera info)
- outputs include a pose-like estimate appropriate for navigation integration

Record:

- what you replaced
- what topics changed (if any)
- how you verified latency/throughput change (timestamps + `ros2 topic hz`)

## Assessment

- Explain the difference between “FPS increased” and “end-to-end latency improved” in one paragraph.

