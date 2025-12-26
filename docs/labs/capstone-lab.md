---
title: "Lab (Capstone): End-to-End Autonomous Pipeline"
---

# Lab (Capstone): End-to-End Autonomous Pipeline

## Prerequisites

- A runnable sim environment (Gazebo recommended for the core path).
- Nav2 works in simulation (or you have an alternative locomotion controller).
- You can build and run `rclpy` packages in `~/q2_ws`.

## Goal

Deliver a single “bringup” command that launches the entire system end-to-end and supports at least three high-level commands reliably.

## Deliverable

- A ROS workspace layout under `~/q2_ws/src/` containing (names may vary):
  - `<project>_description` (URDF/Xacro)
  - `<project>_sim` (worlds + simulation glue)
  - `<project>_bringup` (launch + parameters)
  - `<project>_vla` (command → plan → validated actions)
- A one-command bringup:
  - `ros2 launch <project>_bringup bringup.launch.py`
- Evidence:
  - a short screen recording or log transcript showing bringup, TF visualization, and successful command execution

## Evaluation checklist (rubric gates)

1. **Reproducibility**: bringup works from a clean terminal after sourcing workspace.
2. **Observability**: key topics/actions exist and can be inspected via CLI.
3. **Reliability**: 3 high-level commands succeed ≥ 80% over 10 attempts.
4. **Safety**: invalid/ambiguous commands do not reach actuators.

