---
title: "Lab (Module 4): Voice/LLM-Style Planning to ROS Actions"
---

# Lab (Module 4): Voice/LLM-Style Planning to ROS Actions

## Prerequisites

- ROS 2 Humble.
- Nav2 available (recommended: complete Chapter 8 lab first).

## Goal

Demonstrate a safe “VLA-style” pipeline where a user command becomes a validated plan and then triggers a robot action.

## Deliverable

- A package `q2_vla` in `~/q2_ws/src/` containing:
  - a command input node
  - a planner/validator node that emits a schema-constrained plan
  - an executor node that triggers a ROS action (Nav2 navigate-to-pose)
- Evidence: recorded terminal output showing a command and a corresponding plan + action invocation.

## Steps

Follow Chapter 9 to implement:

- `/vla/command` topic (text input)
- `/vla/plan` topic (validated JSON plan)
- Nav2 action client to `navigate_to_pose`

## Verification

With Nav2 running, verify:

```bash
ros2 topic list | rg vla
ros2 topic echo /vla/plan --once
ros2 action list -t | rg NavigateToPose
```

## Assessment

- Provide one “unsafe” example command and explain how your validator/executor prevents it from executing.

