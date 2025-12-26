---
title: "Glossary"
---

# Glossary

## Prerequisites

- None.

## Concepts

- **DDS**: Data Distribution Service; the middleware ROS 2 uses (via an RMW implementation).
- **TF/TF2**: transform tree for coordinate frames over time.
- **URDF**: robot model description (links + joints).
- **Xacro**: macro language for URDF.
- **Nav2**: ROS 2 navigation framework (planning + control + recovery behaviors).
- **Action**: ROS primitive for long-running goals with feedback/result.
- **QoS**: message delivery policy (reliability, durability, depth, etc.).
- **Sim time**: using `/clock` from simulation instead of wall time (`use_sim_time:=True`).

## Hands-on Lab

Pick any unfamiliar term and locate it in ROS docs or a running system:

```bash
ros2 topic list
ros2 param list
```

## Troubleshooting

- Overloaded vocabulary
  - Build a mental map: *data* (topics) vs *commands* (topics/services/actions) vs *frames* (TF).

## Quick Quiz

1. What problem does TF2 solve that “just publishing poses” does not?
2. When is QoS the root cause of missing messages?
3. Why does Nav2 prefer an action interface for navigation?

