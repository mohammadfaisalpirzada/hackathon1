---
title: "Chapter 3 — rclpy Patterns (Timers, QoS, Params, Composition)"
---

# Chapter 3 - rclpy Patterns (Timers, QoS, Params, Composition)

## Learning Objectives

- Structure ROS nodes around short callbacks and non-blocking timers.
- Explain QoS tradeoffs (reliability/durability/depth) and pick profiles for sensors vs commands.
- Verify QoS behavior using ROS CLI tools (including late-joiner behavior).

## Key Terms

- callback, timer, QoS, reliability, durability, history depth, transient local, best effort

## Prerequisites

- You can create and run `ament_python` packages.
- You know how to inspect topics (`ros2 topic echo`, `ros2 topic hz`).

## Concepts

### Timers + callbacks (why structure matters)

ROS nodes are event-driven. You typically have:

- subscription callbacks (incoming data)
- timer callbacks (periodic work)
- service/action callbacks (requests/goals)

Rule: keep callbacks short. If a callback blocks, your node becomes unresponsive.

### QoS (why “same topic name” is not enough)

QoS controls how DDS delivers messages. Common knobs:

- **reliability**: `RELIABLE` vs `BEST_EFFORT`
- **durability**: `VOLATILE` vs `TRANSIENT_LOCAL` (late-joiners get last message)
- **history/depth**: queue size

Typical patterns:

- Sensors (camera/LiDAR): `BEST_EFFORT`, small depth
- Commands/state: `RELIABLE`
- Static config: `TRANSIENT_LOCAL`

## Hands-on Lab: latched config topic + fast sensor topic

Goal: publish a "latched" config once, and publish a fast stream separately.

## Lab Deliverable

- A package `q2_qos` that publishes `/q2/config` as TRANSIENT_LOCAL and `/q2/fast` at high rate, with a recorded verification showing late-join behavior for `/q2/config`.

## Assessment Item

- Practical check: show `ros2 topic info /q2/config -v` and explain why that QoS is good for config but dangerous for high-rate sensor streams.

### 1) Create package

```bash
cd ~/q2_ws/src
ros2 pkg create q2_qos --build-type ament_python --dependencies rclpy std_msgs
```

### 2) Config publisher (TRANSIENT_LOCAL)

Create `~/q2_ws/src/q2_qos/q2_qos/config_pub.py`:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ConfigPublisher(Node):
    def __init__(self) -> None:
        super().__init__("config_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(String, "/q2/config", qos)
        msg = String()
        msg.data = "mode=training; max_speed=1.0"
        self.pub.publish(msg)
        self.get_logger().info("Published latched config once; keep node running for discovery.")


def main() -> None:
    rclpy.init()
    node = ConfigPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

### 3) Fast publisher (BEST_EFFORT)

Create `~/q2_ws/src/q2_qos/q2_qos/fast_pub.py`:

```python
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FastPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fast_publisher")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(String, "/q2/fast", qos)
        self.timer = self.create_timer(0.01, self.on_tick)  # 100 Hz
        self.counter = 0

    def on_tick(self) -> None:
        self.counter += 1
        msg = String()
        msg.data = f"{self.counter} {time.time():.3f}"
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(FastPublisher())


if __name__ == "__main__":
    main()
```

### 4) Register + build + run

Edit `~/q2_ws/src/q2_qos/setup.py`:

```python
entry_points={
    "console_scripts": [
        "config_pub = q2_qos.config_pub:main",
        "fast_pub = q2_qos.fast_pub:main",
    ],
},
```

Build:

```bash
cd ~/q2_ws
colcon build --symlink-install
source ~/q2_ws/install/setup.bash
```

Run config publisher:

```bash
# Terminal A
ros2 run q2_qos config_pub
```

Now start a late-joining echo:

```bash
# Terminal B (start this after A is already running)
ros2 topic echo /q2/config --once
```

Measure fast topic rate:

```bash
ros2 topic hz /q2/fast
```

## Troubleshooting

- Late-joining `echo` shows nothing for `/q2/config`
  - You used default QoS somewhere. Verify durability via:
    `ros2 topic info /q2/config -v`
- `/q2/fast` rate is much lower than expected
  - Your machine is loaded; reduce to 50 Hz and confirm.
  - Python timers are not hard real-time; don’t assume exact periods.

## Quick Quiz

1. Why is `TRANSIENT_LOCAL` useful for “config” but dangerous for high-rate topics?
2. When would you pick `BEST_EFFORT` over `RELIABLE`?
3. What is the practical difference between “timer-based” and “event-based” work?
