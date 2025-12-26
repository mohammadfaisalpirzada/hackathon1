---
title: "Chapter 2 — ROS 2 Fundamentals (Nodes, Topics, Services, Actions)"
---

# Chapter 2 - ROS 2 Fundamentals (Nodes, Topics, Services, Actions)

## Learning Objectives

- Distinguish when to use topics vs services vs actions in robotics systems.
- Use ROS CLI introspection tools to diagnose “what exists” and “what’s happening now”.
- Implement a small `rclpy` package containing a publisher, a service, and a parameter.

## Key Terms

- node, topic, service, action, parameter, message type, introspection

## Prerequisites

- ROS 2 Humble installed and sourced.
- You can build a workspace with `colcon`.

## Concepts

### Mental model

- **Node**: a process with ROS interfaces (publish/subscribe/service/action/params).
- **Topic**: streaming data (pub/sub). Example: `/camera/image_raw`.
- **Service**: request/response (RPC). Example: “reset”.
- **Action**: long-running goal with feedback + result. Example: “navigate to pose”.
- **Parameters**: runtime configuration (typed key/value).

### How you debug ROS 2 systems (why the CLI matters)

Start with observability:

```bash
ros2 node list
ros2 topic list
ros2 topic info /some/topic -v
ros2 topic echo /some/topic --once
ros2 interface show geometry_msgs/msg/Twist
```

## Hands-on Lab: build all four primitives in one package (`rclpy`)

We'll create:

- a topic publisher (`/q2/heartbeat`)
- a service (`/q2/reset_counter`)
- a parameter (`publish_rate_hz`)
- an action client (Nav2 later; here we stub the shape)

## Lab Deliverable

- A package `q2_ros2_basics` that publishes `/q2/heartbeat` and serves `/q2/reset_counter`, verifiable via `ros2 topic echo` and `ros2 service call`.

## Assessment Item

- Quiz: given three tasks (telemetry stream, reset command, “navigate to pose”), choose topic/service/action and justify.

### 1) Create package

```bash
cd ~/q2_ws/src
ros2 pkg create q2_ros2_basics --build-type ament_python --dependencies rclpy std_msgs std_srvs
```

### 2) Heartbeat publisher with a parameter

Create `~/q2_ws/src/q2_ros2_basics/q2_ros2_basics/heartbeat.py`:

```python
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__("heartbeat")
        self.pub = self.create_publisher(String, "/q2/heartbeat", 10)
        self.rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_tick)
        self.counter = 0

    def on_tick(self) -> None:
        self.counter += 1
        msg = String()
        msg.data = f"{self.counter} @ {time.time():.3f}"
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    rclpy.spin(Heartbeat())


if __name__ == "__main__":
    main()
```

### 3) Reset service

Create `~/q2_ws/src/q2_ros2_basics/q2_ros2_basics/reset_service.py`:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ResetService(Node):
    def __init__(self) -> None:
        super().__init__("reset_service")
        self.counter = 0
        self.srv = self.create_service(Trigger, "/q2/reset_counter", self.on_reset)

    def on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.counter = 0
        response.success = True
        response.message = "Counter reset to 0"
        return response


def main() -> None:
    rclpy.init()
    rclpy.spin(ResetService())


if __name__ == "__main__":
    main()
```

### 4) Register entry points + build

Edit `~/q2_ws/src/q2_ros2_basics/setup.py`:

```python
entry_points={
    "console_scripts": [
        "heartbeat = q2_ros2_basics.heartbeat:main",
        "reset_service = q2_ros2_basics.reset_service:main",
    ],
},
```

Build:

```bash
cd ~/q2_ws
colcon build --symlink-install
source ~/q2_ws/install/setup.bash
```

Run:

```bash
# Terminal A
ros2 run q2_ros2_basics heartbeat --ros-args -p publish_rate_hz:=5.0
```

```bash
# Terminal B
ros2 run q2_ros2_basics reset_service
```

Inspect + call:

```bash
ros2 topic echo /q2/heartbeat --once
ros2 service call /q2/reset_counter std_srvs/srv/Trigger "{}"
```

## Troubleshooting

- `ModuleNotFoundError` when running a node
  - You forgot to rebuild or source: `colcon build --symlink-install && source install/setup.bash`.
- Service call hangs
  - Node isn’t running, or you called the wrong service type. Check: `ros2 service list -t`.
- Parameter doesn’t change behavior
  - You used `declare_parameter` but cached the old value; implement parameter callbacks if you
    want dynamic reconfigure.

## Quick Quiz

1. When should you choose an action over a service?
2. What are parameters used for, and what are they not good for?
3. Which CLI commands do you run first when “it doesn’t work”?
