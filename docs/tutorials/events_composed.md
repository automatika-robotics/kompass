# Complex Events: Logic Gates & Fusion

In the previous tutorials, we triggered actions based on single, isolated conditions (e.g., "If Mapper Fails" or "If Person Seen").

However, real-world autonomy is rarely that simple. A robot shouldn't stop *every* time it sees an obstacle, maybe it only stops if it's moving fast. It shouldn't return home *just* because the battery is low, maybe it waits until it finishes its current task.

In this tutorial, we will use **Logic Gates** and **Multi-Topic Fusion** to create smarter, more robust reflexes.


## Complex Condition Logic

Kompass allows you to compose complex triggers using standard Python bitwise operators. This effectively turns your Event definitions into a high-level logic circuit.

| Logic | Operator | Description | Use Case |
| :--- | :--- | :--- | :--- |
| **AND** | `&` | All conditions must be True. | "Safety Checks" (Speed > 0 **AND** Obstacle Close) |
| **OR** | `\|` | At least one condition is True. | "Redundancy" (Lidar Blocked **OR** Bumper Hit) |
| **NOT** | `~` | Inverts the condition. | "Exclusion" (Target Seen **AND NOT** Low Battery) |

---

## Scenario: The "Smart" Emergency Stop (AND Logic)

**The Problem:** A naive emergency stop triggers whenever an object is within 0.5m. But if the robot is docking or maneuvering in a tight elevator, this stops it unnecessarily.

**The Solution:** We create a "Smart Stop" that triggers ONLY if: An obstacle is close (< 0.5m) **AND** The robot is moving fast (> 0.1 m/s)

1. Define Event Sources (Topics)

We need data from the **Radar** (perception) and **Odometry** (state).

```python
from kompass.ros import Topic

# 1. Lidar (0.2s timeout for safety)
radar = Topic(name="/radar_front", msg_type="Float32", data_timeout=0.2)

# 2. Odometry (0.5s timeout)
odom = Topic(name="/odom", msg_type="Odometry", data_timeout=0.5)

```

2. Define The Composed Event

We combine the conditions using the `&` operator.

```python
from kompass.ros import Event

# Condition A: Obstacle Close
is_obstacle_close = radar.msg.data < 0.3

# Condition B: Moving Fast (Linear X velocity)
is_robot_moving_fast = odom.msg.twist.twist.linear.x > 0.1

# Composed Event: BOTH must be True
event_smart_stop = Event(
    event_condition=(is_obstacle_close & is_robot_moving_fast),
    on_change=True
)

```
