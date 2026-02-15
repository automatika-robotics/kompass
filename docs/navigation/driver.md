# Drive Manager

**Safety enforcement and command smoothing.**

The [DriveManager](../apidocs/kompass/kompass.components.drive_manager.md) is the final gatekeeper before commands reach your robot's low-level interfaces. Its primary job is to ensure that every command falls within the robot's physical limits, satisfies smoothness constraints, and does not lead to a collision.

It acts as a safety shield, intercepting velocity commands from the Controller and applying **Emergency Stops** or **Slowdowns** based on immediate sensor data.


## Safety Layers

The Drive Manager implements a multi-stage safety pipeline.

```{list-table}
:widths: 30 70
* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`pan_tool;1.5em;sd-text-primary` Emergency Stop</span>
  - **Critical Zone**. Checks proximity sensors directly. If an obstacle enters the configured **Safety Distance** (m) and **Angle** (rad), the robot stops immediately to prevent collision.

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`speed;1.5em;sd-text-primary` Dynamic Slowdown</span>
  - **Warning Zone**. If an obstacle enters the **Slowdown Zone**, the robot's velocity is proportionally reduced, allowing for smoother reactions than a hard stop.

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`equalizer;1.5em;sd-text-primary` Control Limiting</span>
  - **Kinematic Constraints**. Clamps incoming velocity and acceleration commands to the robot's physical limits (max vel, max acc) to prevent hardware stress.

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`waves;1.5em;sd-text-primary` Smoothing</span>
  - **Jerk Control**. Applies smoothing filters (e.g., low-pass) to incoming commands to prevent jerky movements and wheel slip.
```

```{figure} ../_static/images/diagrams/drive_manager_dark.png
:class: dark-only
:alt: Emergency Zone & Slowdown Zone
:align: center
:width: 70%

```

```{figure} ../_static/images/diagrams/drive_manager_light.png
:class: light-only
:alt: Emergency Zone & Slowdown Zone
:align: center
:width: 70%

Emergency Zone & Slowdown Zone
```

:::{admonition} High-Performance Core
:class: tip
Critical and Slowdown Zone checking is implemented in C++ via [kompass-core](https://github.com/automatika-robotics/kompass-core). The implementation supports both **GPU** and **CPU** acceleration (defaults to GPU if available) for minimal latency.
:::


## Built-in Actions

The Drive Manager provides built-in behaviors for direct control and recovery. These can be triggered via [Events](../tutorials/events_actions.md).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`arrow_upward` move_forward</span> - Moves the robot forward for `max_distance` meters, if the path is clear.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`arrow_downward` move_backward</span> - Moves the robot backward for `max_distance` meters, if the path is clear.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`rotate_right` rotate_in_place</span> - Rotates the robot for `max_rotation` radians, checking the safety margin.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`lock_open` move_to_unblock</span> - **Recovery Behavior.** Automatically attempts to move forward, backward, or rotate to free the robot from a collision state or blockage.

:::{admonition} Sensor Requirement
:class: warning
All movement actions require active $360^o$ sensor data (`LaserScan` or `PointCloud2`) data to verify that the movement direction is collision-free.
:::



## Interface

### Inputs

The Drive Manager subscribes to commands and raw sensor data.

```{list-table}
:widths: 15 35 15 35
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">command</span>
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - 1
  - `/control` (`Twist`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">multi_command</span>
  - `TwistArray`
  - 1
  - `/control_list`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">sensor_data</span>
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), `std_msgs.msg.Float64`, `std_msgs.msg.Float32`, [`sensor_msgs.msg.PointCloud2`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1+ (Up to 10)
  - `/scan` (`LaserScan`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">location</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `/odom` (`Odometry`)

```

### Outputs

The processed commands sent to the hardware.

```{list-table}
:widths: 15 35 15 35
:header-rows: 1

* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">robot_command</span>
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - 1
  - `/cmd_vel` (`Twist`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">emergency_stop</span>
  - `Bool`
  - 1
  - `/emergency_stop`

```

## Usage Example

```python
from kompass.components import DriveManager, DriveManagerConfig
from kompass.ros import Topic

# 1. Configuration
# Define safety zones and loop behavior
my_config = DriveManagerConfig(
    closed_loop=True,              # Check robot state feedback
    critical_zone_distance=0.1,    # Stop if obstacle < 10cm
    slowdown_zone_distance=0.3,    # Slow down if obstacle < 30cm
    critical_zone_angle=90.0       # Check 90 degrees cone in front
)

# 2. Instantiate
driver = DriveManager(component_name="driver", config=my_config)

# 3. Remap Outputs
# Send safe commands to your specific robot topic
driver.outputs(robot_command=Topic(name='/my_robot_cmd', msg_type='TwistStamped'))

```

## See Next

Learn how to trigger the unblocking behaviors automatically using Events.

:::{button-link} ../tutorials/events_index.html
:color: primary
:ref-type: doc
:outline:
Event-Driven Recovery Tutorial →
:::
