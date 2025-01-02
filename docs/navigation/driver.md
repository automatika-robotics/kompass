# Drive Manager

[DriveManager](../apidocs/kompass/kompass.components.drive_manager.md) is responsible for direct control communication with the robot. It is used to perform last checks on any control command before passing it to the robot to ensure that the commands falls within the robot control limits, satisfies smoothness conditions and does not lead to a collision with a nearby obstacle.

The DriveManager component can perform one or multiple of the following functionalities based on the desired config:

```{list-table}
:widths: 20 70
* - **Control limiting**
  - Checks that the incoming control is within the robot control limits. If a control command is outside the robot control limits, the DriveManager limits the value to the maximum/minimum allowed value before passing the command to the robot

* - **Emergency stopping**
  - Checks the direct sensor information from the configured proximity sensors and performs an emergency stop if an obstacle is closer than the allowed safety limit

* - **Control Smoothing (Filtering)**
  - Performs smoothing filtering on the incoming control commands before sending the commands to the robot

* - **Robot Unblocking**
  - Moves the robot forward, backwards or rotates in place if the space is free to move the robot away from a blocking point. This action can be configured to be triggered with an external event
```

DriveManager also includes built-in movement actions used for directly control the robot or unblocking the robot in certain conditions:

```{list-table}
:widths: 20 70
:header-rows: 1

* - Action
  - Function

* - **[move_forward](../apidocs/kompass/kompass.components.drive_manager.md/#kompass.components.drive_manager.DriveManager)**
  - Moves the robot forward for `max_distance` meters, if the forward direction is clear of obstacles.

* - **[move_backward](../apidocs/kompass/kompass.components.drive_manager.md/#kompass.components.drive_manager.DriveManager)**
  - Moves the robot backwards for `max_distance` meters, if the backward direction is clear of obstacles.

* - **[rotate_in_place](../apidocs/kompass/kompass.components.drive_manager.md/#kompass.components.drive_manager.DriveManager)**
  - Rotates the robot in place for `max_rotation` radians, if the given safety margin around the robot is clear of obstacles.


* - **[move_to_unblock](../apidocs/kompass/kompass.components.drive_manager.md/#kompass.components.drive_manager.DriveManager)**
  - Moves the robot forward, backwards or rotates in place if the space is free to move the robot away from a blocking point.
```

```{note}
All the previous movement actions require `LaserScan` information to determine if the movement direction is collision-free
```

```{seealso}
Check an example on configuring the robot unblocking functionality with an external event in [this tutorial](/tutorials/events_actions.md)
```


## Available Run Types

```{list-table}
:widths: 10 80
* - **Timed**
  - Sends incoming command periodically to the robot
```

## Inputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1

* - Key Name
  - Allowed Types
  - Number
  - Default

* - command
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
  - 1
  - `Topic(name="/control", msg_type="Twist")`

* - multi_command
  - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/msg)
  - 1
  - `Topic(name="/control_list", msg_type="TwistArray")`

* - sensor_data
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), `std_msgs.msg.Float64`, `std_msgs.msg.Float32`
  - 1 + (10 optional)
  - `Topic(name="/scan", msg_type="LaserScan")`

* - location
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`
```


## Outputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - robot_command
  - `geometry_msgs.msg.Twist`
  - 1
  - `Topic(name="/cmd_vel", msg_type="Twist")`
* - emergency_stop
  - `std_msgs.msg.Bool`
  - 1
  - `Topic(name="/emergency_stop", msg_type="Bool")`
```

## Configuration Parameters:

See [DriveManagerConfig](../apidocs/kompass/kompass.components.drive_manager.md/#kompass.components.drive_manager.DriveManagerConfig)

## Usage Example:
```python
    from kompass.components import DriveManager, DriveManagerConfig
    from kompass.topic import Topic

    # Setup custom configuration
    # cmd_rate: rate for sending commands to the robot (Hz)
    # critical_zone_distance: for emergency stp (m)
    my_config = DriveManagerConfig(cmd_rate=10.0, critical_zone_distance=0.05)

    driver = DriveManager(component_name="driver", config=my_config)

    # Change the robot command output
    driver.outputs(robot_command=Topic(name='/my_robot_cmd', msg_type='Twist'))
```
