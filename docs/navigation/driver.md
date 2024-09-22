# Drive Manager

[DriveManager](../apidocs/Kompass/Kompass.components.drive_manager.md) is responsible for direct control communication with the robot. It is used to perform last checks on any control command before passing it to the robot to ensure that the commands falls within the robot control limits, satisfies smoothness conditions and does not lead to a collision with a nearby obstacle.

The Drive Manager can perform one or multiple of the following functionalities based on the desired config:

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
:widths: 10 30 15 20 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Number of Topics
  - Default Value

* - **command**
  - One control command to pass to the robot
  - `Twist`
  - 1
  - `Topic(name="/control", msg_type="Twist")`

* - **multi_command**
  - A set of control commands to pass to the robot
  - `TwistArray`
  - 1
  - `Topic(name="/control_list", msg_type="TwistArray")`

* - **sensor_data**
  - Direct sensor input
  - `LaserScan, Float64, Float32`
  - 1 + (10 optional)
  - `Topic(name="/scan", msg_type="LaserScan")`

* - **location**
  - Robot current location
  - `Odometry, PoseWithCovariance, Pose`
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`
```

## Outputs

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Default Value

* - **robot_command**
  - Robot control command
  - `Twist`
  - `Topic(name="/cmd_vel", msg_type="Twist")`

* - **emergency_stop**
  - Emergency stop declaration flag
  - `Bool`
  - `Topic(name="/emergency_stop", msg_type="Bool")`
```


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
