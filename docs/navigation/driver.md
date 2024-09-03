# Drive Manager

[DriveManager](../apidocs/Kompass/Kompass.components.drive_manager.md) is responsible for direct control communication with the robot. It is used to perform last checks on any control command before passing it to the robot to ensure that the commands falls within the robot control limits, satisfies smoothness conditions and does not lead to a collision with a nearby obstacle.

The Drive Manager can perform one or multiple of the following functionalities based on the desired config:

### Control limit check
Checks that the incoming control is within the robot control limits.

### Emergency stop
Subscribes to direct sensor input and handles emergency stopping

### Control Smoothing (Filtering)

### Robot Unblocking


## Available Run Types

```{list-table}
:widths: 10 80
* - **Timed**
  - Filters/sends command periodically
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

    # Setup custom configuration
    # cmd_rate: rate for sending commands to the robot (Hz)
    # critical_zone_distance: for emergency stp (m)
    my_config = DriveManagerConfig(cmd_rate=10.0, critical_zone_distance=0.05)

    driver = DriveManager(node_name="driver", config=my_config)

    # Change the robot command output
    driver.outputs(robot_command=Topic(name='/my_robot_cmd', msg_type='Twist'))
```
