# Controller

Local planning and control is essential for any mobile robot in real-world applications to deal with unexpected situations while tracking the global trajectory.

Controller component used for path tracking and control around dynamic obstacles during navigation.

## Available Run Types
Set from ControllerConfig class or directly from Controller 'run_type' property.

```{list-table}
:widths: 10 80
* - **Timed**
  - Compute a new control command periodically if all inputs are available

* - **ActionServer**
  - Offers a ControlPath ROS action and continuously computes a new control once an action request is received until goal point is reached
```

## Controller Inputs

```{list-table}
:widths: 10 30 15 20 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Number of Topics
  - Default Value

* - **plan**
  - Path to reach the goal point from start location
  - `Path`
  - 1
  - `Topic(name="/plan", msg_type="Path")`

* - **location**
  - Robot current location
  - `Odometry, PoseWithCovariance, Pose`
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`

* - **sensor_data**
  - Direct sensor input
  - `LaserScan, PointCloud2`
  - 1
  - `Topic(name="/scan", msg_type="LaserScan")`

* - **local_map**
  - Local occupancy map
  - `OccupancyGrid`
  - 1
  - `Topic(name="/scan", msg_type="LaserScan")`
```

## Controller Outputs

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Default Value

* - **command**
  - One control command
  - `Twist`
  - `Topic(name="/control", msg_type="Twist")`

* - **multi_command**
  - A set of future control commands
  - `TwistArray`
  - `Topic(name="/control_list", msg_type="TwistArray")`

* - **interpolation**
  - Interpolated path
  - `Path`
  - `Topic(name="/interpolated_path", msg_type="Path")`

* - **tracked_point**
  - Tracked (closest) path point
  - `PoseStamped`
  - `Topic(name="/tracked_point", msg_type="PoseStamped")`
```

## Available Algorithms:

- [Stanley](../advanced/algorithms/stanley.md) (pure follower)
- [DVZ](../advanced/algorithms/dvz.md) (Deformable Virtual Zone)
- [DWA](../advanced/algorithms/dwa.md) (Dynamic Window Approach)


## Usage Example:
```python
    # Setup custom configuration
    my_config = ControllerConfig(loop_rate=10.0)

    # Init a controller object
    my_controller = Controller(component_name="controller", config=my_config)

    # Change an input
    my_controller.inputs(plan=Topic(name='/global_path', msg_type='Path'))

    # Change run type (default "Timed")
    my_controller.run_type = "ActionServer"

    # Change plugin
    my_controller.plugin = 'DWA'
```
