# Controller

Local planning and control is essential for any mobile robot in real-world applications to deal with unexpected situations while tracking the global trajectory.

Controller is used for path tracking and control around dynamic obstacles during navigation.

## Available Run Types
Set from ControllerConfig class or directly from Controller 'run_type' property.

```{list-table}
:widths: 10 80
* - **Timed**
  - Compute a new control command periodically if all inputs are available

* - **ActionServer**
  - Offers a [ControlPath](https://github.com/automatika-robotics/kompass/blob/main/kompass_interfaces/action/ControlPath.action) ROS action and continuously computes a new control once an action request is received until goal point is reached
```

## Inputs:
```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - plan
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - `Topic(name="/plan", msg_type="Path")`

* - location
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`

* - sensor_data
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1
  - `Topic(name="/scan", msg_type="LaserScan")`

* - local_map
  - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
  - 1
  - `Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid")`

* - vision_tracking
  - [`automatika_embodied_agents.msg.Trackings`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg), [`automatika_embodied_agents.msg.Detections2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
  - 1
  - None, Should be provided to use the vision target tracking
```

```{tip}
Provide a 'vision_tracking' input topic to the controller to activate the creation of the a vision-based target following action server. See [this example](../tutorials/vision_tracking.md) for more details.
```

## Outputs:

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
  - ```Topic(name="/control", msg_type="Twist")```
* - multi_command
  - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/msg)
  - 1
  - ```Topic(name="/control_list", msg_type="TwistArray")```
* - interpolation
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - ```Topic(name="/interpolated_path", msg_type="Path")```
* - local_plan
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - ```Topic(name="/local_path", msg_type="Path")```
* - tracked_point
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)[`automatika_embodied_agents.msg.Detection2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
  - 1
  - ```Topic(name="/tracked_point", msg_type="PoseStamped")```
```

## Available Algorithms:

- [Stanley](../advanced/algorithms/stanley.md) (pure follower)
- [DVZ](../advanced/algorithms/dvz.md) (Deformable Virtual Zone)
- [DWA](../advanced/algorithms/dwa.md) (Dynamic Window Approach)
- [VisionFollower](../advanced/algorithms/vision.md) (Vision target following controller)

## Configuration Parameters:

See [ControllerConfig](../apidocs/kompass/kompass.components.controller.md/#kompass.components.controller.ControllerConfig)


## Usage Example:
```python
    from kompass.components import ControllerConfig, Controller
    from kompass.topic import Topic

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
