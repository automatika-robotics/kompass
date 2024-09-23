# Supported ROS2 Messages

Kompass components create automatic subscribers and callbacks to all inputs and publishers to all outputs. Kompass comes with a set of supported message types; each supported type has an associated callback and publisher.

```{tip}
Access all callbacks in a component in `self.callbacks: Dict[str, GenericCallback]` and get the topic incoming message using `get_ouput` method in the callback class
```

```{tip}
Access all publishers in a `BaseComponent` in `self.publishers_dict: Dict[str, Publisher]` and publish a new message to the topic using `publish` method in the publisher class
```

Below is a list of supported messages and the types accepted by their publishers `publish` method and returned by their callback `get_output` method:

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Message
  - ROS2 package
  - Callback return type
  - Publisher converts from

* - **[String](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - std_msgs
  - `str`
  - `str`

* - **[Bool](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - std_msgs
  - `bool`
  - `bool`

* - **[Float32](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - std_msgs
  - `float`
  - `float`

* - **[Float64](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - std_msgs
  - `float`
  - `float`

* - **[Point](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - geometry_msgs
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[PointStamped](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - geometry_msgs
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[Pose](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - geometry_msgs
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[PoseStamped](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - geometry_msgs
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[Twist](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - geometry_msgs
  - `geometry_msgs.msg.Twist`
  - `geometry_msgs.msg.Twist`

* - **[TwistArray](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - kompass_interfaces
  - `kompass_interfaces.msg.TwistArray`
  - `kompass_interfaces.msg.TwistArray`

* - **[Image](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - sensor_msgs
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[Audio](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - sensor_msgs
  - `bytes`
  - `str | bytes`

* - **[Odometry](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - nav_msgs
  - `kompass_core.models.RobotState`
  - `kompass_core.models.RobotState`

* - **[LaserScan](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - sensor_msgs
  - `kompass_core.datatypes.LaserScanData`
  - `kompass_core.datatypes.LaserScanData`

* - **[PointCloud2](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - sensor_msgs
  - `kompass_core.datatypes.PointCloudData`
  - `sensor_msgs.msg.PointCloud2`

* - **[Path](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - nav_msgs
  - `nav_msgs.msg.Path`
  - `nav_msgs.msg.Path`

* - **[OccupancyGrid](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - nav_msgs
  - `nav_msgs.msg.OccupancyGrid | np.ndarray | Dict`
  - `numpy.ndarray`

* - **[ComponentStatus](../apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - ros_sugar_interfaces
  - `ros_sugar_interfaces.msg.ComponentStatus`
  - `ros_sugar_interfaces.msg.ComponentStatus`

```
