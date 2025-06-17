# 📨 Supported ROS2 Messages

Kompass components create automatic subscribers and callbacks to all inputs and publishers to all outputs. Kompass comes with a set of supported message types; each supported type has an associated callback and publisher.

```{tip}
Access all callbacks in a component in `self.callbacks: Dict[str, GenericCallback]` and get the topic incoming message using `get_output` method in the callback class
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

* - **[String](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)
  - `str`
  - `str`

* - **[Bool](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html)
  - `bool`
  - `bool`

* - **[Float32](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)
  - `float`
  - `float`

* - **[Float32MultiArray](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32MultiArray.html)
  - `numpy.ndarray[float]`
  - `numpy.ndarray[float]`

* - **[Float64](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64.html)
  - `float`
  - `float`

* - **[Float64MultiArray](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64MultiArray.html)
  - `numpy.ndarray[float]`
  - `numpy.ndarray[float]`

* - **[Point](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html)
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[PointStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PointStamped.html)
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[Pose](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html)
  - `kompass_core.models.RobotState`
  - `numpy.ndarray`

* - **[PoseStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html)
  - [`kompass_core.models.RobotState`](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L228)
  - `numpy.ndarray`

* - **[Twist](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)
  - `geometry_msgs.msg.Twist`
  - `geometry_msgs.msg.Twist`

* - **[TwistStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - `geometry_msgs.msg.Twist`
  - `geometry_msgs.msg.Twist`

* - **[TwistArray](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [kompass_interfaces](https://github.com/automatika-robotics/kompass/blob/main/kompass_interfaces/msg/motion/TwistArray.msg)
  - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/blob/main/kompass_interfaces/msg/motion/TwistArray.msg)
  - `kompass_interfaces.msg.TwistArray`

* - **[Image](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)
  - `numpy.ndarray`
  - `numpy.ndarray`

* - **[CompressedImage](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CompressedImage.html)
  - `numpy.ndarray`
  - `sensor_msgs.msg.CompressedImage | numpy.ndarray`

* - **[Audio](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Audio.html)
  - `bytes`
  - `str | bytes`

* - **[LaserScan](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)
  - [`kompass_core.datatypes.LaserScanData`]((https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/datatypes/laserscan.py))
  - `kompass_core.datatypes.LaserScanData`

* - **[PointCloud2](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)
  - [`kompass_core.datatypes.PointCloudData`](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/datatypes/pointcloud.py)
  - `sensor_msgs.msg.PointCloud2`

* - **[Odometry](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)
  - `kompass_core.models.RobotState`
  - `kompass_core.models.RobotState`


* - **[Path](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)
  - `nav_msgs.msg.Path`
  - `nav_msgs.msg.Path`

* - **[OccupancyGrid](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)
  - `nav_msgs.msg.OccupancyGrid | np.ndarray | Dict`
  - `numpy.ndarray`

* - **[ComponentStatus](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [automatika_ros_sugar](https://github.com/automatika-robotics/sugarcoat/blob/main/msg/ComponentStatus.msg)
  - `automatika_ros_sugar.msg.ComponentStatus`
  - `automatika_ros_sugar.msg.ComponentStatus`

* - **Detections**
  - [automatika_agents_interfaces](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Detections2D.msg)
  - [`Optional[kompass_core.datatypes.Bbox2D]`](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_cpp/bindings/bindings_types.cpp)
  - [`automatika_agents_interfaces.msg.Detections2D`](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Detections2D.msg)

* - **Trackings**
  - [automatika_agents_interfaces](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Trackings.msg)
  - [`Optional[kompass_core.datatypes.Bbox2D]`](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_cpp/bindings/bindings_types.cpp)
  - [`automatika_agents_interfaces.msg.Trackings`](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Trackings.msg)

```
