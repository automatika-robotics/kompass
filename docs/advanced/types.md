# Supported ROS2 Messages

Kompass handles the ROS2 plumbing so you don't have to! Every component automatically initializes the necessary subscribers and publishers for its inputs and outputs, ensuring seamless data flow across your system.

Below is the comprehensive list of ROS2 message types natively supported by the Kompass

```{list-table}
:widths: 40 40
:header-rows: 1
* - Message
  - ROS2 package

* - **[String](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html)


* - **[Bool](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Bool.html)


* - **[Float32](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html)


* - **[Float32MultiArray](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32MultiArray.html)

* - **[Float64](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64.html)


* - **[Float64MultiArray](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.md/#classes)**
  - [std_msgs](https://docs.ros2.org/foxy/api/std_msgs/msg/Float64MultiArray.html)


* - **[Point](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html)


* - **[PointStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PointStamped.html)


* - **[Pose](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html)


* - **[PoseStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html)


* - **[Twist](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)


* - **[TwistStamped](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [geometry_msgs](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)


* - **[TwistArray](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [kompass_interfaces](https://github.com/automatika-robotics/kompass/blob/main/kompass_interfaces/msg/motion/TwistArray.msg)


* - **[Image](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Image.html)

* - **[CompressedImage](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/CompressedImage.html)


* - **[Audio](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Audio.html)


* - **[LaserScan](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html)


* - **[PointCloud2](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [sensor_msgs](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html)


* - **[Odometry](../apidocs/kompass/kompass.data_types.md/#classes)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html)



* - **[Path](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/Path.html)


* - **[OccupancyGrid](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [nav_msgs](https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html)


* - **[ComponentStatus](https://automatika-robotics.github.io/sugarcoat/apidocs/ros_sugar/ros_sugar.io.supported_types.html)**
  - [automatika_ros_sugar](https://github.com/automatika-robotics/sugarcoat/blob/main/msg/ComponentStatus.msg)


* - **Detections**
  - [automatika_agents_interfaces](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Detections2D.msg)


* - **Trackings**
  - [automatika_agents_interfaces](https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Trackings.msg)

```
