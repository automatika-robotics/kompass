# Vision Follower

An vision based target following algorithm is available to be used in the [Controller](../../navigation/control.md) component. The method requires [Detections](../types.md/#-supported-ros2-messages) or [Trackings](../types.md/#-supported-ros2-messages) information from an external source, this can be provided using an object detection ML model. The [Controller](../../navigation/control.md) creates a VisionFollowing ROS `ActionServer` if a detection/tracking input is provided to the controller.

The Vision Follower can operate in two modes based on the available information in the incoming `Detections` (or `Trackings`) message:

  - Using 2D detections (or trackings) and Depth Image: This option is prioritized if a depth image is provided. In this case the controller will use the depth information to convert the 2D detected boxes into 3D boxes, track the target box and compute the tracking control.
  - Using 2D detections (or trackings) and RGB Image: If no depth information is provided, the VisionFollower will use the RGB image directly to compute the tracking control based on the relative shift and size of the 2D detected box to keep it centered in the 2D image.

```{note}
Using Depth information will provide more robust tracking.
```
```{note}
The `Controller` does not subscribe directly to the `Depth` or `RGB` Images. The component expects the relevant image to be sent within the same `Detections` (or `Trackings`) message to insure data synchronization. For more details, check the structure of the (Detections Message)[https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Detection2D.msg] and (Trackings Message)[https://github.com/automatika-robotics/ros-agents/blob/main/agents/msg/Tracking.msg].
```

```{seealso}
See a full example of using the RGB mode vision follower from the Controller component along with a `Vision` component from (**EmbodiedAgents**)[https://automatika-robotics.github.io/ros-agents/] [here](../../tutorials/vision_tracking.md).
```

## Supported Inputs

- [Detections](../types.md/#-supported-ros2-messages)
- [Trackings](../types.md/#-supported-ros2-messages)

## Parameters and Default Values

```{list-table}
:widths: 10 10 10 70
:header-rows: 1
* - Name
  - Type
  - Default
  - Description
* - control_time_step
  - `float`
  - `0.1`
  - Time interval between control actions. Must be between `1e-4` and `1e6`.
* - control_horizon
  - `int`
  - `2`
  - Number of future steps to consider in control. Must be between `1` and `1000`.
* - tolerance
  - `float`
  - `0.1`
  - Acceptable error margin for target tracking. Must be between `1e-6` and `1e3`.
* - target_distance
  - `Optional[float]`
  - `None`
  - Desired distance to maintain from target. No validation constraints.
* - target_search_timeout
  - `int`
  - `30`
  - Maximum number of steps allowed for target search before timeout. Must be between `0` and `1000`.
* - target_search_pause
  - `int`
  - `2`
  - Number of steps to pause between search actions. Must be between `0` and `1000`.
* - target_search_radius
  - `float`
  - `0.5`
  - Radius of the search area for target detection. Must be between `1e-4` and `1e4`.
* - rotation_multiple
  - `float`
  - `1.0`
  - Scaling factor for rotation commands. Must be between `1e-9` and `1.0`.
* - speed_depth_multiple
  - `float`
  - `0.7`
  - Scaling factor for speed based on depth information. Must be between `1e-9` and `1.0`.
* - min_vel
  - `float`
  - `0.01`
  - Minimum allowable velocity. Must be between `1e-9` and `1e9`.
* - enable_search
  - `bool`
  - `True`
  - Flag to enable or disable search behavior when the target is lost.
```
