# Vision Follower

A vision-based target following algorithm is available to be used from the [Controller](../../navigation/control.md) component. The method requires [Detections](../types.md/#supported-ros2-messages) or [Trackings](../types.md/#supported-ros2-messages) information from an external source, this can be provided using an object detection ML model.

```{note}
If the method is used with `Trackings` information as an input, then the controller would follow the given object with the same ID. In case of using `Detections`, the controller would follow the first detected object with the same given `label` in the scene.
```

```{seealso}
See a full example of using the vision follower from the controller component along with a vision component [here](../../tutorials/vision_tracking.md).
```

## Supported Motion Models

- ACKERMANN
- DIFFERENTIAL_DRIVE
- OMNI

## Supported Inputs

- [Detections](../types.md/#supported-ros2-messages)
- [Trackings](../types.md/#supported-ros2-messages)

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
