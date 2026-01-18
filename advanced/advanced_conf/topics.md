# Inputs and Outputs

Components in Kompass are defined to accept only restricted types of inputs/outputs to help lock the functionality of a specific Component implementation. Each input/output is associated with a unique keyword name and is set to accept one or many of ROS2 message types. Additionally, the input/output keyword in the Component can define a category of Topics rather than a single one. To see an example of this check the [DriveManager](../../navigation/driver.md) Component. In this component the input [sensor_data](../../navigation/driver.md/#inputs) defines any proximity sensor input (LiDAR, Radar, etc.) and can optionally take up to 10 Topics of such types to fuse it internally during execution.

Configuring an input/output of a Component is very straightforward and can be done in one line in your Python script. Below is an example for configuring the previously mentioned DriveManager:

```python
    from kompass.components import DriveManager
    from kompass.ros import Topic

    driver = DriveManager(component_name="driver")

    # Configure an input
    driver.inputs(sensor_data=[Topic(name='/scan', msg_type='LaserScan'),
                               Topics(name='radar_data', msg_type='Float64')])

    # Configure an output
    driver.outputs(emergency_stop=Topic(name='alarm', msg_type='Bool'))
```

```{seealso}
See the input/output configuration class `Topic` in detail [here](../../apidocs/kompass/kompass.components.ros.md)
```

All Inputs/Outputs in Kompass Components are defined with fixed **key names** across the stack, each containing:
 - Set of allowed types for the stream (equivalent to ROS2 messages)
 - The number of required streams for the key name
 - The maximum number of additional streams that can be assigned.


 Below is a list of all the streams (inputs and outputs) key names available in Kompass stack:

```{list-table}
:widths: 20 20 60
:header-rows: 1
* - Enum TopicsKeys
  - Name Value
  - Description

* - GOAL_POINT
  - `"goal_point"`
  - Target destination point on the map for the robot point navigation

* - GLOBAL_PLAN
  - `"plan"`
  - Global navigation plan (path) from start to goal

* - GLOBAL_MAP
  - `"map"`
  - Global (reference) map used for navigation

* - ROBOT_LOCATION
  - `"location"`
  - Current position and orientation of the robot

* - SPATIAL_SENSOR
  - `"sensor_data"`
  - Raw data from robot's spatial sensors (e.g., LIDAR, depth sensors)

* - VISION_TRACKINGS
  - `"vision_tracking"`
  - Visual tracking data from robot's cameras or vision systems

* - DEPTH_CAM_INFO
  - `"depth_camera_info"`
  - Depth camera information which includes camera intrinsics parameters

* - LOCAL_PLAN
  - `"local_plan"`
  - Short-term path plan considering immediate surroundings

* - INTERMEDIATE_CMD
  - `"command"`
  - Robot velocity command produced by the control system

* - INTERMEDIATE_CMD_LIST
  - `"multi_command"`
  - List of intermediate velocity commands

* - LOCAL_MAP
  - `"local_map"`
  - Map of the immediate surroundings for local navigation (control)

* - LOCAL_MAP_OCC
  - `"local_map"`
  - Occupancy grid representation of the local environment

* - INTERPOLATED_PATH
  - `"interpolation"`
  - Interpolated global path

* - TRACKED_POINT
  - `"tracked_point"`
  - Specific point being tracked by the robot's systems on the reference path of reference vision target

* - FINAL_COMMAND
  - `"robot_command"`
  - Final control command sent to robot's driver

* - EMERGENCY
  - `"emergency_stop"`
  - Emergency stop signal for immediate robot halt

* - REACHED_END
  - `"reached_end"`
  - Flag indicating whether the goal point has been reached

* - RUN_TESTS
  - `"run_tests"`
  - Flag to initiate system test procedures
```

```{tip}
You can refer to each individual component in the stack to see its respective Inputs and Output keys, along with their allowed types and number of optional and required streams. (See [Planner Inputs](../../navigation/path_planning.md/#inputs) or example)
```
