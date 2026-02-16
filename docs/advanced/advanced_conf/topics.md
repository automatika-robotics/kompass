# Inputs and Outputs

**Strict typing and standardized data streams**

In Kompass, Components are designed with strict interfaces to "lock" their functionality. Each input and output is associated with a **unique keyword name** and restricted to a set of specific ROS2 message types.

A single keyword (e.g., `sensor_data`) can accept:
1.  **A Single Topic:** One laser scan.
2.  **Multiple Topics:** A list of topics (e.g., LiDAR + Radar) which are then fused internally by the component.

## Configuration

Configuring inputs and outputs is done via the `.inputs()` and `.outputs()` methods on any Component instance.

```python
from kompass.components import DriveManager
from kompass.ros import Topic

driver = DriveManager(component_name="driver")

# 1. Configure Inputs
# The 'sensor_data' key accepts a list of topics (fusion)
driver.inputs(
    sensor_data=[
        Topic(name='/scan', msg_type='LaserScan'),
        Topic(name='/radar_data', msg_type='Float64')
    ]
)

# 2. Configure Outputs
# Remap the 'emergency_stop' signal to a custom topic name
driver.outputs(
    emergency_stop=Topic(name='/system/alarm', msg_type='Bool')
)

```

:::{seealso}
See the `Topic` configuration class details in the [API Documentation](https://www.google.com/search?q=../../apidocs/kompass/kompass.components.ros.md).
:::

## Standard Stream Keys

Kompass uses a standardized set of keys across the entire stack.

### {material-regular}`explore` Navigation & Planning

| Enum Key | Keyword String | Description |
| --- | --- | --- |
| **GOAL_POINT** | `"goal_point"` | Target destination for the global planner. |
| **GLOBAL_PLAN** | `"plan"` | The computed path from start to goal. |
| **LOCAL_PLAN** | `"local_plan"` | Short-term path plan output by the Controller. |
| **INTERPOLATED_PATH** | `"interpolation"` | Smoothed or interpolated global path. |
| **REACHED_END** | `"reached_end"` | Flag indicating the goal has been reached. |

### {material-regular}`map` Mapping & Perception

| Enum Key | Keyword String | Description |
| --- | --- | --- |
| **GLOBAL_MAP** | `"map"` | The static global reference map. |
| **LOCAL_MAP** | `"local_map"` | Dynamic occupancy grid of immediate surroundings. |
| **SPATIAL_SENSOR** | `"sensor_data"` | Raw spatial data (LIDAR, Radar, Depth). |
| **VISION_TRACKINGS** | `"vision_tracking"` | Tracking data from vision systems (bounding boxes/masks). |
| **DEPTH_CAM_INFO** | `"depth_camera_info"` | Camera intrinsics parameters. |
| **TRACKED_POINT** | `"tracked_point"` | Specific point or object currently being tracked. |

### {material-regular}`gamepad` Control & Actuation

| Enum Key | Keyword String | Description |
| --- | --- | --- |
| **INTERMEDIATE_CMD** | `"command"` | Velocity command produced by the Controller. |
| **INTERMEDIATE_CMD_LIST** | `"multi_command"` | List of candidate velocity commands. |
| **FINAL_COMMAND** | `"robot_command"` | Final, safety-checked command sent to the Driver. |
| **EMERGENCY** | `"emergency_stop"` | Signal for immediate robot halt. |

### {material-regular}`settings_system_daydream` System & State

| Enum Key | Keyword String | Description |
| --- | --- | --- |
| **ROBOT_LOCATION** | `"location"` | Current robot position and orientation (Odometry). |
| **RUN_TESTS** | `"run_tests"` | Trigger flag to initiate calibration procedures. |

:::{tip}
Each component's specific requirements (required vs. optional streams) can be found in its respective documentation page (e.g., [Planner Inputs](../../navigation/path_planning.md/#inputs), [DriveManager Outputs](../../navigation/driver.md/#outputs)).
:::
