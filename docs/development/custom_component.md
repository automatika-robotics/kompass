---
title: Creating a Custom Navigation Component
---

# Creating a Custom Navigation Component

Every execution unit in Kompass is a `Component`, which maps to a ROS 2 Lifecycle Node. This guide explains how to create a new component by subclassing `kompass.components.component.Component`.

## Component Base Class

`Component` (defined in `kompass/kompass/components/component.py`) extends `BaseComponent` from [ros-sugar](https://github.com/automatika-robotics/ros-sugar). It adds Kompass-specific features:

- **Keyed I/O** via `TopicsKeys` enum -- each input/output has a semantic key, not just a topic name.
- **Allowed topics** enforcement via `AllowedTopics` -- restricts which message types a component accepts.
- **Default topic configurations** -- sensible defaults for rapid prototyping.
- **TF listener helpers** -- `odom_tf_listener`, `scan_tf_listener`, `depth_tf_listener`, `pc_tf_listener`.

### Constructor Signature

```python
def __init__(
    self,
    component_name: str,
    config: Optional[ComponentConfig] = None,
    config_file: Optional[str] = None,
    inputs: Optional[Dict[TopicsKeys, Union[Topic, List[Topic], None]]] = None,
    outputs: Optional[Dict[TopicsKeys, Union[Topic, List[Topic], None]]] = None,
    fallbacks: Optional[ComponentFallbacks] = None,
    allowed_inputs: Optional[Dict[str, AllowedTopics]] = None,
    allowed_outputs: Optional[Dict[str, AllowedTopics]] = None,
    allowed_run_types: Optional[List[ComponentRunType]] = None,
    callback_group=None,
    **kwargs,
):
```

## TopicsKeys Enum

`TopicsKeys` (in `kompass.components.defaults`) provides standardized key names for all topics in the Kompass navigation stack:

| Key | Value | Purpose |
|---|---|---|
| `GOAL_POINT` | `"goal_point"` | Navigation goal |
| `GLOBAL_PLAN` | `"plan"` | Global path from planner |
| `GLOBAL_MAP` | `"map"` | Global occupancy grid |
| `ROBOT_LOCATION` | `"location"` | Robot odometry/pose |
| `SPATIAL_SENSOR` | `"sensor_data"` | LiDAR, depth, point cloud |
| `VISION_DETECTIONS` | `"vision_detections"` | Object detection results |
| `DEPTH_CAM_INFO` | `"depth_camera_info"` | Depth camera intrinsics |
| `LOCAL_PLAN` | `"local_plan"` | Local path segment |
| `PATH_SAMPLES` | `"path_samples"` | Sampled path candidates |
| `INTERMEDIATE_CMD` | `"command"` | Single velocity command |
| `INTERMEDIATE_CMD_LIST` | `"multi_command"` | Velocity command array |
| `LOCAL_MAP` | `"local_map"` | Local occupancy grid |
| `INTERPOLATED_PATH` | `"interpolation"` | Interpolated path |
| `TRACKED_POINT` | `"tracked_point"` | Currently tracked point on path |
| `FINAL_COMMAND` | `"robot_command"` | Command sent to robot driver |
| `EMERGENCY` | `"emergency_stop"` | Emergency stop signal |
| `REACHED_END` | `"reached_end"` | Goal reached flag |

Using `TopicsKeys` ensures consistent naming across components and enables automatic wiring in recipes.

## AllowedTopics

Each component defines which message types are valid for each key using `AllowedTopics` (from `ros_sugar.io`):

```python
from ros_sugar.io import AllowedTopics

my_allowed_inputs = {
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(
        types=["Odometry", "PoseStamped", "Pose"]
    ),
    TopicsKeys.GOAL_POINT: AllowedTopics(
        types=["PointStamped", "PoseStamped"]
    ),
}
```

`AllowedTopics` parameters:

| Parameter | Default | Description |
|---|---|---|
| `types` | (required) | List of allowed message type names |
| `number_required` | `1` | Minimum number of topics required for this key |
| `number_optional` | `0` | Additional optional topics allowed (`-1` for unlimited) |

For example, a component that requires 1 sensor input but can optionally accept up to 3:

```python
TopicsKeys.SPATIAL_SENSOR: AllowedTopics(
    types=["LaserScan", "PointCloud2"],
    number_required=1,
    number_optional=2,
)
```

## Default Topic Configurations

Existing components define their defaults in `kompass.components.defaults`. For example, the Planner:

```python
planner_default_inputs = {
    TopicsKeys.GLOBAL_MAP: Topic(name="/map", msg_type="OccupancyGrid", ...),
    TopicsKeys.GOAL_POINT: Topic(name="/goal", msg_type="PointStamped"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}
```

When users provide custom topics, they are merged with your defaults using `update_topics()`:

```python
from kompass.components.ros import update_topics

in_topics = (
    update_topics(my_default_inputs, **inputs)
    if inputs
    else my_default_inputs
)
```

This lets users override individual topics without having to redefine the entire dictionary.

## Lifecycle and Key Methods

Components follow the ROS 2 Lifecycle pattern. The methods you override to add your logic:

### `init_variables()`

Called during node initialization (before `configure`). Use this to set up algorithm instances, internal state, and any variables that depend on `self.config`:

```python
def init_variables(self):
    self.goal = None
    self.robot_state = None
    self._my_algorithm = MyAlgorithm(
        robot_type=self.config.robot.model_type,
    )
```

### `custom_on_configure()`

Called when the lifecycle node transitions to **Inactive**. Subscriptions and publishers are created by the base class. Always call `super()` first:

```python
def custom_on_configure(self):
    super().custom_on_configure()
    # Additional configuration here
```

### `custom_on_activate()`

Called when the lifecycle node transitions to **Active**. The main execution loop starts after this.

### `_execution_step()`

The main loop body, called at `config.loop_rate` Hz (for `TIMED` run type). This is where your component reads inputs, runs its algorithm, and publishes outputs:

```python
def _execution_step(self):
    self._update_state()  # Read latest values from all callbacks
    if self.robot_state and self.goal:
        result = self._my_algorithm.compute(self.robot_state, self.goal)
        self.get_publisher(TopicsKeys.GOAL_POINT).publish(result)
```

### Run Types

The `ComponentRunType` enum controls how the component executes:

| Run Type | Description |
|---|---|
| `TIMED` | Calls `_execution_step()` at a fixed rate |
| `EVENT` | Executes in response to an event trigger |
| `SERVER` | Runs as a ROS 2 service server |
| `ACTION_SERVER` | Runs as a ROS 2 action server (supports feedback/cancel) |

Set `allowed_run_types` in your constructor to restrict which modes are valid for your component.

## Accessing Inputs and Outputs

### Reading Inputs: `get_callback()`

```python
callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)
```

Returns a `GenericCallback` object (or `None` if the input is not configured). Key API:

| Method / Property | Description |
|---|---|
| `.got_msg` | `True` if at least one message has been received |
| `.get_output()` | Returns the latest processed message data |
| `.get_output(clear_last=True)` | Returns data and clears the internal buffer |
| `.clear_last_msg()` | Clears the last received message |
| `.frame_id` | Frame ID from the message header (if available) |

For keys with multiple topics (e.g. multiple sensors), use the `idx` parameter:

```python
sensor_0 = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx=0)
sensor_1 = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx=1)
```

### Publishing Outputs: `get_publisher()`

```python
publisher = self.get_publisher(TopicsKeys.GOAL_POINT)
publisher.publish(my_msg)
```

Returns a `Publisher` object. Call `.publish()` with the message data. The publisher handles ROS message conversion automatically.

### Checking All Inputs

Use `callbacks_inputs_check()` to block until all required inputs are available:

```python
def _execution_step(self):
    if not self.callbacks_inputs_check():
        return
    # All inputs are ready
```

## Skeleton Example: WaypointManager Component

Below is a complete skeleton for a hypothetical `WaypointManager` component that accepts a set of waypoints and publishes the next goal point.

```python
from typing import Dict, List, Optional, Union
from attrs import define, field

from kompass.components.component import Component
from kompass.components.ros import Topic, update_topics
from kompass.components.defaults import TopicsKeys
from kompass.config import ComponentConfig, ComponentRunType
from ros_sugar.io import AllowedTopics


# --- Configuration ---
@define
class WaypointManagerConfig(ComponentConfig):
    """Configuration for the WaypointManager component."""
    waypoint_reach_threshold: float = field(default=0.5)
    loop_waypoints: bool = field(default=False)


# --- Allowed I/O ---
waypoint_manager_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(
        types=["Odometry", "PoseStamped", "Pose"]
    ),
    TopicsKeys.GLOBAL_PLAN: AllowedTopics(types=["Path"]),
}

waypoint_manager_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.GOAL_POINT: AllowedTopics(
        types=["PointStamped", "PoseStamped"]
    ),
}

# --- Default Topics ---
waypoint_manager_default_inputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
    TopicsKeys.GLOBAL_PLAN: Topic(name="/waypoints", msg_type="Path"),
}

waypoint_manager_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.GOAL_POINT: Topic(name="/goal", msg_type="PointStamped"),
}


# --- Component ---
class WaypointManager(Component):
    """Manages a list of waypoints and publishes the next goal."""

    def __init__(
        self,
        component_name: str = "waypoint_manager",
        config: Optional[WaypointManagerConfig] = None,
        config_file: Optional[str] = None,
        inputs: Optional[Dict] = None,
        outputs: Optional[Dict] = None,
        **kwargs,
    ):
        config = config or WaypointManagerConfig()

        # Merge user-provided topics with defaults
        in_topics = (
            update_topics(waypoint_manager_default_inputs, **inputs)
            if inputs
            else waypoint_manager_default_inputs
        )
        out_topics = (
            update_topics(waypoint_manager_default_outputs, **outputs)
            if outputs
            else waypoint_manager_default_outputs
        )

        super().__init__(
            component_name=component_name,
            config=config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=waypoint_manager_allowed_inputs,
            allowed_outputs=waypoint_manager_allowed_outputs,
            allowed_run_types=[ComponentRunType.TIMED],
            **kwargs,
        )

    def init_variables(self):
        """Called at node init -- set up internal state."""
        self._current_waypoint_idx = 0
        self._waypoints = []

    def custom_on_configure(self):
        """Called on lifecycle configure transition."""
        super().custom_on_configure()
        self._current_waypoint_idx = 0

    def _execution_step(self):
        """Main timed callback -- called at config.loop_rate Hz."""
        # Read current robot location
        location_cb = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        if location_cb is None or not location_cb.got_msg:
            return

        robot_pose = location_cb.get_output()

        # Get the waypoints path
        path_cb = self.get_callback(TopicsKeys.GLOBAL_PLAN)
        if path_cb is None or not path_cb.got_msg:
            return

        waypoints = path_cb.get_output()

        # Compute and publish the next waypoint
        # ... your logic here ...
        publisher = self.get_publisher(TopicsKeys.GOAL_POINT)
        publisher.publish(next_goal)
```

## Key Patterns to Follow

1. **Always define `allowed_inputs` and `allowed_outputs`** to enforce type safety.
2. **Provide default topics** so the component works out of the box.
3. **Merge with `update_topics()`** so users can override individual topics without redefining everything.
4. **Use `TopicsKeys`** for all I/O keys to maintain stack-wide consistency.
5. **Override `init_variables()`** to set up algorithm instances and state that depends on `self.config`.
6. **Override `custom_on_configure()`** and call `super()` first.
7. **Implement `_execution_step()`** as your main loop body.
8. **Use `get_callback(key)` and `get_publisher(key)`** to access I/O by semantic key rather than topic name.
9. **Set `allowed_run_types`** to restrict how the component can be executed.

## Next Steps

See the [Advanced Guide: Health Status, Fallbacks & Events](./advanced_component.md) for making your component resilient with automatic failure recovery.
