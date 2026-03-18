---
title: Data Types, Callbacks & Publishers
---

# Data Types, Callbacks & Publishers

Kompass uses a type-driven I/O system built on [ros-sugar](https://github.com/automatika-robotics/ros-sugar). Each ROS message type is represented by a `SupportedType` class that bundles three things together:

1. **The ROS message type** (`_ros_type`) -- the underlying ROS 2 message class.
2. **A callback** (`callback`) -- how to process incoming messages of this type.
3. **A converter** (`convert()`) -- how to create outgoing messages for publishing.

When a `Topic` is configured with `msg_type="Odometry"`, the system automatically uses the `Odometry` SupportedType to set up the right callback for subscriptions and the right converter for publishing.

## How It Works

```
Subscribing (input):
  ROS message ──▶ GenericCallback.callback(msg) ──▶ stores raw msg
                                                       │
  component calls get_output() ──▶ _get_output() ──▶ post_processors ──▶ processed data

Publishing (output):
  component calls publisher.publish(data) ──▶ pre_processors ──▶ SupportedType.convert() ──▶ ROS message
```

### Concrete Example: Controller Data Flow

1. **Input**: `sensor_msgs/LaserScan` arrives on `/scan` topic.
2. **Callback**: `LaserScanCallback` converts it to `kompass_core.datatypes.LaserScanData`.
3. **Algorithm**: DWA controller reads `LaserScanData` along with `RobotState` (from odometry) and a reference `Path`.
4. **Output**: DWA computes velocity commands; `Twist.convert(vx, vy, omega)` creates `geometry_msgs/Twist`.
5. **Publish**: Result is published on `/control` topic.

## Built-in Types

Kompass extends the standard ros-sugar types with navigation-specific processing:

| Type class | ROS message | Callback | `_get_output()` returns | `convert()` accepts |
|---|---|---|---|---|
| `Odometry` | `nav_msgs/Odometry` | `OdomCallback` | `RobotState` | `RobotState` |
| `Pose` | `geometry_msgs/Pose` | `PoseCallback` | `RobotState` | `np.ndarray` |
| `PoseStamped` | `geometry_msgs/PoseStamped` | `PoseStampedCallback` | `RobotState` | `np.ndarray` |
| `Point` | `geometry_msgs/Point` | `PointCallback` | `RobotState` | `np.ndarray` |
| `PointStamped` | `geometry_msgs/PointStamped` | `PointStampedCallback` | `RobotState` | `np.ndarray` |
| `LaserScan` | `sensor_msgs/LaserScan` | `LaserScanCallback` | `LaserScanData` | `LaserScanData` |
| `PointCloud2` | `sensor_msgs/PointCloud2` | `PointCloudCallback` | `PointCloudData` | -- |
| `CameraInfo` | `sensor_msgs/CameraInfo` | `CameraInfoCallback` | `dict` | -- |
| `Twist` | `geometry_msgs/Twist` | `GenericCallback` | raw msg | `(vx, vy, omega)` |
| `TwistStamped` | `geometry_msgs/TwistStamped` | `TwistStampedCallback` | `Twist` (inner) | `(vx, vy, omega)` |
| `TwistArray` | `kompass_interfaces/TwistArray` | `GenericCallback` | raw msg | `TwistArray` |
| `Path` | `nav_msgs/Path` | `GenericCallback` | raw msg | `Path` (passthrough) |
| `OccupancyGrid` | `nav_msgs/OccupancyGrid` | `OccupancyGridCallback` | `np.ndarray` | `(np.ndarray, resolution)` |
| `Trackings` | `automatika_embodied_agents/Trackings` | `TrackingsCallback` | `Bbox2D` | -- |
| `Detections` | `automatika_embodied_agents/Detections2D` | `DetectionsCallback` | `List[Bbox2D]` | -- |

:::{note}
`Trackings` and `Detections` require the optional `automatika_embodied_agents` package.
:::

### kompass-core Data Types

The callback outputs above use data structures from `kompass_core` -- these are pure Python/C++ types with **no ROS dependency**, keeping `kompass-core` framework-agnostic:

- **`RobotState`** (`kompass_core.models`) -- position (x, y, yaw), velocity (vx, vy, omega), and speed.
- **`LaserScanData`** (`kompass_core.datatypes`) -- structured laser scan with angle/range arrays.
- **`PointCloudData`** (`kompass_core.datatypes`) -- 3D point cloud representation with field offsets.
- **`Bbox2D`** (`kompass_core.datatypes`) -- 2D bounding box for vision-based tracking.

## Creating a Custom Callback

A callback controls how incoming ROS messages are processed before your component sees them. To create one, subclass `GenericCallback` and implement `_get_output()`.

### Step 1: Implement the Callback

```python
# In kompass/kompass/callbacks.py (or your own module)
from typing import Optional
import numpy as np
from ros_sugar.io import GenericCallback


class MyImuCallback(GenericCallback):
    """Processes sensor_msgs/Imu into a simplified dict."""

    def __init__(self, input_topic, node_name: Optional[str] = None):
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[dict]:
        if not self.msg:
            return None

        return {
            "orientation_yaw": 2 * np.arctan2(
                self.msg.orientation.z, self.msg.orientation.w
            ),
            "angular_velocity": self.msg.angular_velocity.z,
            "linear_acceleration": np.array([
                self.msg.linear_acceleration.x,
                self.msg.linear_acceleration.y,
            ]),
        }
```

**Key points:**
- `self.msg` holds the raw ROS message (set automatically by the base class).
- Return `None` when no message is available yet.
- `_get_output()` can accept keyword arguments (e.g. `transformation`) that are passed from `get_output()`.

### Step 2: Support TF Transforms (Optional)

If your callback needs to apply TF transforms, add a `transformation` property:

```python
class MyTransformableCallback(GenericCallback):
    def __init__(self, input_topic, node_name=None, transformation=None):
        super().__init__(input_topic, node_name)
        self.__tf = transformation

    @property
    def transformation(self):
        return self.__tf

    @transformation.setter
    def transformation(self, transform):
        self.__tf = transform

    def _get_output(self, transformation=None, **_):
        if not self.msg:
            return None
        if transformation or self.transformation:
            return self._transform(self.msg, transformation or self.transformation)
        return self._process(self.msg)

    def _process(self, msg):
        # Convert raw message to your data type
        ...

    def _transform(self, msg, transform):
        # Apply TF transform, then convert
        ...
```

The component's TF listeners (e.g. `odom_tf_listener`) set the `transformation` property at runtime, and `get_output()` passes it through.

## Creating a Custom SupportedType

A `SupportedType` ties together the ROS message class, callback, and publisher converter.

### Step 1: Define the Type

```python
# In kompass/kompass/data_types.py (or your own module)
from ros_sugar.supported_types import SupportedType
from sensor_msgs.msg import Imu as ROSImu
from .callbacks import MyImuCallback


class Imu(SupportedType):
    """Support for sensor_msgs/Imu messages."""

    _ros_type = ROSImu
    callback = MyImuCallback

    @classmethod
    def convert(cls, orientation_yaw: float, angular_velocity: float, **_) -> ROSImu:
        """Convert component data back to a ROS Imu message for publishing."""
        msg = ROSImu()
        msg.orientation.z = np.sin(orientation_yaw / 2)
        msg.orientation.w = np.cos(orientation_yaw / 2)
        msg.angular_velocity.z = angular_velocity
        return msg
```

**Three things to define:**

| Attribute | Purpose |
|---|---|
| `_ros_type` | The ROS 2 message class (used for subscription/publisher creation) |
| `callback` | Your callback class (used when subscribing to this message type) |
| `convert()` | Classmethod that creates a ROS message from component data (used when publishing) |

If your type is input-only (no publishing), you can skip `convert()` -- the base class returns the raw output as-is.

If your type is output-only (no custom processing), you can use `GenericCallback` as the callback.

### Step 2: Register the Type

Types defined in `kompass/kompass/data_types.py` are registered automatically at import time. The registration happens in `kompass.components.defaults`:

```python
from ros_sugar.supported_types import add_additional_datatypes
from ros_sugar.io import get_all_msg_types
from .. import data_types

add_additional_datatypes(get_all_msg_types(data_types))
```

`get_all_msg_types(data_types)` introspects the module and collects all `SupportedType` subclasses. So for types added directly to `data_types.py`, you just need to add the class name to `__all__` -- no manual registration call needed.

For types defined **outside** `kompass.data_types` (e.g. in your own package), register explicitly:

```python
from ros_sugar.supported_types import add_additional_datatypes

add_additional_datatypes([Imu])
```

After registration, the type name works as a string in `Topic`:

```python
Topic(name="/imu", msg_type="Imu")
```

### Step 3: Use in a Component

```python
# In allowed inputs
my_allowed_inputs = {
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["Imu"]),
}

# In default topics
my_default_inputs = {
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/imu", msg_type="Imu"),
}

# In _execution_step
def _execution_step(self):
    imu_cb = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
    if imu_cb and imu_cb.got_msg:
        imu_data = imu_cb.get_output()  # Returns your dict from _get_output()
```

## Processing Pipelines

Both callbacks and publishers support processing pipelines for chaining transformations.

### Post-Processors (Callbacks)

Post-processors run after `_get_output()`, transforming the result before it reaches your component:

```python
def clip_scan_ranges(scan_data: LaserScanData) -> LaserScanData:
    """Clip laser scan to 5m max range."""
    scan_data.ranges = scan_data.ranges.clip(max=5.0)
    return scan_data

# Attach to a callback
callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
callback.add_post_processors([clip_scan_ranges])

# Now get_output() returns clipped data
data = callback.get_output()
```

**Rules:**
- Each processor receives the output of the previous one.
- The return type must match the input type.
- If any processor returns `None`, `get_output()` returns `None`.

### Pre-Processors (Publishers)

Pre-processors run before `convert()`, transforming component data before it becomes a ROS message:

```python
def clamp_velocity(vx, vy, omega):
    """Clamp velocity commands to safe limits."""
    return (
        max(-1.0, min(1.0, vx)),
        max(-0.5, min(0.5, vy)),
        max(-2.0, min(2.0, omega)),
    )

# Attach to a publisher
publisher = self.get_publisher(TopicsKeys.INTERMEDIATE_CMD)
publisher.add_pre_processors([clamp_velocity])

# Now publish() clamps before converting
publisher.publish(vx, vy, omega)
```

**Rules:**
- Each processor receives the output tuple of the previous one.
- The return types must match the input types.
- If any processor returns `None`, the message is not published.

### Event-Triggered Callbacks

You can attach a function that fires every time a message arrives, before `get_output()` is called:

```python
def on_new_path(msg, topic, output):
    """Called whenever a new path message arrives."""
    self._controller.set_path(output)

callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
callback.on_callback_execute(on_new_path, get_processed=True)
```

Parameters passed to your function:
- `msg` -- the raw ROS message
- `topic` -- the `Topic` object
- `output` -- the processed output from `_get_output()` (only if `get_processed=True`)

## Config Bridging Patterns

Algorithm configuration classes in `kompass-core` (e.g., `DWAConfig`, `PurePursuitConfig`) are `attrs` `@define` classes. Kompass re-exports them directly:

```python
# kompass/kompass/control.py
from kompass_core.control import (
    ControllersID,
    DWAConfig,
    PurePursuitConfig,
    StanleyConfig,
    DVZConfig,
    VisionRGBFollowerConfig,
    VisionRGBDFollowerConfig,
)
```

At runtime, the Controller component looks up the corresponding config class from `ControlConfigClasses[config.algorithm]` and instantiates it, either from defaults or from a YAML/JSON/TOML configuration file. This keeps algorithm configuration decoupled from the component framework while allowing full configuration via Kompass's standard file-based config system.

## GPU Acceleration via SYCL

`kompass-core` algorithms that operate on spatial data (local mapping, DWA obstacle checking, etc.) can leverage GPU acceleration through [AdaptiveCpp](https://github.com/AdaptiveCpp/AdaptiveCpp) (SYCL). The acceleration is transparent to the data type layer:

- The same `LaserScanData` / `PointCloudData` types are used regardless of CPU or GPU execution.
- GPU memory transfers are handled internally by `kompass-core`'s C++ layer.
- No code changes are needed in the Kompass Python layer to enable GPU acceleration.

## Complete Example: Adding a Custom Sensor Type

```python
# --- callbacks.py ---
from typing import Optional
import numpy as np
from ros_sugar.io import GenericCallback


class UltrasonicCallback(GenericCallback):
    """Processes sensor_msgs/Range into a distance float."""

    def __init__(self, input_topic, node_name=None):
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[float]:
        if not self.msg:
            return None
        # Clamp to valid range
        return float(np.clip(
            self.msg.range,
            self.msg.min_range,
            self.msg.max_range,
        ))


# --- data_types.py ---
from ros_sugar.supported_types import SupportedType, add_additional_datatypes
from sensor_msgs.msg import Range as ROSRange
from .callbacks import UltrasonicCallback


class Ultrasonic(SupportedType):
    _ros_type = ROSRange
    callback = UltrasonicCallback

    @classmethod
    def convert(cls, distance: float, **_) -> ROSRange:
        msg = ROSRange()
        msg.range = distance
        return msg

# Register at import time
add_additional_datatypes([Ultrasonic])


# --- component.py ---
from kompass.components.component import Component
from kompass.components.defaults import TopicsKeys
from ros_sugar.io import AllowedTopics, Topic

class ProximityMonitor(Component):
    def __init__(self, **kwargs):
        super().__init__(
            component_name="proximity_monitor",
            inputs={
                TopicsKeys.SPATIAL_SENSOR: Topic(
                    name="/ultrasonic", msg_type="Ultrasonic"
                ),
            },
            allowed_inputs={
                TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["Ultrasonic"]),
            },
            **kwargs,
        )

    def _execution_step(self):
        cb = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
        if cb and cb.got_msg:
            distance = cb.get_output()  # Returns float from UltrasonicCallback
            if distance < 0.3:
                self.health_status.set_fail_algorithm(
                    algorithm_names=["ProximityCheck"]
                )
```
