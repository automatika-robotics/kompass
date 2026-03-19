---
title: Adding a Python Control Algorithm
---

# Adding a Python Control Algorithm

This guide walks through adding a new control algorithm in Python. For implementing algorithms in C++ with nanobind bindings, see the [C++ Algorithm Guide](./adding_cpp_algorithms.md).

Control algorithms live in [`kompass-core`](https://github.com/automatika-robotics/kompass-core) and are registered in Kompass through a set of enum mappings and configuration bridges.

## Architecture Overview

The Controller component (`kompass.components.controller.Controller`) dispatches to algorithms defined in `kompass_core.control`. The dispatch mechanism relies on two key registries:

- **`ControlClasses`** -- a dictionary mapping each `ControllersID` enum member to its algorithm implementation class.
- **`ControlConfigClasses`** -- a dictionary mapping each `ControllersID` enum member to its configuration class.

Both are defined in `kompass_core.control` and imported into the Controller component.

### Existing Algorithms

| `ControllersID` | Algorithm | Base Class | Config Class |
|---|---|---|---|
| `DWA` | Dynamic Window Approach | `FollowerTemplate` | `DWAConfig` |
| `PURE_PURSUIT` | Pure Pursuit | `FollowerTemplate` | `PurePursuitConfig` |
| `STANLEY` | Stanley Controller | `FollowerTemplate` | `StanleyConfig` |
| `DVZ` | Deformable Virtual Zone | `FollowerTemplate` | `DVZConfig` |
| `VISION_IMG` | Vision Follower (RGB) | `ControllerTemplate` | `VisionRGBFollowerConfig` |
| `VISION_DEPTH` | Vision Follower (RGB-D) | `ControllerTemplate` | `VisionRGBDFollowerConfig` |

### Base Classes

There are two base classes for control algorithms, depending on the type of algorithm:

- **`ControllerTemplate`** -- the generic base class for any control algorithm. It defines the minimal interface: `loop_step`, `logging_info`, and velocity output properties. Use this when your algorithm is self-contained and manages its own control logic entirely in Python.

- **`FollowerTemplate`** (extends `ControllerTemplate`) -- a specialized base for path-following algorithms. It adds a `planner` property that holds a `kompass_cpp.control.Follower` C++ backend object. The base class provides ready-made implementations of `set_path`, `reached_end`, `path`, `tracked_state`, `distance_error`, and `orientation_error` -- all delegating to the C++ planner. Use this when your algorithm builds on the `kompass_cpp` path-following infrastructure.

### Controller Modes

The `ControllerMode` enum in `kompass.components.controller` determines the operating mode:

- **`PATH_FOLLOWER`** -- algorithms that track a `nav_msgs/Path` (DWA, Pure Pursuit, Stanley, DVZ).
- **`VISION_FOLLOWER`** -- algorithms that track a visual target (VisionFollowerRGB, VisionFollowerRGBD).

The mode is set automatically based on the selected `ControllersID`, but can also be set explicitly via `ControllerConfig._mode`.

## Option A: Pure Python Algorithm

Use this approach when your algorithm is self-contained and does not require a C++ path-following backend. Your class inherits from `ControllerTemplate` and implements all control logic in Python.

### 1. Define the Config

Create an `attrs` config class. Since this is a standalone controller, you don't need to inherit from `FollowerConfig` -- define whatever parameters your algorithm needs:

```python
# In kompass_core/control/my_algorithm.py
from attrs import define, field

@define
class MyAlgorithmConfig:
    """Configuration for MyAlgorithm."""
    control_time_step: float = field(default=0.1)
    gain: float = field(default=1.0)
    lookahead: float = field(default=0.5)
```

### 2. Implement the Algorithm

Inherit from `ControllerTemplate` and implement all abstract members:

```python
from typing import Optional, Union, List
import numpy as np
from kompass_core.control._base_ import ControllerTemplate
from kompass_core.models import Robot, RobotCtrlLimits, RobotState

class MyAlgorithm(ControllerTemplate):
    def __init__(
        self,
        robot: Robot,
        ctrl_limits: RobotCtrlLimits,
        config: Optional[MyAlgorithmConfig] = None,
        config_file: Optional[str] = None,
        config_root_name: Optional[str] = None,
        **_,
    ):
        self._robot = robot
        self._ctrl_limits = ctrl_limits
        self._config = config or MyAlgorithmConfig()

        if config_file:
            self._config.from_file(
                file_path=config_file, nested_root_name=config_root_name
            )

        self._vx: List[float] = [0.0]
        self._vy: List[float] = [0.0]
        self._omega: List[float] = [0.0]

    def loop_step(self, *, current_state: RobotState, **_) -> bool:
        """One control iteration. Returns True if a valid command was found."""
        # Your control logic here -- compute self._vx, self._vy, self._omega
        # from current_state and whatever target/input your algorithm uses.
        ...
        return True

    def logging_info(self) -> str:
        return f"MyAlgorithm cmd: vx={self._vx}, vy={self._vy}, omega={self._omega}"

    @property
    def linear_x_control(self) -> Union[List[float], np.ndarray]:
        return self._vx

    @property
    def linear_y_control(self) -> Union[List[float], np.ndarray]:
        return self._vy

    @property
    def angular_control(self) -> Union[List[float], np.ndarray]:
        return self._omega
```

The `loop_step` method is where all the work happens. The Controller component calls it on every control cycle, passing the current robot state. Your implementation computes velocity commands and stores them so the velocity properties can return them.

### Single Command vs. Command Sequence (MPC-style)

The velocity properties (`linear_x_control`, `linear_y_control`, `angular_control`) return **lists**, not single values. This is by design -- your algorithm can return either:

- **A single command** -- e.g. `[0.5]` -- the Controller publishes one velocity per cycle. This is the simpler case, suited for reactive controllers.

- **A sequence of commands** -- e.g. `[0.5, 0.4, 0.3, 0.2]` -- the Controller publishes the entire sequence over multiple time steps. This enables **MPC-style** (Model Predictive Control) algorithms that compute an optimal trajectory over a horizon.

When returning a command sequence, the relevant configuration parameters are:

| Parameter | Description |
|---|---|
| `control_time_step` | Time interval between consecutive commands in the sequence (seconds) |
| `control_horizon` | Number of time steps of commands to apply before recomputing |
| `prediction_horizon` | Number of time steps the algorithm looks ahead when optimizing |

For example, DWA uses this pattern: it samples trajectories over a `prediction_horizon`, selects the best one, and returns commands up to the `control_horizon`:

```python
# DWA returns multiple commands up to the control horizon
@property
def linear_x_control(self) -> Union[List[float], np.ndarray]:
    if self._result.is_found:
        return self.control_till_horizon.vx[: self._end_of_ctrl_horizon]
    return [0.0]
```

The Controller component handles multi-command output through three publishing modes (configured via `ControllerConfig.ctrl_publish_type`):

| `CmdPublishType` | Behavior |
|---|---|
| `TWIST_SEQUENCE` | Publishes commands one by one, blocking between each (`control_time_step` apart) |
| `TWIST_PARALLEL` | Publishes commands one by one in a separate thread, while the algorithm computes the next batch |
| `TWIST_ARRAY` | Publishes all commands at once as a `TwistArray` message |

### 3. Register, Export, and Use

Follow the [common registration steps](#common-registration-steps) below to make the algorithm available in Kompass.

---

## Option B: C++ Backed Path Follower

Use this approach when your algorithm builds on the `kompass_cpp` path-following infrastructure. Your class inherits from `FollowerTemplate`, which provides ready-made path management (setting paths, tracking progress, checking goal reached) via a C++ `kompass_cpp.control.Follower` object.

This option requires two pieces of work:

1. **A C++ Follower implementation** in `kompass_cpp` with nanobind bindings (covered in the [Adding a C++ Algorithm](./adding_cpp_algorithms.md) guide).
2. **A Python wrapper class** in `kompass_core` that inherits from `FollowerTemplate`.

The steps below cover the Python wrapper, assuming the C++ side is already in place.

### 1. Define the Config

Your config inherits from `FollowerConfig`, which provides shared path-following parameters (`control_time_step`, `wheel_base`, `goal_dist_tolerance`, `goal_orientation_tolerance`, `path_segment_length`, etc.). Add your algorithm-specific parameters on top:

```python
# In kompass_core/control/my_follower.py
from attrs import define, field
from kompass_core.control._base_ import FollowerConfig

@define
class MyFollowerConfig(FollowerConfig):
    """Configuration for MyFollower."""
    gain: float = field(default=1.0)
    prediction_horizon: float = field(default=1.0)
```

### 2. Implement the Wrapper

Inherit from `FollowerTemplate`. The key addition compared to Option A is the `planner` property, which returns your C++ follower object. The base class uses this property to handle `set_path`, `reached_end`, `path`, `tracked_state`, `distance_error`, and `orientation_error` automatically.

```python
from typing import Optional, Union, List
import numpy as np
import kompass_cpp
from kompass_core.control._base_ import FollowerTemplate
from kompass_core.models import Robot, RobotCtrlLimits, RobotState, RobotType

class MyFollower(FollowerTemplate):
    def __init__(
        self,
        robot: Robot,
        ctrl_limits: RobotCtrlLimits,
        config: Optional[MyFollowerConfig] = None,
        config_file: Optional[str] = None,
        config_root_name: Optional[str] = None,
        control_time_step: Optional[float] = None,
        **_,
    ):
        self._robot = robot
        self._config = config or MyFollowerConfig(wheel_base=robot.wheelbase)

        if config_file:
            self._config.from_file(
                file_path=config_file, nested_root_name=config_root_name
            )

        if control_time_step:
            self._config.control_time_step = control_time_step

        self._control_time_step = self._config.control_time_step

        # Initialize the C++ planner -- this is YOUR C++ implementation
        # bound via nanobind (see the Adding a C++ Algorithm guide)
        self._planner = kompass_cpp.control.MyFollower(
            self._config.to_kompass_cpp()
        )

        # Set control limits on the C++ side
        self._planner.set_linear_ctr_limits(
            ctrl_limits.linear_to_kompass_cpp_lib(ctrl_limits.vx_limits),
            ctrl_limits.linear_to_kompass_cpp_lib(ctrl_limits.vy_limits),
        )
        self._planner.set_angular_ctr_limits(
            ctrl_limits.angular_to_kompass_cpp_lib()
        )

        # Initialize the result container
        self._result = kompass_cpp.control.FollowingResult()

    @property
    def planner(self) -> kompass_cpp.control.Follower:
        """The C++ backend. FollowerTemplate uses this for path management."""
        return self._planner

    def loop_step(self, *, current_state: RobotState, **_) -> bool:
        """One control iteration."""
        self._planner.set_current_state(
            current_state.x, current_state.y,
            current_state.yaw, current_state.speed,
        )
        if self.reached_end():
            return True

        self._result = self._planner.compute_velocity_commands(
            self._control_time_step
        )
        return (
            self._result.status
            == kompass_cpp.control.FollowingStatus.COMMAND_FOUND
        )

    def logging_info(self) -> str:
        return (
            f"MyFollower status: {self._result.status}, "
            f"cmd: {self._result.velocity_command}"
        )

    @property
    def linear_x_control(self) -> Union[List[float], np.ndarray]:
        return [self._planner.get_vx_cmd()]

    @property
    def linear_y_control(self) -> Union[List[float], np.ndarray]:
        return [self._planner.get_vy_cmd()]

    @property
    def angular_control(self) -> Union[List[float], np.ndarray]:
        return [self._planner.get_omega_cmd()]
```

**What `FollowerTemplate` gives you for free** (via the `planner` property):

| Method / Property | What it does |
|---|---|
| `set_path(global_path)` | Parses a `nav_msgs/Path` and passes it to the C++ planner |
| `reached_end()` | Checks if the C++ planner considers the goal reached |
| `path` | Whether the planner currently has a path |
| `tracked_state` | The state the planner is currently tracking on the path |
| `distance_error` | Lateral cross-track error (m) |
| `orientation_error` | Heading error (rad) |
| `interpolated_path()` | Returns the interpolated path from the planner |
| `set_interpolation_type(...)` | Sets the path interpolation method |

### 3. Register, Export, and Use

Follow the [common registration steps](#common-registration-steps) below.

---

## Common Registration Steps

These steps are the same regardless of whether you chose Option A or Option B.

### Register in kompass-core

Add a new member to the `ControllersID` enum and register both the algorithm class and its config class in the dispatch dictionaries in `kompass_core/control/__init__.py`:

```python
# In kompass_core/control/__init__.py

class ControllersID(StrEnum):
    STANLEY = "Stanley"
    DWA = "DWA"
    DVZ = "DVZ"
    VISION_IMG = "VisionRGBFollower"
    VISION_DEPTH = "VisionRGBDFollower"
    PURE_PURSUIT = "PurePursuit"
    MY_ALGORITHM = "MyAlgorithm"  # <-- Add this

ControlClasses = {
    # ... existing entries ...
    ControllersID.MY_ALGORITHM: MyAlgorithm,  # <-- Add this
}

ControlConfigClasses = {
    # ... existing entries ...
    ControllersID.MY_ALGORITHM: MyAlgorithmConfig,  # <-- Add this
}
```

### Export the Config in Kompass

Add the new config class to `kompass/kompass/control.py` so users can import it:

```python
# In kompass/kompass/control.py
from kompass_core.control import (
    ControllersID,
    StanleyConfig,
    DWAConfig,
    DVZConfig,
    VisionRGBFollowerConfig,
    VisionRGBDFollowerConfig,
    PurePursuitConfig,
    MyAlgorithmConfig,  # <-- Add this
)
```

And add it to the `__all__` list in the same file.

### Use the Algorithm

Once registered, the algorithm is available through standard configuration:

```python
from kompass.components import Controller, ControllerConfig

controller = Controller(
    component_name="my_controller",
    config=ControllerConfig(algorithm="MyAlgorithm"),
)
```

Or via YAML configuration:

```yaml
my_controller:
  algorithm: "MyAlgorithm"
  control_time_step: 0.1
```

## How the Controller Instantiates Algorithms

When the Controller component starts, it looks up the algorithm class and config class from the registries and instantiates them:

```python
# Simplified from kompass/kompass/components/controller.py
_controller_config = ControlConfigClasses[self.algorithm](**config_kwargs)
self.__path_controller = ControlClasses[self.algorithm](
    robot=self.__robot,
    config=_controller_config,
    ctrl_limits=self.__robot_ctr_limits,
    config_file=self._config_file,
    config_root_name=f"{self.node_name}.{self.config.algorithm}",
    control_time_step=self.config.control_time_step,
)
```

This is why your constructor must accept `robot`, `ctrl_limits`, `config`, `config_file`, `config_root_name`, and `**kwargs`.

## Configuration Pattern

The `ControllerConfig` class uses `ControllersID` for the `algorithm` field with an automatic string-to-enum converter:

```python
@define
class ControllerConfig(ComponentConfig):
    algorithm: Union[ControllersID, str] = field(
        default=ControllersID.DWA,
        converter=lambda value: (
            ControllersID(value) if isinstance(value, str) else value
        ),
    )
```

This means users can pass either the enum member or a plain string matching the enum value.

## Testing

After adding your algorithm, verify that:

1. The `ControllersID` enum resolves your algorithm name correctly.
2. `ControlClasses[ControllersID.MY_ALGORITHM]` returns your implementation class.
3. `ControlConfigClasses[ControllersID.MY_ALGORITHM]` returns your config class.
4. The Controller component can instantiate and run your algorithm end-to-end.
5. The `loop_step` method returns the expected result on each control cycle.
6. The `linear_x_control`, `linear_y_control`, and `angular_control` properties return valid commands after a `loop_step` call.
