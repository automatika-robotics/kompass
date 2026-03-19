---
title: Adding a C++ Control Algorithm
---

# Adding a C++ Control Algorithm

This guide covers how to add a new path-following algorithm in C++ to `kompass_cpp` and expose it to Python via nanobind bindings. Once the C++ side is in place, wrap it in Python following the [Python Algorithm Guide -- Option B](./adding_python_algorithms.md#option-b-c-backed-path-follower).

## C++ Class Hierarchy

All control algorithms in `kompass_cpp` live under the `Kompass::Control` namespace:

```
Controller                     (controller.h)
  ├─ setCurrentState()         Update robot pose
  ├─ setLinearControlLimits()  Set velocity bounds
  ├─ setAngularControlLimits() Set angular bounds
  ├─ restrictVelocityTolimits()  Clamp velocity to limits
  └─ Result { Status, Velocity2D }
      │
      └─ Follower              (follower.h)
           ├─ setCurrentPath()       Receive and interpolate a global path
           ├─ isGoalReached()        Check if end of path is reached
           ├─ determineTarget()      Find the tracked point on the path
           ├─ getTrackedTarget()     Get crosstrack/heading errors
           ├─ getLinearVelocityCmdX/Y()  Clamped velocity getters
           ├─ getAngularVelocityCmd()
           └─ FollowerParameters     Configuration (inherits Parameters)
                │
                └─ YourAlgorithm     (your_algorithm.h)
                     ├─ computeVelocityCommand(timeStep)
                     ├─ execute(currentPosition, deltaTime)
                     └─ YourAlgorithmParameters
```

The key base classes:

- **`Controller`** -- manages robot state, control type (Ackermann / Differential / Omni), and velocity limits. Provides `restrictVelocityTolimits()` for clamping computed velocities.
- **`Follower`** (extends `Controller`) -- adds path management: interpolation, segmentation, closest-point tracking, and goal checking. Your algorithm inherits from this.

## Step-by-Step

### 1. Create the Header

Create `kompass_cpp/include/controllers/your_algorithm.h`:

```cpp
#pragma once

#include "controllers/follower.h"
#include "datatypes/control.h"
#include "datatypes/parameter.h"
#include "datatypes/path.h"

namespace Kompass {
namespace Control {

class YourAlgorithm : public Follower {
public:
  // --- Parameters ---
  class YourAlgorithmParameters : public Follower::FollowerParameters {
  public:
    YourAlgorithmParameters() : Follower::FollowerParameters() {
      // addParameter(name, Parameter(default, min, max))
      addParameter("my_gain", Parameter(1.0, 0.0, 100.0));
      addParameter("my_threshold", Parameter(0.05, 0.001, 10.0));
    }
  };

  // --- Constructors ---
  YourAlgorithm();
  YourAlgorithm(YourAlgorithmParameters config);

  ~YourAlgorithm() = default;

  /// Main entry point: compute a velocity command for the given time step.
  Controller::Result computeVelocityCommand(double timeStep);

  /// Convenience: set state + compute in one call.
  Controller::Result execute(Path::State currentPosition, double deltaTime);

protected:
  YourAlgorithmParameters config;

  // Algorithm-specific members
  double my_gain{1.0};
  double my_threshold{0.05};
};

} // namespace Control
} // namespace Kompass
```

### 2. Implement the Algorithm

Create `kompass_cpp/src/controllers/your_algorithm.cpp`:

```cpp
#include "controllers/your_algorithm.h"
#include "utils/angles.h"
#include "utils/logger.h"
#include <algorithm>
#include <cmath>

namespace Kompass {
namespace Control {

YourAlgorithm::YourAlgorithm() : Follower() {
  // Read parameters from the config map
  my_gain = config.getParameter<double>("my_gain");
  my_threshold = config.getParameter<double>("my_threshold");
}

YourAlgorithm::YourAlgorithm(YourAlgorithmParameters config)
    : YourAlgorithm() {
  setParams(config);
}

Controller::Result YourAlgorithm::execute(Path::State currentPosition,
                                          double deltaTime) {
  setCurrentState(currentPosition);
  return computeVelocityCommand(deltaTime);
}

Controller::Result YourAlgorithm::computeVelocityCommand(double timeStep) {
  // No path loaded yet
  if (!path_processing_) {
    return {(reached_goal_ ? Result::Status::GOAL_REACHED
                           : Result::Status::NO_COMMAND_POSSIBLE),
            {0.0, 0.0, 0.0}};
  }

  // Find the tracked target on the path (crosstrack error, heading error, etc.)
  determineTarget();
  const Target target = *currentTrackedTarget_;

  // --- Your control law ---
  double target_speed = target.reverse ? -ctrlimitsParams.velXParams.maxVel
                                       : ctrlimitsParams.velXParams.maxVel;

  double steering = my_gain * Angle::normalizeToMinusPiPlusPi(target.heading_error);

  // Clamp to velocity limits using the inherited helper
  double vx_cmd = restrictVelocityTolimits(
      latest_velocity_command_.vx(), target_speed,
      ctrlimitsParams.velXParams.maxAcceleration,
      ctrlimitsParams.velXParams.maxDeceleration,
      ctrlimitsParams.velXParams.maxVel, timeStep);

  double omega_cmd = restrictVelocityTolimits(
      latest_velocity_command_.omega(), steering,
      ctrlimitsParams.omegaParams.maxAcceleration,
      ctrlimitsParams.omegaParams.maxDeceleration,
      ctrlimitsParams.omegaParams.maxOmega, timeStep);

  latest_velocity_command_ = Control::Velocity2D{vx_cmd, 0.0, omega_cmd, 0.0};

  // Update segment tracking
  current_segment_index_ = target.segment_index;
  current_position_in_segment_ = target.position_in_segment;

  return {Result::Status::COMMAND_FOUND, latest_velocity_command_};
}

} // namespace Control
} // namespace Kompass
```

**Key inherited members you will use:**

| Member | Type | Description |
|---|---|---|
| `currentState` | `Path::State` | Current robot pose (x, y, yaw) |
| `currentVel` | `Velocity2D` | Current robot velocity |
| `ctrlimitsParams` | `ControlLimitsParams` | Velocity/acceleration limits |
| `currentTrackedTarget_` | `unique_ptr<Target>` | Tracked point on the path |
| `latest_velocity_command_` | `Velocity2D` | Last issued command (for smoothing) |
| `path_processing_` | `bool` | Whether a valid path is loaded |
| `reached_goal_` | `bool` | Whether the goal was reached |
| `determineTarget()` | method | Compute the tracked target from current state |
| `restrictVelocityTolimits()` | method | Clamp a velocity respecting acc/decel limits |

**The `Target` struct** (computed by `determineTarget()`):

| Field | Type | Description |
|---|---|---|
| `crosstrack_error` | `double` | Lateral distance to the path (m) |
| `heading_error` | `double` | Orientation error to the path tangent (rad) |
| `movement` | `Path::State` | The tracked point's pose on the path |
| `reverse` | `bool` | Whether driving in reverse |
| `lookahead` | `double` | Lookahead distance used |
| `segment_index` | `size_t` | Current path segment index |

**The `Result` struct**:

| Field | Values |
|---|---|
| `status` | `COMMAND_FOUND`, `GOAL_REACHED`, `LOOSING_GOAL`, `NO_COMMAND_POSSIBLE` |
| `velocity_command` | `Velocity2D` with `vx()`, `vy()`, `omega()`, `steer_ang()` |

### 3. Add nanobind Bindings

Add your algorithm's bindings to `bindings/bindings_control.cpp`. There are two parts: the parameters class and the algorithm class.

```cpp
// At the top, add the include
#include "controllers/your_algorithm.h"

// Inside bindings_control(), add:

// Parameters binding -- inherit from FollowerParameters
py::class_<Control::YourAlgorithm::YourAlgorithmParameters,
           Control::Follower::FollowerParameters>(
    m_control, "YourAlgorithmParameters")
    .def(py::init<>());

// Algorithm binding -- inherit from Follower
py::class_<Control::YourAlgorithm, Control::Follower>(
    m_control, "YourAlgorithm")
    .def(py::init<>(),
         "Init with default parameters")
    .def(py::init<Control::YourAlgorithm::YourAlgorithmParameters>(),
         "Init with custom config")
    .def("compute_velocity_commands",
         &Control::YourAlgorithm::computeVelocityCommand,
         py::rv_policy::reference_internal)
    .def("execute", &Control::YourAlgorithm::execute);
```

**Key binding patterns:**

- **Parameter classes** always inherit from their parent's parameter class in the binding (`FollowerParameters` for followers).
- The `Parameters` base class already exposes `from_dict()` to Python, which is how the Python config's `to_kompass_cpp()` method passes values to C++.
- Use `py::rv_policy::reference_internal` for methods that return references to internal state (like `computeVelocityCommand`).
- The inherited `Follower` bindings (`set_current_path`, `is_goal_reached`, `get_vx_cmd`, etc.) are automatically available -- you only need to bind your algorithm-specific methods.

### 4. Build

The build system uses CMake with scikit-build-core. Source files are auto-discovered via `file(GLOB_RECURSE)`, so you only need to place your `.h` and `.cpp` files in the right directories:

```
kompass_cpp/
├── include/controllers/your_algorithm.h    # <-- header here
├── src/controllers/your_algorithm.cpp      # <-- implementation here
bindings/
└── bindings_control.cpp                    # <-- add bindings here
```

Build and install:

```bash
cd kompass-navigation
pip install -e .
```

Verify the binding is accessible:

```python
import kompass_cpp
algo = kompass_cpp.control.YourAlgorithm()
print(algo)  # Should print the object
```

## Parameter System

The `Parameters` / `Parameter` classes form the C++ configuration system. Each parameter has a default value, min/max bounds, and an optional description.

```cpp
// In your Parameters constructor:
addParameter("name", Parameter(default, min, max));     // double or int
addParameter("flag", Parameter(true));                   // bool (no min/max)
addParameter("label", Parameter(std::string("value")));  // string

// Read in your algorithm:
double val = config.getParameter<double>("name");

// Set from Python via from_dict() -- automatic via nanobind
```

The Python side uses `to_kompass_cpp()` on the `attrs` config to convert to the C++ parameters:

```python
def to_kompass_cpp(self) -> kompass_cpp.control.YourAlgorithmParameters:
    params = kompass_cpp.control.YourAlgorithmParameters()
    params.from_dict(self.asdict())  # Maps attrs fields -> C++ parameters by name
    return params
```

**Important:** The keys in the Python `attrs` config must match the C++ parameter names exactly (e.g., `my_gain` in both places).

## Complete Example: Stanley

The Stanley controller is a good reference for the full pattern:

| Layer | File |
|---|---|
| C++ header | `kompass_cpp/include/controllers/stanley.h` |
| C++ implementation | `kompass_cpp/src/controllers/stanley.cpp` |
| nanobind bindings | `bindings/bindings_control.cpp` (search for `Stanley`) |
| Python wrapper | `kompass_core/control/stanley.py` |
| Python config | `kompass_core/control/stanley.py` (`StanleyConfig`) |

## Next Steps

Once the C++ algorithm and bindings are in place:

1. **Write the Python wrapper** -- see [Python Algorithm Guide -- Option B](./adding_python_algorithms.md#option-b-c-backed-path-follower).
2. **Register in kompass-core** -- see [Common Registration Steps](./adding_python_algorithms.md#common-registration-steps).
3. **Test** -- verify the algorithm works end-to-end through the Controller component.
