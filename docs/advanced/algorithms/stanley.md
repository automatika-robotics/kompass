# Stanley Steering

Stanley is a pure path following method designed to track a reference by computing an orientation and cross-track errors [^1]

:::{tip} Although Stanley is a pure path following method, it can be used in KOMPASS in the Controller component in combination with the Drive Manager Component to provide following + emergency stop.
:::

## Supported Motion Models

- ACKERMANN
- DIFFERENTIAL_DRIVE
- OMNI


:::{note} Stanley is designed for ACKERMANN models, however, it is adjusted for all models in Kompass by adapting a rotate-then-move strategy for non ACKERMANN robot.
:::

## Supported Sensory Inputs

Does not require sensory input for obstacles

## Parameters and Default Values

```{list-table}
:widths: 10 10 10 70
:header-rows: 1

* - Name
  - Type
  - Default
  - Description

* - heading_gain
  - `float`
  - `0.7`
  - Heading gain in the control law. Must be between `0.0` and `1e2`.

* - cross_track_min_linear_vel
  - `float`
  - `0.05`
  - Minimum linear velocity for cross-track control (m/s). Must be between `1e-4` and `1e2`.

* - min_angular_vel
  - `float`
  - `0.01`
  - Minimum allowable angular velocity (rad/s). Must be between `0.0` and `1e9`.

* - cross_track_gain
  - `float`
   - `1.5`
  - Gain for cross-track in the control law. Must be between `0.0` and `1e2`.

* - max_angle_error
  - `float`
  - `np.pi / 16`
  - Maximum allowable angular error (rad). Must be between `1e-9` and `π`.

* - max_distance_error
  - `float`
  - `0.1`
  - Maximum allowable distance error (m). Must be between `1e-9` and `1e9`.

```

## usage Example

Stanley algorithm can be used in the [Controller](../../navigation/control.md) component by setting 'algorithm' property or component config parameter. The Controller will configure Stanley algorithm using the default values of all the previous configuration parameters. To configure custom values of the parameters, a YAML file is passed to the component.


```{code-block} python
:caption: stanley.py

from kompass.components import Controller, ControllerConfig
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig
)
from kompass.control import LocalPlannersID

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([0.3, 0.3, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.2, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3)
)

# Set Stanley algorithm using the config class
controller_config = ControllerConfig(algorithm="Stanley")  # or LocalPlannersID.STANLEY

# Set YAML config file
config_file = "my_config.yaml"

controller = Controller(component_name="my_controller",
                        config=controller_config,
                        config_file=config_file)

# algorithm can also be set using a property
controller.algorithm = LocalPlannersID.STANLEY      # or "Stanley"

```

```{code-block} yaml
:caption: my_config.yaml

my_controller:
  # Component config parameters
  loop_rate: 10.0
  control_time_step: 0.1
  ctrl_publish_type: 'Sequence'

  # Algorithm parameters under the algorithm name
  Stanley:
    cross_track_gain: 1.0
    heading_gain: 2.0
```


[^1]:  [Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing." American Control Conference. 2007, pp. 2296–2301](https://ieeexplore.ieee.org/document/4282788)
