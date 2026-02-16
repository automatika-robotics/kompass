# Stanley Steering

**Front-wheel feedback control for path tracking.**

Stanley is a geometric path tracking method originally developed for the DARPA Grand Challenge.[^1] Unlike [Pure Pursuit](./pure_pursuit.md) (which looks ahead), Stanley uses the **Front Axle** as its reference point to calculate steering commands.

It computes a steering angle $\delta(t)$ based on two error terms:
1.  **Heading Error** ($\psi_e$): Difference between the robot's heading and the path direction.
2.  **Cross-Track Error** ($e$): Lateral distance from the front axle to the nearest path segment.

The control law combines these to minimize error exponentially:

$$
\delta(t) = \psi_e(t) + \arctan \left( \frac{k \cdot e(t)}{v(t)} \right)
$$

## Key Features

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`directions_car` Ackermann Native</span> - Designed specifically for car-like steering geometry. It is naturally stable at high speeds for these vehicles.
- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`rotate_right` Multi-Model Support</span> - Kompass extends Stanley to Differential and Omni robots by applying a **Rotate-Then-Move** strategy when orientation errors are large.
- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`blind` Sensor-Less</span> - Does not require LiDAR or depth data. It is a pure path follower.


## Configuration Parameters


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
from kompass.control import ControllersID

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
controller_config = ControllerConfig(algorithm="Stanley")  # or ControllersID.STANLEY

# Set YAML config file
config_file = "my_config.yaml"

controller = Controller(component_name="my_controller",
                        config=controller_config,
                        config_file=config_file)

# algorithm can also be set using a property
controller.algorithm = ControllersID.STANLEY      # or "Stanley"

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

:::{admonition} Safety Note
:class: warning
Stanley does **not** have built-in obstacle avoidance. It is strongly recommended to use this controller in conjunction with the **[Drive Manager](https://www.google.com/search?q=drive_manager.md)** to provide Emergency Stop and Slowdown safety layers.
:::


[^1]:  [Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing." American Control Conference. 2007, pp. 2296–2301](https://ieeexplore.ieee.org/document/4282788)
