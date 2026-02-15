# DVZ

**Fast, reactive collision avoidance for dynamic environments.**

The DVZ (Deformable Virtual Zone) is a reactive control method introduced by R. Zapata in 1994.[^1] It models the robot's safety perimeter as a "virtual bubble" (zone) that deforms when obstacles intrude.

Unlike sampling methods (like DWA) that simulate future trajectories, DVZ calculates a reaction vector based directly on how the bubble is being "squished" by the environment. This makes it **extremely computationally efficient** and ideal for crowded, fast-changing environments where rapid reactivity is more important than global optimality.

[^1]: [Zapata, R., Lépinay, P., and Thompson, P. “Reactive behaviors of fast mobile robots”.
In: Journal of Robotic Systems 11.1 (1994)](https://www.researchgate.net/publication/221787033_Reactive_Motion_Planning_for_Mobile_Robots)


## How it Works

The algorithm continuously computes a deformation vector to steer the robot away from intrusion.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radio_button_unchecked` 1. Define Zone</span> - **Create Bubble.**
  <br>Define a circular (or elliptical) protection zone around the robot with a nominal undeformed radius $R$.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radar` 2. Sense</span> - **Measure Intrusion.**
  <br>Using LaserScan data, compute the *deformed radius* $d_h(\alpha)$ for every angle $\alpha \in [0, 2\pi]$ around the robot.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`compress` 3. Compute Deformation</span> - **Calculate Metrics.**
  <br>
  * **Intrusion Intensity ($I_D$):** How much total "stuff" is inside the zone.
    $I_D = \frac{1}{2\pi} \int_{0}^{2\pi}\frac{R - d_h(\alpha)}{R} d\alpha$
  * **Deformation Angle ($\Theta_D$):** The primary direction of the intrusion.
    $\Theta_D = \frac{\int_{0}^{2\pi} (R - d_h(\alpha))\alpha d\alpha}{I_D}$

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`shortcut` 4. React</span> - **Control Law.**
  <br>The final control command minimizes $I_D$ (pushing away from the deformation) while trying to maintain the robot's original heading towards the goal.

## Supported Sensory Inputs

DVZ relies on dense 2D range data to compute the deformation integral.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radar` LaserScan</span>


## Configuration Parameters

DVZ balances two competing forces: **Path Following** (Geometric) vs. **Obstacle Repulsion** (Reactive).

```{list-table}
:widths: 10 10 10 70
:header-rows: 1

* - Name
  - Type
  - Default
  - Description

* - min_front_margin
  - `float`
  - `1.0`
  - Minimum front margin distance. Must be between `0.0` and `1e2`.
* - K_linear
  - `float`
  - `1.0`
  - Proportional gain for linear control. Must be between `0.1` and `10.0`.

* - K_angular
  - `float`
  - `1.0`
  - Proportional gain for angular control. Must be between `0.1` and `10.0`.

* - K_I
  - `float`
  - `5.0`
  - Proportional deformation gain. Must be between `0.1` and `10.0`.

* - side_margin_width_ratio
  - `float`
  - `1.0`
  - Width ratio between the deformation zone front and side (circle if 1.0). Must be between `1e-2` and `1e2`.

* - heading_gain
  - `float`
  - `0.7`
  - Heading gain of the internal pure follower control law. Must be between `0.0` and `1e2`.

* - cross_track_gain
  - `float`
   - `1.5`
  - Gain for cross-track error of the internal pure follower control law. Must be between `0.0` and `1e2`.


```

## usage Example

DVZ algorithm can be used in the [Controller](../../navigation/control.md) component by setting 'algorithm' property or component config parameter. The Controller will configure DVZ algorithm using the default values of all the previous configuration parameters. To configure custom values of the parameters, a YAML file is passed to the component.


```{code-block} python
:caption: dvz.py

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

# Set DVZ algorithm using the config class
controller_config = ControllerConfig(algorithm="DVZ")  # or LocalPlannersID.DVZ

# Set YAML config file
config_file = "my_config.yaml"

controller = Controller(component_name="my_controller",
                        config=controller_config,
                        config_file=config_file)

# algorithm can also be set using a property
controller.algorithm = ControllersID.DVZ      # or "DVZ"

```

```{code-block} yaml
:caption: my_config.yaml

my_controller:
  # Component config parameters
  loop_rate: 10.0
  control_time_step: 0.1
  ctrl_publish_type: 'Sequence'

  # Algorithm parameters under the algorithm name
  DVZ:
    cross_track_gain: 1.0
    heading_gain: 2.0
    K_angular: 1.0
    K_linear: 1.0
    min_front_margin: 1.0
    side_margin_width_ratio: 1.0
```
