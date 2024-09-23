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

## usage Example

```python
from kompass_core.control import StanleyConfig, Stanley
from kompass_core.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    Robot,
    RobotCtrlLimits,
    RobotGeometry,
    RobotType,
)

# Configure the robot
my_robot = Robot(
        robot_type=RobotType.ACKERMANN,
        geometry_type=RobotGeometry.Type.CYLINDER,
        geometry_params=np.array([0.1, 0.4]),
    )

# Configure the control limits (used to compute the dynamic window)
robot_ctr_limits = RobotCtrlLimits(
    vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=5.0, max_decel=10.0),
    omega_limits=AngularCtrlLimits(
        max_vel=2.0, max_acc=3.0, max_decel=3.0, max_steer=np.pi
    ),
)

# Configure the controller
 config = StanleyConfig(cross_track_gain=3.0, heading_gain=1.0)

stanley = Stanley(
    robot=my_robot,
    ctrl_limits=robot_ctr_limits,
    config=config,
    control_time_step=control_time_step,
)
```

[^1]:  [Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and Sebastian Thrun. "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing." American Control Conference. 2007, pp. 2296–2301](https://ieeexplore.ieee.org/document/4282788)
