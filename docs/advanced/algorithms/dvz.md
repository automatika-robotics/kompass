# DVZ (Deformable Virtual Zone)

The DVZ is a reactive motion control method first introduced by R. Zapata in 1994 [^1]. It has been used since to model systems' maneuvers in both 2D and 3D spaces. The idea is to surround the system under study with a virtual zone, and any body entering that zone will cause a deformation. Then, the system can be driven in the direction minimizing this deformation or changing it in a desired way.

[^1]: [Zapata, R., Lépinay, P., and Thompson, P. “Reactive behaviors of fast mobile robots”.
In: Journal of Robotic Systems 11.1 (1994)](https://www.researchgate.net/publication/221787033_Reactive_Motion_Planning_for_Mobile_Robots)


## Supported Motion Models

- ACKERMANN
- DIFFERENTIAL_DRIVE
- OMNI

## Supported Sensory Inputs

- LaserScan


## Implementation

- Define a circular zone around the robot with known undeformed radius ($$R$$)
- Using LaserScan sensor data compute the deformed radius:
$ d_h(\alpha) : \alpha in [0, 2\pi]$

- Compute the zone deformation:

$ I_D = \frac{1}{2\pi} \int_{0}^{2\pi}\frac{R - d_h(\alpha)}{R} d\alpha  \in [0, 1]$

- Compute the zone deformation radius (if $I_D > 0$):

$ \Theta_D = \frac{\int_{0}^{2\pi} (R - d_h(\alpha))\alpha d\alpha}{I_D}   \in [0, 2\pi]$

- The control is calculated to minimize $I_D$ and stay close to the reference trajectory


## usage Example

```python
from kompass_core.control import DVZ
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
dvz = DVZ(
        robot=my_robot,
        ctrl_limits=robot_ctr_limits,
        control_time_step=control_time_step,
    )
```
