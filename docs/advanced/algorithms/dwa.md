# DWA (Dynamic Window Approach)

DWA is a famous local planning method developed since the 90s.[^1] DWA is a sampling-method consists of sampling a set of constant velocity trajectories within a window of admissible reachable velocities. This window of reachable velocities will change based on the current velocity and the acceleration limits, i.e. a Dynamic Window.

Implementation Requirements:

- Define the acceleration limits and the motion model of the robot
- Define a sampling step (or max number of samples) for the velocity commands
- Define an admissible trajectory criteria
- Simulate the robot movement based on the motion model for each velocity sample in the dynamic window and generate a set of admissible trajectories
- Define a cost or optimization criteria to select the best trajectory (minimal cost) among the admissible trajectories

## Supported Motion Models

- ACKERMANN
- DIFFERENTIAL_DRIVE
- OMNI

## Supported Sensory Inputs

- LaserScan
- PointCloud
- OccupancyGrid (comping soon)


## Trajectory Samples Generator:

### Admissible trajectory criteria:

A collision-free admissibility criteria in implemented within the trajectory samples generator using [FCL](../../integrations/fcl.md) to check the collision between the simulated robot state and the reference sensor input.

:::{figure-md} fig-acker
:class: myclass

<img src="../../_static/images/trajectories_ACKERMANN.png" alt="Trajectory Samples for ACKERMANN Robot" width="700px">

Generated Trajectory Samples for an Ackermann Robot
:::

:::{note} To maintain a natural movement For both Differential drive and Omni robots; rotation in place and linear movement at the same time are not supported. The trajectory sampler implements rotate-then-move policy.
:::


:::{figure-md} fig-diff
:class: myclass

<img src="../../_static/images/trajectories_DIFFERENTIAL_DRIVE.png" alt="Trajectory Samples for Differential Drive Robot" width="700px">

Generated Trajectory Samples for a Differential Drive Robot
:::

:::{figure-md} fig-omni
:class: myclass

<img src="../../_static/images//trajectories_OMNI.png" alt="Trajectory Samples for OMNI Robot" width="700px">

Generated Trajectory Samples for an Omni motion Robot
:::

## Trajectory Selection

The cost of each admissible sample is computed using [Cost Evaluator](cost_eval.md) and the sample with the lowest cost is selected for navigation.

## usage Example

```python
from kompass_core.control import DWAConfig, DWA
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
config = DWAConfig(
        max_linear_samples=20,
        max_angular_samples=20,
        octree_resolution=0.1,
        prediction_horizon=1.0,
        control_horizon=0.2,
        control_time_step=0.1,
    )

controller = DWA(robot=my_robot, ctrl_limits=robot_ctr_limits, config=config)
```


[^1]: [Dieter Foxy, Wolf Burgardy and Sebastian Thrun. The Dynamic Window Approach to Collision Avoidance. IEEE Robotics & Automation Magazine ( Volume: 4, Issue: 1, March 1997)](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)
