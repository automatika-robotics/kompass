# DWA (Dynamic Window Approach)

DWA is a popular local planning method developed since the 90s.[^1] DWA is a sampling-method that consists of sampling a set of constant velocity trajectories within a window of admissible reachable velocities. This window of reachable velocities will change based on the current velocity and the acceleration limits, i.e. a Dynamic Window.

At each step, the reachable velocity range is computed based on the acceleration limits and the motion model of the robot. Then a set of constant velocity trajectories is [sampled](#trajectory-samples-generator) within the range after checking its [admissibility](#admissible-trajectory-criteria). Finally, the best trajectory is selected using the trajectory [cost evaluation](#trajectory-selection) functions.


## Supported Sensory Inputs

- LaserScan
- PointCloud
- OccupancyGrid


## Parameters and Default Values

```{list-table}
:widths: 10 10 10 70
:header-rows: 1
* - Name
  - Type
  - Default
  - Description

* - control_time_step
  - `float`
  - `0.1`
  - Time interval between control actions (sec). Must be between `1e-4` and `1e6`.

* - prediction_horizon
  - `float`
  - `1.0`
  - Duration over which predictions are made (sec). Must be between `1e-4` and `1e6`.

* - control_horizon
  - `float`
  - `0.2`
  - Duration over which control actions are planned (sec). Must be between `1e-4` and `1e6`.

* - max_linear_samples
  - `int`
  - `20`
  - Maximum number of linear control samples. Must be between `1` and `1e3`.

* - max_angular_samples
  - `int`
  - `20`
  - Maximum number of angular control samples. Must be between `1` and `1e3`.

* - sensor_position_to_robot
  - `List[float]`
  - `[0.0, 0.0, 0.0]`
  - Position of the sensor relative to the robot in 3D space (x, y, z) coordinates.

* - sensor_rotation_to_robot
  - `List[float]`
  - `[0.0, 0.0, 0.0, 1.0]`
  - Orientation of the sensor relative to the robot as a quaternion (x, y, z, w).

* - octree_resolution
  - `float`
  - `0.1`
  - Resolution of the Octree used for collision checking. Must be between `1e-9` and `1e3`.

* - costs_weights
  - `TrajectoryCostsWeights`
  - see [defaults](cost_eval.md/#costs-weights)
  - Weights for trajectory cost evaluation.

* - max_num_threads
  - `int`
  - `1`
  - Maximum number of threads used when running the controller. Must be between `1` and `1e2`.

```

```{note}
All the previous parameters can be configured when using DWA algorithm using a YAML config file (as shown in the usage example)
```

## Usage Example:

DWA algorithm can be used in the [Controller](../../navigation/control.md) component by setting 'algorithm' property or component config parameter. The Controller will configure DWA algorithm using the default values of all the previous configuration parameters. To configure custom values of the parameters, a YAML file is passed to the component.



```{code-block} python
:caption: dwa.py

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

# Set DWA algorithm using the config class
controller_config = ControllerConfig(algorithm="DWA")

# Set YAML config file
config_file = "my_config.yaml"

controller = Controller(component_name="my_controller",
                        config=controller_config,
                        config_file=config_file)

# algorithm can also be set using a property
controller.algorithm = ControllersID.DWA      # or "DWA"

```

```{code-block} yaml
:caption: my_config.yaml

my_controller:
  # Component config parameters
  loop_rate: 10.0
  control_time_step: 0.1
  prediction_horizon: 4.0
  ctrl_publish_type: 'Array'

  # Algorithm parameters under the algorithm name
  DWA:
    control_horizon: 0.6
    octree_resolution: 0.1
    max_linear_samples: 20
    max_angular_samples: 20
    max_num_threads: 3
    costs_weights:
      goal_distance_weight: 1.0
      reference_path_distance_weight: 1.5
      obstacles_distance_weight: 2.0
      smoothness_weight: 1.0
      jerk_weight: 0.0
```

## Trajectory Samples Generator:

Trajectory samples are generated using a constant velocity generator for each velocity value within the reachable range to generate the configured maximum number of samples (see `max_linear_samples` and `max_angular_samples` in the [config parameters](#parameters-and-default-values)).

:::{tip} The effective total number of generated samples will depend on the motion model of the robot, as non-holonomic robots will only sample along the forward and angular velocity, while holonomic robots (Omni) will also sample lateral velocities.
:::

:::{note} To maintain a natural movement For both Differential drive and Omni robots; rotation in place and linear movement at the same time are not supported. The trajectory sampler implements rotate-then-move policy.
:::

:::{figure-md} fig-acker
:class: myclass

<img src="../../_static/images/trajectories_ACKERMANN.png" alt="Trajectory Samples for ACKERMANN Robot" width="700px">

Generated Trajectory Samples for an Ackermann Robot
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

### Admissible trajectory criteria:

A collision-free admissibility criteria is implemented within the trajectory samples generator using [FCL](../../integrations/fcl.md) to check the collision between the simulated robot state and the reference sensor input.

## Trajectory Selection

The cost of each admissible sample is computed using [Cost Evaluator](cost_eval.md) and the sample with the lowest cost is selected for navigation. After calculating the cost using the cost evaluation function, the total cost is computed as the sum of all the costs weighed by each cost respective weight.

:::{tip} Set the costs weights directly in the DWA config
:::


[^1]: [Dieter Foxy, Wolf Burgardy and Sebastian Thrun. The Dynamic Window Approach to Collision Avoidance. IEEE Robotics & Automation Magazine ( Volume: 4, Issue: 1, March 1997)](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)
