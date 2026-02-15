# Pure Pursuit

**Geometric path tracking with reactive collision avoidance.**

Pure Pursuit is a fundamental path-tracking algorithm. It calculates the curvature required to move the robot from its current position to a specific "lookahead" point on the path, simulating how a human driver looks forward to steer a vehicle.

Kompass enhances the standard implementation (based on [Purdue SIGBOTS](https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit)) by adding an integrated **Simple Search Collision Avoidance** layer. This allows the robot to deviate locally from the path to avoid unexpected obstacles without needing a full replan.

## How it Works

The controller executes a four-step cycle:

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`ads_click` 1. Find Target</span> - **Locate Lookahead.**
  <br>Find the point on the path that is distance $L$ away from the robot. $L$ scales with speed ($L = k \cdot v$).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`gesture` 2. Steering</span> - **Compute Curvature.**
  <br>Calculate the arc required to reach that target point based on the robot's kinematic constraints.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`verified_user` 3. Safety</span> - **Collision Check.**
  <br>Project the robot's motion forward using the `prediction_horizon` to check for immediate collisions.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`alt_route` 4. Avoidance</span> - **Local Search.**
  <br>If the nominal arc is blocked, the controller searches through `max_search_candidates` to find a safe velocity offset that clears the obstacle while maintaining progress.


## Supported Sensory Inputs

To enable the collision avoidance layer, spatial data is required.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radar` LaserScan</span>
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`grain` PointCloud</span>
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`grid_on` OccupancyGrid</span>

*(Note: The controller can run in "blind" tracking mode without these inputs, but collision avoidance will be disabled.)*

## Configuration Parameters

```{list-table}
:widths: 15 10 10 65
:header-rows: 1
* - Name
  - Type
  - Default
  - Description

* - lookahead_gain_forward
  - `float`
  - `0.8`
  - Factor to scale lookahead distance by current velocity ($L = k \cdot v$).

* - prediction_horizon
  - `int`
  - `10`
  - Number of future steps used to check for potential collisions along the path.

* - path_search_step
  - `float`
  - `0.2`
  - Offset step used to search for alternative velocity commands when the nominal path is blocked.

* - max_search_candidates
  - `int`
  - `10`
  - Maximum number of search iterations to find a collision-free command.
```

## Usage Example:

Pure Pursuit controller can be used in the [Controller](../../navigation/control.md) component by setting 'algorithm' property or component config parameter. The Controller will configure Pure Pursuit algorithm using the default values of all the previous configuration parameters. The specific algorithms parameters can be configured using a config file or the algorithm's configuration class.


```{code-block} python
:caption: pure_pursuit.py

from kompass.components import Controller, ControllerConfig
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig
)
from kompass.control import ControllersID, PurePursuitConfig

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.OMNI,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([0.3, 0.3, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.2, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3)
)

# Initialize the controller
controller = Controller(component_name="my_controller")

# Set the all algorithms desired configuration
pure_pursuit_config = PurePursuitConfig(
        lookahead_gain_forward=0.5, prediction_horizon=8, max_search_candidates=20
    )

controller.algorithms_config = pure_pursuit_config

# NOTE: We can configure more than one algorithm to switch during runtime
# other_algorithm_config = ....
# controller.algorithms_config = [pure_pursuit_config, other_algorithm_config]

# Set the algorithm to Pure Pursuit
controller.algorithm = ControllersID.PURE_PURSUIT

```

## Performance & Results

The following tests demonstrate the controller's ability to track reference paths (**thin dark blue**) and avoid obstacles (**red x**).

### 1. Nominal Tracking
Performance on standard geometric paths (U-Turns and Circles) without interference.

::::{grid} 1 3 3 3
:gutter: 2

:::{grid-item-card} Ackermann
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_Ackermann_UTurn_traj.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_Ackermann_UTurn_traj.png" alt="Ackermann U-Turn" style="width: 100%; border-radius: 4px;">
</a>

**U-Turn**
:::

:::{grid-item-card} Differential
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_DiffDrive_Circle_traj.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_DiffDrive_Circle_traj.png" alt="Differential Circle" style="width: 100%; border-radius: 4px;">
</a>

**Circle**
:::

:::{grid-item-card} Omni
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_Omni_Circle_traj.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_Omni_Circle_traj.png" alt="Omni Circle" style="width: 100%; border-radius: 4px;">
</a>

**Circle**
:::
::::

### 2. Collision Avoidance
Scenarios where static obstacles are placed directly on the global path. The controller successfully identifies the blockage and deviates to finding a safe path around it.

::::{grid} 1 3 3 3
:gutter: 2

:::{grid-item-card} Ackermann
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_Ackermann_Straight_traj_with_obstacles.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_Ackermann_Straight_traj_with_obstacles.png" alt="Ackermann Obstacles" style="width: 100%; border-radius: 4px;">
</a>

**Straight + Obstacles**
:::

:::{grid-item-card} Differential
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_DiffDrive_UTurn_traj_with_obstacles.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_DiffDrive_UTurn_traj_with_obstacles.png" alt="Differential Obstacles" style="width: 100%; border-radius: 4px;">
</a>

**U-Turn + Obstacles**
:::

:::{grid-item-card} Omni
:class-card: sugar-card
<a href="../../_static/images/pure_pursuit_Omni_Straight_traj_with_obstacles.png" target="_blank">
  <img src="../../_static/images/pure_pursuit_Omni_Straight_traj_with_obstacles.png" alt="Omni Obstacles" style="width: 100%; border-radius: 4px;">
</a>

**Straight + Obstacles**
:::
::::

:::{admonition} Observations
:class: note
* **Convergence:** Smooth convergence to the reference path across all kinematic models.
* **Clearance:** The simple search algorithm successfully clears obstacle boundaries before returning to the path.
* **Stability:** No significant oscillation observed during avoidance maneuvers.
:::
