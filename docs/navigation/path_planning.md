# Planner

**Global path planning and trajectory generation.**

The Planner component is responsible for finding an 'optimal' or 'suboptimal' path from a start to a goal location using complete map information.(i.e. the global or reference map).[^1]

It leverages the **[Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/)** backend to support various sampling-based algorithms (RRT*, PRM, etc.), capable of handling complex kinematic constraints.


[^1]: More on path planning algorithms can be seen in recent surveys on the topic in [Qin et al., 2023](https://www.mdpi.com/2504-446X/7/3/211) and [Karur et al., 2021](https://www.mdpi.com/2624-8921/3/3/27)


## Run Types

The Planner is flexible and can be executed in four different modes depending on your architecture:

```{list-table}
:widths: 20 80
* - **{material-regular}`schedule;1.2em;sd-text-primary` Timed**
  - **Periodic Re-planning.** Compute a new plan periodically (e.g., at 1Hz) from the robot's current location to the last received goal.

* - **{material-regular}`touch_app;1.2em;sd-text-primary` Event**
  - **Reactive Planning.** Trigger a new plan computation *only* when a new message is received on the `goal_point` topic.

* - **{material-regular}`dns;1.2em;sd-text-primary` Service**
  - **Request/Response.** Offers a standard ROS2 Service (`PlanPath`). Computes a single plan per request and returns it immediately.

* - **{material-regular}`hourglass_top;1.2em;sd-text-primary` Action Server**
  - **Long-Running Goal.** Offers a standard ROS2 Action. continuously computes and updates the plan until the goal is reached or canceled.
```


## Interface

### Inputs

The Planner subscribes to the following topics to understand the world state:

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">map</span>
  - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
  - 1
  - `/map`, `QoSConfig(durability=TRANSIENT_LOCAL)`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">goal_point</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.PointStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PointStamped.html)
  - 1
  - `/goal` (`PointStamped`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">location</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `/odom` (`Odometry`)
```

:::{note} 'goal_point' input is only used if the Planner is running as TIMED or EVENT Component. In the other two types, the goal point is provided in the service request or the action goal.
:::


### Outputs

The Planner publishes the resulting trajectory and motion state:

```{list-table}
:widths: 10 40 10 40
:header-rows: 1

* - Key Name
  - Allowed Types
  - Number
  - Default


* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">plan</span>
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - `/plan`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">reached_end</span>
  - `std_msgs.msg.Bool`
  - 1
  - `/reached_end`
```


## Algorithms

Kompass integrates OMPL to provide state-of-the-art geometric planners.

:::{card} Available Planners →
:link: ../integrations/ompl.html#available-algorithms-from-ompl
:link-type: url

Click here to view the full list of supported OMPL algorithms (RRT, PRM, EST, etc.)
:::

<br>

## Usage Example

```python
import numpy as np
from kompass.components import Planner, PlannerConfig
from kompass.config import ComponentRunType
from kompass.ros import Topic
from kompass.control import RobotType, RobotConfig, RobotGeometry, LinearCtrlLimits, AngularCtrlLimits


# 1. Configure your Robot Constraints
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]), # Radius, Height
    ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(max_vel=1.0, max_acc=2.0)
)

# 2. Configure the Planner
config = PlannerConfig(
    robot=my_robot,
    loop_rate=1.0  # Re-plan at 1Hz (if Timed)
)

# 3. Instantiate
planner = Planner(component_name="planner", config=config)

# 4. Set Run Type & Remap Topics
planner.run_type = ComponentRunType.EVENT

# Example: Remap input to listen to Rviz clicks
planner.inputs(goal_point=Topic(name="/clicked_point", msg_type="PoseStamped"))

```

## See Next

The Planner produces a complete path from the start point (robot location) to the final end point, then the robot can start following and locally modifying the planned path using the Controller components.

:::{button-link} control.html
:color: primary
:ref-type: url
:outline:
Discover the Controller Component →
:::
