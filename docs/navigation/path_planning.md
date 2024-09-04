# Planner

Path planning consists of finding an 'optimal' or 'suboptimal' path from a start to a goal location given partial (local planning) or complete space information (global planning).
In a 2D navigation system, path planning refers to the **global path planning** which seeks an optimal path given largely complete static environmental information that is perfectly known to the robot (i.e. the global or reference map).[^1]

[^1]: More on path planning algorithms can be seen in recent surveys on the topic in [Qin et al., 2023](https://www.mdpi.com/2504-446X/7/3/211) and [Karur et al., 2021](https://www.mdpi.com/2624-8921/3/3/27)

Path planning algorithms produce a complete path from the start point (robot location) to the final end point, then the robot can start following and locally modifying the planned path (see [Control](control.md)).


[Planner](../apidocs/Kompass/Kompass.components.planner.md) Component is in charge of global path planning in Kompass. Planner uses the [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/) plugins to perform the path planning (more on [OMPL integration](../integrations/ompl.md) with Kompass)


## Available Run Types
Planner can be used with all four available RunTypes:

```{list-table}
:widths: 10 80
* - **Timed**
  - Compute a new plan periodically from current location (last message received on location input Topic) to the goal location (last message received on goal_point input Topic)

* - **Event**
  - Compute a new plan from current location on every new message received on [**goal_point**](#planner-inputs) input Topic

* - **Server**
  - Offers a PlanPath ROS service and computes a new plan on every service request

* - **ActionServer**
  - Offers a PlanPath ROS action and continuously computes a plan once an action request is received until goal point is reached
```

## Planner Inputs
Planner requires configuring three inputs:

```{list-table}
:widths: 10 30 15 20 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Number of Topics
  - Default Value

* - **map_layer**
  - Global map used for planning
  - `OccupancyGrid`
  - 1
  - `defualt_qos=QoSConfig(durability=qos.DurabilityPolicy.TRANSIENT_LOCAL)
  Topic(name="/map", msg_type="OccupancyGrid" qos_profile=defualt_qos)`

* - **location**
  - Robot current location
  - `Odometry, PoseWithCovariance, Pose`
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`

* - **goal_point**
  - Navigation goal point on the map
  - `Odometry, PoseStamped, PointStamped`
  - 1
  - `Topic(name="/goal", msg_type="PointStamped")`
```

:::{note} 'goal_point' input is only used if the Planner is running as TIMED or EVENT Component. In the other two types, the goal point is provided in the service request or the action goal.
:::


## Planner Outputs
Planner offers two outputs:

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Default Value

* - **plan**
  - Path to reach the goal point from start location
  - `Path`
  - `Topic(name="/plan", msg_type="Path")`

* - **reached_end**
  - Flag indicating that the current planning target is reached
  - `Bool`
  - `Topic(name="/reached_end", msg_type="Bool")`
```

## Available Algorithms:

OMPL geometric planners (see [available OMPL integrations](../integrations/ompl.md/#available-algorithms-from-ompl) for more details)

## Usage Example
```python
    from kompass.components import Planner, PlannerConfig
    from kompass.config import ComponentRunType
    from Navigation.models import RobotType, Robot, RobotGeometry, LinearCtrlLimits, AngularCtrlLimits
    import numpy as np

    # Configure your robot
    my_robot = RobotConfig(
            model_type=RobotType.DIFFERENTIAL_DRIVE,
            geometry_type=RobotGeometry.Type.CYLINDER,
            geometry_params=np.array([0.1, 0.3]),
            ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5),
            ctrl_omega_limits=AngularCtrlLimits(
                max_vel=1.0, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
            ),
        )

    # Setup the planner config
    config = PlannerConfig(
        robot=my_robot,
        loop_rate=1.0 # 1Hz
    )

    planner = Planner(node_name="planner", config=config)

    planner.run_type = ComponentRunType.EVENT   # Can also pass a string "Event"

    # Add rviz clicked_point as input topic
    goal_topic = Topic(name="/clicked_point", msg_type="PoseStamped")
    planner.inputs(goal_point=goal_topic)
```