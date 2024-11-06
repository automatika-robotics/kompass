# Using Events/Actions in your application

Events/Actions are a powerful tool to make your robot application adaptive to the changing working conditions (both internal and external to the robot), or to extend the operational scope of your application by adapting it to new spaces/usecases.

In this tutorial we will extend the Turtlebot3 quick start [recipe](../quick_start.md) by adding a few events/actions to the system.

## First Event/Action Pair

First we define two events, the first is associated with the Controller internal algorithm failing to calculate a new control command by accessing the value of the Health Status broadcasted by the Controller on its own status topic. The second event is triggered when an emergency stop is detected by the DriveManager:

```python
from kompass import event

event_controller_fail = event.OnEqual(
    "controller_fail",
    Topic(name="controller_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    "status",
)

event_emergency_stop = event.OnEqual(
        "emergency_stop",
        Topic(name="emergency_stop", msg_type="Bool"),
        True,
        "data",
    )
```

We will link both events with an Action provided by the `DriveManager` called `move_to_unblock`. This is to address two a problem that can occur during navigation when the robot is too close to obstacles or an obstacle is already touching the chassis which will lead many control algorithms to fail. In such cases `move_to_unblock` will help the robot to move a little and escape the clutter or collision area.

```python
from kompass.actions import Action

unblock_action = Action(method=driver.move_to_unblock)
```

We can also define an event for the Planner algorithm failing to produce a new map. We can associate it with the same action for the case where the reference map of the space used by the robot is not accurate. If the map is not accurate the robot can be at a starting point that is marked occupied on the map where it's in fact free. In this case the the Planner will fail to solve the navigation problem as an invalid start state is passed to the algorithm. In this case executing the `move_to_unblock` action can help escape the invalid space.


```python
event_planner_algo_fail = event.OnEqual(
    "planner_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    ("status"),
)
```

```{tip}
If you are using Kompass with the [Costmap 2D](wiki.ros.org/costmap_2d) node: You can associate another Action to the Planner algorithm level fail event which is a ROS2 service call to the node `clear_costmap` service. (Keep reading to see how similar actions are executed!)
```

```{note}
In the quick start [recipe](../quick_start.md) we added a generic on_fail policy to restart the failed component. This policy will be applied whenever any type of failure is occurs, including the previously used algorithm failure. We can choose to keep this policy, meaning that while the driver is executing the `move_to_unblock` action, then the planner or controller will get restarted.
```

## Tweak the system using events

In our [_quick start_](../quick_start.md) example we used the Planner with a `Timed` run type and used RVIZ clicked point as the goal point input. In this configuration, once a point is clicked on RVIZ (a message is published on the topic), the Planner will keep producing a new plan each loop step from the current position to the clicked point. To track the mission during execution and end the mission once the point is reached we can run the Planner as an `ActionServer`. In this case the Planner produces a new plan on an incoming action request and will no longer take goals directly from RVIZ topic.

We will use events here to run the Planner as an `ActionServer` and accept the goal directly from RVIZ to get the best of both worlds.

First, we define an event that is triggered on any clicked point on RVIZ:

```python
from kompass import event
from kompass.topic import Topic

# On any clicked point
event_clicked_point = event.OnGreater(
    "rviz_goal",
    Topic(name="/clicked_point", msg_type="PointStamped"),
    0,
    ["header", "stamp", "sec"],
)
```

Then we will add an Action that sends a goal to the Planner's ActionServer:

```python
from kompass.actions import ComponentActions
from kompass_interfaces.action import PlanPath

# Define an Action to send a goal to the planner ActionServer
send_goal: Action = ComponentActions.send_action_goal(
    action_name="/planner/plan_path",
    action_type=PlanPath,
    action_request_msg=PlanPath.Goal(),
)
```
The only thing left is to parse the clicked point message published on the event topic into the `send_goal` action goal. For this we will write a small method that executes tha desired parsing and pass it to the action using the `event_parser` method available in the Action class:

```python
from kompass_interfaces.msg import PathTrackingError
from geomerty_msgs.msg import PointStamped

# Define a method to parse a message of type PointStamped to the planner PlanPath Goal
def goal_point_parser(*, msg: PointStamped, **_):
    action_request = PlanPath.Goal()
    goal = Pose()
    goal.position.x = msg.point.x
    goal.position.y = msg.point.y
    action_request.goal = goal
    end_tolerance = PathTrackingError()
    end_tolerance.orientation_error = 0.2
    end_tolerance.lateral_distance_error = 0.05
    action_request.end_tolerance = end_tolerance
    return action_request

# Adds the parser method as an Event parser of the send_goal action
send_goal.event_parser(goal_point_parser, output_mapping="action_request_msg")
```

Now we can write a dictionary to link all our events with their respective actions:

```python
from kompass.actions import ComponentActions, LogInfo

# Define Events/Actions dictionary
events_actions = {
    event_clicked_point: [LogInfo(msg="Got a new goal point from RVIZ"), send_goal],
    event_emergency_stop: [
        ComponentActions.restart(component=planner),
        unblock_action,
    ],
    event_controller_fail: unblock_action,
}
```

## Events/Actions to interact with external systems

We can also use events/actions to communicate with systems or ROS2 nodes external to Kompass. Here for example, we will add another event for a system-level failure in the planner and an Action to send a service request to the Map Server node to load our reference map from the file:

```python
import os
from kompass.actions import ComponentActions
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import (
    get_package_share_directory,
)

package_dir = get_package_share_directory(package_name="kompass")

load_map_req = LoadMap()
load_map_req.map_url = os.path.join(
        package_dir, "maps", "turtlebot3_webots.yaml"
    )

action_load_map = ComponentActions.send_srv_request(
    srv_name="/map_server/load_map",
    srv_type=LoadMap,
    srv_request_msg=load_map_req,
)

event_planner_system_fail = event.OnEqual(
    "planner_system_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_SYSTEM_LEVEL,
    "status",
)
```

```{tip}
`SYSTEM_LEVEL_FAILURE` occurs in a component when an external error occurs such as an unavailable input.
```

## Complete Recipe

Et voila! We extended the robustness and adaptivity of our system by using events/actions and without modifying any of the used components and without introducing any new components.

Finally the full recipe with the added events/actions will look as follows:

```{code-block} python
:caption: turtlebot3 test
:linenos:

import numpy as np
import os
from nav2_msgs.srv import LoadMap

from kompass_core.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)
from kompass_core.control import LocalPlannersID

from sugar.msg import ComponentStatus
from kompass_interfaces.action import PlanPath
from kompass_interfaces.msg import PathTrackingError
from geometry_msgs.msg import Pose, PointStamped

from kompass import event
from kompass.actions import Action

from kompass.components import (
    Controller,
    DriveManager,
    Planner,
    PlannerConfig,
    LocalMapper,
)
from kompass.actions import ComponentActions, LogInfo
from kompass.config import RobotConfig
from kompass.launcher import Launcher
from kompass.topic import Topic

from ament_index_python.packages import get_package_share_directory


package_dir = get_package_share_directory(package_name="kompass")
config_file = os.path.join(package_dir, "params", "turtlebot3.yaml")

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.2, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3)
)

config = PlannerConfig(robot=my_robot, loop_rate=1.0)
planner = Planner(component_name="planner", config=config, config_file=config_file)

controller = Controller(component_name="controller")
driver = DriveManager(component_name="drive_manager")
mapper = LocalMapper(component_name="mapper")

# Configure Controller options
controller.algorithm = LocalPlannersID.STANLEY
controller.direct_sensor = False

planner.run_type = "ActionServer"

driver.on_fail(action=Action(driver.restart))

# DEFINE EVENTS
event_emergency_stop = event.OnEqual(
    "emergency_stop",
    Topic(name="emergency_stop", msg_type="Bool"),
    True,
    "data",
)
event_controller_fail = event.OnEqual(
    "controller_fail",
    Topic(name="controller_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    "status",
)

event_planner_system_fail = event.OnEqual(
    "planner_system_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_SYSTEM_LEVEL,
    "status",
)

# Unblock action
unblock_action = Action(method=driver.move_to_unblock)

# On any clicked point
event_clicked_point = event.OnGreater(
    "rviz_goal",
    Topic(name="/clicked_point", msg_type="PointStamped"),
    0,
    ["header", "stamp", "sec"],
)

# Define an Action to send a goal to the planner ActionServer
send_goal: Action = ComponentActions.send_action_goal(
    action_name="/planner/plan_path",
    action_type=PlanPath,
    action_request_msg=PlanPath.Goal(),
)

# Define a method to parse a message of type PointStamped to the planner PlanPath Goal
def goal_point_parser(*, msg: PointStamped, **_):
    action_request = PlanPath.Goal()
    goal = Pose()
    goal.position.x = msg.point.x
    goal.position.y = msg.point.y
    action_request.goal = goal
    end_tolerance = PathTrackingError()
    end_tolerance.orientation_error = 0.2
    end_tolerance.lateral_distance_error = 0.05
    action_request.end_tolerance = end_tolerance
    return action_request

# Adds the parser method as an Event parser of the send_goal action
send_goal.event_parser(goal_point_parser, output_mapping="action_request_msg")

# Load map action
load_map_req = LoadMap()
load_map_req.map_url = os.path.join(package_dir, "maps", "turtlebot3_webots.yaml")

action_load_map = ComponentActions.send_srv_request(
    srv_name="/map_server/load_map",
    srv_type=LoadMap,
    srv_request_msg=load_map_req)

# Define Events/Actions dictionary
events_actions = {
    event_clicked_point: [LogInfo(msg="Got new goal point"), send_goal],
    event_emergency_stop: [
        ComponentActions.restart(component=planner),
        unblock_action,
    ],
    event_controller_fail: unblock_action,
    event_planner_system_fail: action_load_map
}

# Setup the launcher
launcher = Launcher(config_file=config_file)

# Add Kompass components
launcher.kompass(
    components=[planner, controller, mapper, driver],
    events_actions=events_actions,
    activate_all_components_on_start=True,
    multi_processing=True)

# Get odom from localizer filtered odom for all components
odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
launcher.inputs(location=odom_topic)

# Set the robot config for all components
launcher.robot = my_robot

launcher.bringup(introspect=True)
```
