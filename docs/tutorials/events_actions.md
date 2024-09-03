# Using Events/Actions in your application

Events/Actions are a powerful tool to make your robot application adaptive to the changing working conditions (both internal and external to the robot), or to extend the operational scope of your application by adapting it to new spaces/usecases.

In this tutorial we will extend the Turtlebot3 getting started [recipe](../quick_start.md) by adding a few events/actions to the system.

First we define an event associated with the Controller internal algorithm failing to calculate a new control command by accessing the value of the Health Status broadcasted by the Planner on its own status topic.

```python
from kompass import event

event_controller_fail = event.OnEqual(
    "controller_fail",
    Topic(name="controller_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    ("status"),
)
```

We will link this event with an Action provided by the `DriveManager` called `move_to_unblock`. This is to address two a problem that can occur during navigation when the robot is too close to obstacles or an obstacle is already touching the chassis which will lead many control algorithms to fail. In such cases `move_to_unblock` will help the robot to move a little and escape the clutter or collision area.

```python
from kompass.actions import Action

unblock_action = Action(method=driver.move_to_unblock)
```
Similarly we will define an event for the Planner algorithm failing to produce a new map. We can associate it with the same action for the case where the reference map of the space used by the robot is not accurate. If the map is not accurate the robot can be at a starting point that is marked occupied on the map where it's in fact free. In this case the the Planner will fail to solve the navigation problem as an invalid start state is passed to the algorithm. In this case executing the `move_to_unblock` action can help escape the invalid space.


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

Next, since the previous two events are executed directly by the `DriveManager` we can pass them directly to the component. Before passing the events to the `DriveManager` component, we will add a third event that can use the same unblocking action, which is the emergency stop event:

```python

event_emergency_stop = event.OnEqual(
    "emergency_stop",
    Topic(name="emergency_stop", msg_type="Bool"),
    True,
    ("data"),
)
driver.events_actions = {
    event_control_fail: unblock_action,
    event_planner_algo_fail: unblock_action,
    event_emergency_stop: unblock_action,
}
```

```{note}
In the getting started [recipe](../quick_start.md) we added a generic on_fail policy to restart the failed component. This policy will be applied whenever any type of failure is occurs, including the previously used algorithm failure. We can choose to keep this policy, meaning that while the driver is executing the `move_to_unblock` action, then the planner or controller will get restarted.
```
Next we will add another 'system level' event that we will pass directly to the launcher. This event is related to a `SYSTEM_LEVEL_FAILURE` in the Planner. In this case we will add an Action to send a service request to the Map Server node to load our reference map from the file:

```python
import os
from kompass.actions import ComponentActions
from nav2_msgs.srv import LoadMap

current_working_dir = os.path.dirname(os.path.abspath(__file__))

load_map_req = LoadMap()
load_map_req.map_url = os.path.join(
        current_dir, "../resources/maps", "turtlebot3_webots.yaml"
    )

event_planner_system_fail = event.OnEqual(
    "planner_system_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_SYSTEM_LEVEL,
    ("status"),
)

launcher_events_actions = {
    event_planner_system_fail: ComponentActions.send_srv_request(
    srv_name="/map_server/load_map",
    srv_type=LoadMap,
    srv_request_msg=load_map_req,
)
}
```

```{tip}
`SYSTEM_LEVEL_FAILURE` occurs in a component when an external error occurs such as an unavailable input.
```

Et voila! We extended the robustness and adaptivity of our system by using events/actions and without modifying any of the used components and without introducing any new components.

Finally the full recipe with the added events/actions will look as follows:


```{code-block} python
:caption: turtlebot3 test
:linenos:

import numpy as np
from Navigation.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)
from kompass.components import (
    Controller,
    Planner,
    PlannerConfig,
)
from kompass.config import RobotConfig, RobotFrames
from kompass.launcher import Launcher
from kompass.topic import Topic

from kompass import event
from kompass.actions import ComponentActions, Action
from nav2_msgs.srv import LoadMap

# Configure the robot
robot_config = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=1.0, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),
)
# Set the robot frames
robot_frames = RobotFrames(
    robot_base='base_link',
    odom='odom',
    world='map',
    scan='LDS-01'
)

# Setup components with default config, inputs and outputs
planner_config = PlannerConfig(loop_rate=1.0)       # 1 Hz
planner = Planner(node_name="planner", config=planner_config)

# Set Planner goal input to Rviz Clicked point
goal_topic = Topic(name="/clicked_point", msg_type="PointStamped")
planner.inputs(goal_point=goal_topic)

# Get a default controller component
controller = Controller(node_name="controller")

# Set DriveManager velocity output to the turtlebot3 twist command
driver = DriveManager(node_name="drive_manager")
driver.outputs(command=Topic(name="cmd_vel", msg_type="Twist"))

# Define Controller algorithm fail event
event_controller_fail = event.OnEqual(
    "controller_fail",
    Topic(name="controller_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    ("status"),
)

# Define Planner algorithm fail event
event_planner_algo_fail = event.OnEqual(
    "planner_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
    ("status"),
)

# Define emergency stop event
event_emergency_stop = event.OnEqual(
    "emergency_stop",
    Topic(name="emergency_stop", msg_type="Bool"),
    True,
    ("data"),
)

# Define unblocking action
unblock_action = Action(method=driver.move_to_unblock)

# Add events to driver to execute the unblocking action
driver.events_actions = {
    event_control_fail: unblock_action,
    event_planner_algo_fail: unblock_action,
    event_emergency_stop: unblock_action,
}

# Setup service request to the 'Map Server' node for loading the reference map
current_working_dir = os.path.dirname(os.path.abspath(__file__))
load_map_req = LoadMap()
load_map_req.map_url = os.path.join(
        current_dir, "../resources/maps", "turtlebot3_webots.yaml"
    )

# Define event on Planner system fail
event_planner_system_fail = event.OnEqual(
    "planner_system_fail",
    Topic(name="planner_status", msg_type="ComponentStatus"),
    ComponentStatus.STATUS_FAILURE_SYSTEM_LEVEL,
    ("status"),
)

# Set the associated launcher events/actions
launcher_events_actions = {
    event_planner_system_fail: ComponentActions.send_srv_request(
    srv_name="/map_server/load_map",
    srv_type=LoadMap,
    srv_request_msg=load_map_req,
)
}

# Setup the launcher
launcher = Launcher(
    components=[planner, controller, driver],
    events_actions=launcher_events_actions,
    activate_all_components_on_start=True,
    multi_processing=True,
)

# Set the robot
launcher.robot = robot_config

# Set the frames
launcher.frames = frames_config

# Fallback Policy: If any component fails -> restart it with unlimited retries
launcher.on_fail(action_name="restart")

# After all configuration is done bringup the stack
launcher.bringup(ros_log_level="info")
```
