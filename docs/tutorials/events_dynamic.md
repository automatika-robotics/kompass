# Context-Aware Actions

In previous tutorials, our actions were **Static**, i.e. the action argument were pre-defined in the recipe. For example, in the [External Reflexes Tutorial](./events_external_reflexes.md) we defined an event when a "Person is Detected" $\rightarrow$ execute action to trigger the vision follower action server with a pre-defined request message with the target 'person'. But what if the action depends on **what** object is detected?

Real-world autonomy requires context aware actions, where the action arguments can be fetched from the system or environment at the time of execution

Instead of writing separate hard-coded actions for every possibility, Kompass allows you to use **Dynamic Data Injection** from Topics directly in your Actions. In the [Point Navigation](point_navigation.md) tutorial, we manually set goals using Rviz `clicked_point`. In this tutorial, we will upgrade the Turtlebot3 to understand **Semantic Commands**.



## Semantic Navigation using a Context-Aware Action

We will build a system where you can publish a location name (String) to a topic, and the robot will automatically look up the coordinates and navigate there.

## The Setup

**Prerequisites:** Completed [Point Navigation](https://www.google.com/search?q=point_navigation.md).

We will use the exact same set of component as in the [Point Navigation](point_navigation.md) tutorial, plus a simulated "Command" topic.

```python
from kompass.ros import Topic

# Define the Command Source
# Simulates a voice command or fleet management instruction
# Examples: "kitchen", "reception", "station_A"
command_topic = Topic(name="/user_command", msg_type="String")

```

## The Context-Aware Logic

1. We need a function that translates a *Place Name* (String) into a *Goal Pose* (Coordinates).

```python
from geomerty_msgs.msg import PoseStamped
import subprocess

# A simple "Map" of our environment
# In a real app, this can come from a database or a "semantic memory"
WAYPOINTS = {
    "kitchen":    {"x": 2.0, "y": 0.5},
    "reception":  {"x": 0.0, "y": 0.0},
    "station_a":  {"x": -1.5, "y": 2.0}
}

def navigate_to_location(location_name: str):
    """
    Looks up the location coordinates and sends them to the planner.
    """
    key = location_name.strip().lower()
    if key not in WAYPOINTS:
        print(f"Unknown location: {key}")
        return

    coords = WAYPOINTS[key]
    x = coords["x"]
    y = coords["y"]

    # Construct the Bash Command
    # We publish to /clicked_point (PointStamped) which the Planner listens to
    # Corrected f-string with proper escaping
    topic_cmd = (
        f"ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "
        f"'{{header: {{frame_id: \"map\"}}, point: {{x: {x}, y: {y}, z: 0.0}}}}'"
    )
    print(f"Executing: {topic_cmd}")
    subprocess.run(topic_cmd, shell=True)

```

2. Define an Event that triggers whenever a new command arrives:

```python
# Trigger on ANY new message on the command topic, if the local mapper is healthy
event_command_received = Event(
    event_condition=(command_topic & (mapper.status_topic.msg.status == ComponentStatus.STATUS_HEALTHY)),
)

```

3. Define the action:

```python

action_process_command = Action(
    method=navigate_to_location,
    # DYNAMIC INJECTION:
    # We pass 'command_topic.msg.data' as the argument.
    # Sugarcoat will fetch the actual string ("kitchen") when the event fires.
    args=(command_topic.msg.data,)
)

```

## Complete Recipe

Launch this script, then publish a string to `/user_command` (e.g., `ros2 topic pub /user_command std_msgs/String "data: kitchen" --once`) to see the robot move!

```{code-block} python
:caption: turtlebot3_with_fallbacks.py
:linenos:

import numpy as np
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)
from kompass.config import RobotConfig, RobotFrames
from kompass.components import (
    Controller,
    DriveManager,
    Planner,
    PlannerConfig,
)
from kompass.control import ControllersID
from kompass.ros import Topic, Launcher, Action

# Import the ComponentStatus message to access the status values
from automatika_ros_sugar.msg import ComponentStatus

# Import the reconfiguration action
from kompass.actions import update_parameter

from geomerty_msgs.msg import PoseStamped
import subprocess

# A simple "Map" of our environment
# In a real app, this can come from a database or a "semantic memory"
WAYPOINTS = {
    "kitchen":    {"x": 2.0, "y": 0.5},
    "reception":  {"x": 0.0, "y": 0.0},
    "station_a":  {"x": -1.5, "y": 2.0}
}

def navigate_to_location(location_name: str):
    """
    Looks up the location coordinates and sends them to the planner.
    """
    key = location_name.strip().lower()
    if key not in WAYPOINTS:
        print(f"Unknown location: {key}")
        return

    coords = WAYPOINTS[key]
    x = coords["x"]
    y = coords["y"]

    # Construct the Bash Command
    # We publish to /clicked_point (PointStamped) which the Planner listens to
    topic_cmd = (
        f"ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped "
        f"'{{header: {{frame_id: \"map\"}}, point: {{x: {x}, y: {y}, z: 0.0}}}}'"
    )
    print(f"Executing: {topic_cmd}")
    subprocess.run(topic_cmd, shell=True)

# Define the Command Source
# Simulates a voice command or fleet management instruction
# Examples: "kitchen", "reception", "station_A"
command_topic = Topic(name="/user_command", msg_type="String")

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.2, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3)
)

# Set the robot frames
robot_frames = RobotFrames(
    robot_base='base_link',
    odom='odom',
    world='map',
    scan='LDS-01')

# Setup components with default config, inputs and outputs
planner_config = PlannerConfig(loop_rate=1.0)       # 1 Hz
planner = Planner(component_name="planner", config=planner_config)

# Set Planner goal input to Rviz Clicked point
goal_topic = Topic(name="/clicked_point", msg_type="PointStamped")
planner.inputs(goal_point=goal_topic)

# Get a default controller component
controller = Controller(component_name="controller")

# Configure Controller to use local map instead of direct sensor information
controller.direct_sensor = False

# Select the primary control algorithm
controller.algorithm = ControllersID.DWA

# Define the action
switch_algorithm_action =  Action(method=controller.set_algorithm, args=(ControllersID.PURE_PURSUIT,))

# Set the Controller 'algorithm-level' failure fallback
controller.on_algorithm_fail(action=[Action(controller.restart), switch_algorithm_action], max_retries=1)

# Get the default DriveManager
driver = DriveManager(component_name="drive_manager")

# Publish Twist or TwistStamped from the DriveManager based on the distribution
if "ROS_DISTRO" in os.environ and (
    os.environ["ROS_DISTRO"] in ["rolling", "jazzy", "kilted"]
):
    cmd_msg_type : str = "TwistStamped"
else:
    cmd_msg_type = "Twist"

driver.outputs(robot_command=Topic(name="/cmd_vel", msg_type=cmd_msg_type))

# Set the DriveManager System-Level failure Fallback
# Not setting max_retires -> will attempt restarting infinity
driver.on_system_fail(Action(driver.restart))

# Get a default Local Mapper component
mapper = LocalMapper(component_name="mapper")

# Define Events
# 1. Controller Algorithm Failure
# We set a `on_change` True to trigger the event only when the status changes to algorithm failure the first time
# i.e. the associated action will not keep triggering while the controller keeps detecting the failure
event_controller_fail = Event(
        controller.status_topic.msg.status
        == ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
        keep_event_delay=60.0
    )
# 2. Mapper is NOT Healthy
# We set `handle_once` to True to trigger this event only ONCE during the lifetime oft he system
event_mapper_fault = Event(
    mapper_status.msg.status != ComponentStatus.STATUS_HEALTHY,
    handle_once=True
)

# Trigger on ANY new message on the command topic, if the local mapper is healthy
event_command_received = Event(
    event_condition=(command_topic & (mapper.status_topic.msg.status == ComponentStatus.STATUS_HEALTHY)),
)

# Define Actions
# 1. Drive Manager's Unblock method
# This is a pre-defined method in DriveManager that executes a recovery maneuver
unblock_action = Action(method=driver.move_to_unblock)

# 2. Action to reconfigure Controller to use direct sensor data
activate_direct_sensor_mode_action = update_parameter(
    component=controller, param_name="use_direct_sensor", new_value=True
)

action_process_command = Action(
    method=navigate_to_location,
    # DYNAMIC INJECTION:
    # We pass 'command_topic.msg.data' as the argument.
    # Sugarcoat will fetch the actual string ("kitchen") when the event fires.
    args=(command_topic.msg.data,)
)


# Define your events/action dictionary
events_actions = {
    event_controller_fail: unblock_action,
    event_mapper_fault: activate_direct_sensor_mode_action,
    event_command_received: action_process_command
}

# Init a launcher
launcher = Launcher()

# Pass kompass components to the launcher and register the events
launcher.kompass(
    components=[planner, controller, driver, mapper],
    activate_all_components_on_start=True,
    multi_processing=True,
    events_actions=events_actions
    )

# Set the robot
launcher.robot = robot_config

# Set the frames
launcher.frames = frames_config

# Optional Generic Fallback Policy: If any component fails -> restart it with unlimited retries
# launcher.on_fail(action_name="restart")

# After all configuration is done bringup the stack
launcher.bringup()


```
