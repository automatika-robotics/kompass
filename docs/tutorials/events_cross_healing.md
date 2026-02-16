# Events For Cross-Component Healing

In the [Fallbacks](fallbacks_simple.md) tutorial, we learned how a component can heal *itself* (e.g., restarting itself). But sophisticated autonomy requires more than just self-repair. It requires **System-Level Awareness** and **Behavioral Reflexes**.

In this tutorial, we will use simple **Events** to implement **Cross-Component Healing:** Components monitoring each other to fix system-level issues.


## Scenario A: The "Unstuck" Reflex

The `Controller` gets stuck in a local minimum (e.g., the robot is facing a corner). It will report an `ALGORITHM_FAILURE` because it cannot find a valid velocity command. We detect this status and ask the `DriveManager` to execute a blind "Unblock" maneuver (e.g., rotate in place or back up).

```{tip}
All components health status topics are accessible in the API using `component.status_topic`
```

```python
# Import the required primitives
from kompass.ros import Event, Action, Topic

# Import the ComponentStatus message to access the status values
from automatika_ros_sugar.msg import ComponentStatus


# Define Event: Controller reports Algorithm Failure
# We set a `on_change` True to trigger the event only when the status changes to algorithm failure the first time
# i.e. the associated action will not keep triggering while the controller keeps detecting the failure
event_controller_fail = Event(
        controller.status_topic.msg.status
        == ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
        keep_event_delay=60.0
    )

# Define Action: Drive Manager's Unblock method
# This is a pre-defined method in DriveManager that executes a recovery maneuver
unblock_action = Action(method=driver.move_to_unblock)
```

## Scenario B: The "Blind" Reflex

The `LocalMapper` crashes or faces an error failing to provide a high fidelity local map. The `Controller`, which relies on the map, can detect that the `LocalMapper is "not healthy" and reconfigure itself to use raw sensor data directly (Reactive Mode).

```python
# Import the reconfiguration action
from kompass.actions import update_parameter

# Import the ComponentStatus message to access the status values
from automatika_ros_sugar.msg import ComponentStatus

# Define Event: Mapper is NOT Healthy
# We set `handle_once` to True to trigger this event only ONCE during the lifetime oft he system
event_mapper_fault = Event(
    mapper.status_topic.msg.status != ComponentStatus.STATUS_HEALTHY,
    handle_once=True
)


# Define Action: Reconfigure Controller to use direct sensor data
activate_direct_sensor_mode_action = update_parameter(
    component=controller, param_name="use_direct_sensor", new_value=True
)
```

## Register Your Events/Actions

After defining your event/action pairs you only need to register the dictionary with the system launcher when adding your components:

```python
# Define your events/action dictionary
events_actions = {
    event_controller_fail: unblock_action,
    event_mapper_fault: activate_direct_sensor_mode_action,
}
# Pass kompass components to the launcher and register the events
launcher.kompass(
    components=[planner, controller, driver, mapper],
    activate_all_components_on_start=True,
    multi_processing=True,
    events_actions=events_actions
    )
```

## Complete Recipe

Here is the complete `turtlebot3_cross_healing.py` script combining the point navigation system, the fallbacks and the cross-component healing events

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

# Define Actions
# 1. Drive Manager's Unblock method
# This is a pre-defined method in DriveManager that executes a recovery maneuver
unblock_action = Action(method=driver.move_to_unblock)

# 2. Action to reconfigure Controller to use direct sensor data
activate_direct_sensor_mode_action = update_parameter(
    component=controller, param_name="use_direct_sensor", new_value=True
)

# Define your events/action dictionary
events_actions = {
    event_controller_fail: unblock_action,
    event_mapper_fault: activate_direct_sensor_mode_action,
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

## Next Steps

We have handled single-topic events for cross-component healing. Now let's see how we can use events for external reflexes.

:::{button-link} events_external_reflexes.html
:color: primary
:ref-type: doc
:outline:
Events for External Reflexes →
:::
