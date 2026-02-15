
# Events for External Reflexes

In the previous [Vision Tracking Using Depth](vision_tracking_depth.md) tutorial, the robot was using the `VisionRGBDFollower` algorithm. This meant that the robot will wait for a user to send a request to the controller to follow a an object with a given ID.

In this tutorial, we will make the behavior intelligent. The robot will:
1.  **Idle/Patrol** by default.
2.  **Switch to Follow Mode** automatically when a "person" is detected in the video feed.


## 1. The Setup

**Prerequisites:** Completed [Vision Tracking Using Depth](vision_tracking_depth.md).

We use the same component setup (Vision, Controller, DriveManager), but we initialize the Controller in path following mode (standard navigation) instead of Vision mode.

```python
# Start the controller in standard navigation mode (PathFollowing)
controller = Controller(component_name="controller")
controller.algorithm = "PurePursuit" # Default mode
controller.direct_sensor = True

# Vision Component (as before)

```


## 2. Event A: Person Detected -> Follow

We want to detect if the class "person" appears in the detections list. If it does, we trigger the Controller's Action Server to start the tracking behavior.


```python
from kompass.ros import Event

# Trigger if "person" is in the list of detected labels
event_person_detected= Event(
    event_condition=detections_topic.msg.labels.contains("person"),
    on_change=True
)

```

### The Action

We need to tell the Controller to switch behaviors. We use two actions:

1. **Switch Algorithm:** Change the internal logic to `VisionRGBDFollower`.
2. **Trigger Action Server:** Send a goal to the controller to start the active tracking loop.

```{tip}
If you link an Event to a set of Actions, the action set will get executed in sequence.
```

```python
# Import the required actions
from kompass.actions import update_parameter, send_component_action_server_goal

# Import the vision follower 'ActionServer' ROS2 message
from kompass_interfaces.action import TrackVisionTarget

# Switch to vision following
# Option 1: use set_algorithm action from the controller (same as in the fallbacks tutorial)
# switch_algorithm_action =  Action(method=controller.set_algorithm, args=(ControllersID.VISION_DEPTH,))

# Option 2: use the update_parameter action to change the 'algorithm' parameter value
# the value can be set to ControllersID.VISION_DEPTH or directly the string name of the algorithm class VisionRGBDFollower
switch_algorithm_action = update_parameter(
    component=controller,
    param_name="algorithm",
    new_value="VisionRGBDFollower"
)

# Action to trigger the action server to follow a person
action_request_msg = TrackVisionTarget.Goal()
action_request_msg.label = "person"  # Specify the target to follow
action_start_person_following = send_component_action_server_goal(
    component=controller,
    request_msg=action_request_msg,
)

# Set the event/action(s) pair
events_action = {
    event_person_detected: [switch_algorithm_action, action_start_person_following]
}

```


## 4. Complete Recipe

Here is the complete script. Launch this, run it and stand in front of the robot to make it follow you!

```{code-block} python
:caption: turtlebot3_with_fallbacks.py
:linenos:

import numpy as np
from agents.components import Vision
from agents.config import VisionConfig
from agents.ros import Topic
from kompass.components import Controller, ControllerConfig, DriveManager, LocalMapper
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
from kompass.ros import Launcher, Event
from kompass.actions import update_parameter, send_component_action_server_goal
from kompass_interfaces.action import TrackVisionTarget


image0 = Topic(name="/camera/rgbd", msg_type="RGBD")
detections_topic = Topic(name="detections", msg_type="Detections")

detection_config = VisionConfig(threshold=0.5, enable_local_classifier=True)
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic],
    trigger=image0,
    config=detection_config,
    component_name="detection_component",
)

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=1,0, max_acc=3.0, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=4.0, max_acc=6.0, max_decel=10.0, max_steer=np.pi / 3
    ),
)

depth_cam_info_topic = Topic(name="/camera/aligned_depth_to_color/camera_info", msg_type="CameraInfo")

# Setup the controller
config = ControllerConfig(ctrl_publish_type="Parallel")
controller = Controller(component_name="controller", config=config)
controller.inputs(vision_detections=detections_topic, depth_camera_info=depth_cam_info_topic)
controller.algorithm = "VisionRGBDFollower"
controller.direct_sensor = False

# Add additional helper components
driver = DriveManager(component_name="driver")
mapper = LocalMapper(component_name="local_mapper")


# Define Event: Trigger if "person" is in the list of detected labels
event_person_detected= Event(
    event_condition=detections_topic.msg.labels.contains("person"),
    on_change=True
)

# Define Action(s)
switch_algorithm_action = update_parameter(
    component=controller,
    param_name="algorithm",
    new_value="VisionRGBDFollower"
)

# Action to trigger the action server to follow a person
action_request_msg = TrackVisionTarget.Goal()
action_request_msg.label = "person"  # Specify the target to follow
action_start_person_following = send_component_action_server_goal(
    component=controller,
    request_msg=action_request_msg,
)

# Set the event/action(s) pair
events_action = {
    event_person_detected: [switch_algorithm_action, action_start_person_following]
}

# Bring it up with the launcher
launcher = Launcher()

launcher.add_pkg(components=[vision], ros_log_level="warn",
                 package_name="automatika_embodied_agents",
                 executable_entry_point="executable",
                 multiprocessing=True)

# Add component and the events to monitor
launcher.kompass(components=[controller, mapper, driver], events_actions=events_action)

# Set the robot config for all components
launcher.robot = my_robot
launcher.bringup()

```

## Next Steps

We have handled single-topic events for both cross-component healing and external reflexes. But real-world decisions are rarely based on one factor.

In the next tutorial, we will use **Logic Gates** to create smarter, composed events (e.g., "If Emergency Stop **AND** Battery Low").

:::{button-link} events_composed.html
:color: primary
:ref-type: doc
:outline:
Logic Gates & Composed Events →
:::
