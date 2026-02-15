# Vision Tracking Using Depth Information

This tutorial guides you through creating a vision tracking system using a depth camera. We'll leverage RGB-D with the `VisionRGBDFollower` in Kompass to detect and follow objects more robustly. With the depth information available, this will create more precise understanding of the environment and lead to more accurate and robust object following (as compared to [using RGB images](./vision_tracking.md)).

## Before you start

Lets take care of some preliminary setup.

### Setup Your Depth Camera ROS2 Node

First things first — your robot needs a depth camera to see in 3D and get the `RGBD` input. For this tutorial, we are using an **Intel RealSense** that is available on many mobile robots and well supported in ROS2 and in simulation.

To get your RealSense camera running:

```bash
sudo apt install ros-<ros2-distro>-realsense2-camera

# Launch the camera node to start streaming both color and depth images
ros2 launch realsense2_camera rs_camera.launch.py
```

### Start vision detection using an ML model

To implement and run this example we will need a detection model processing the RGBD camera images to provide the Detection information. Similarly to the [previous tutorial](vision_tracking.md), we will use [**EmbodiedAgents**](https://automatika-robotics.github.io/ros-agents/intro.html) package. EmbodiedAgents provides a [Vision Component](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.components.vision.html), which will allow us to easily deploy a ROS node in our system that interacts with vision models.

Therefore, before starting with this tutorial you need to install both packages:

- Install **EmbodiedAgents**: check the instructions [here](https://automatika-robotics.github.io/ros-agents/installation.html)

```{seealso}
[EmbodiedAgents](https://automatika-robotics.github.io/embodied-agents) is another [Sugarcoat🍬](https://automatika-robotics.github.io/sugarcoat) based package used for creating interactive embodied agents that can understand, remember, and act upon contextual information from their environment.
```

## Setting up the vision component and model client in EmbodiedAgents

In this example, we will set `enable_local_classifier` to `True` in the vision component so the model would be deployed directly on the robot. Additionally, we will set the input topic to be the `RGBD` camera topic. This setting will allow the `Vision` component to **publish both the depth and the rgb image data along with the detections**.

```python
from agents.components import Vision
from agents.config import VisionConfig
from agents.ros import Topic

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
```

```{seealso}
See all available VisionModel options [here](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.models.html), and all available model clients in agents [here](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.clients.html)
```

## Setup your robot and your Controller

You can setup your robot in the same way we did in the [previous tutorial](./vision_tracking.md/#setup-your-robot).
```python
from kompass.actions import Action
from kompass.event import OnAny, OnChangeEqual
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
import numpy as np

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
```

Then we will setup the `Controller` component to use the `VisionRGBDFollower`. We will also need to provide two additional inputs:

- The detections topic
- The depth camera info topic

```python
from kompass.components import Controller, ControllerConfig

depth_cam_info_topic = Topic(name="/camera/aligned_depth_to_color/camera_info", msg_type="CameraInfo")

config = ControllerConfig(ctrl_publish_type="Parallel")
controller = Controller(component_name="controller", config=config)  # Optionally a config file can be provided here config_file=path_to_config_file
controller.inputs(vision_detections=detections_topic, depth_camera_info=depth_cam_info_topic)
controller.algorithm = "VisionRGBDFollower"
```

## Complete your system with helper components

To make the system more complete and robust, we will add:
- `DriveManager` - to handle sending direct commands to the robot and ensure safety with its emergency stop
- `LocalPlanner` - to provide the controller with more robust local perception, to do so we will also set the controller's `direct_sensor` property to `False`

```python
from kompass.components import DriveManager, LocalMapper

controller.direct_sensor = False
driver = DriveManager(component_name="driver")
mapper = LocalMapper(component_name="local_mapper")
```

## Bring it all together!

```{code-block} python
:caption: vision_depth_follower.py
:linenos:

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
from kompass.launcher import Launcher
import numpy as np


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

# Bring it up with the launcher
launcher = Launcher()
launcher.add_pkg(components=[vision], ros_log_level="warn",
                 package_name="automatika_embodied_agents",
                 executable_entry_point="executable",
                 multiprocessing=True)
launcher.kompass(components=[controller, mapper, driver])
# Set the robot config for all components
launcher.robot = my_robot
launcher.bringup()
```

```{tip}
You can take your design to the next step and make your system more robust by adding some [events](events_actions.md) or defining some [fallbacks](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html)!
```
