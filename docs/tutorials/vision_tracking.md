# Following a moving target using vision

In this tutorial we will create a vision-based target following navigation system to follow  using an RGB camera input.

## Before you start

### Get and start your camera ROS2 node

Based on the type of the camera used on your robot, you need to install and launch its respective ROS2 node provided by the manufacturer.

To run and test this example on your development machine, you can use your webcam along with the `usb_cam` package:
```{shell}
sudo apt install ros-<ros2-distro>-usb-cam
ros2 run usb_cam usb_cam_node_exe
```

### Set vision detection/tracking using an ML model

To implement and run this example we will need a detection model processing the RGB camera images to provide the Detection or Tracking information. For this we will use a [Vision Component](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.components.vision.html) from from [ROS Agents](https://automatika-robotics.github.io/ros-agents/intro.html) package, and will use [RoboML] to deploy and serve the model locally.

Therefore, before starting with this tutorial you need to install both packages:

- Install ROS Agents: check the instructions [here](https://automatika-robotics.github.io/ros-agents/installation.html)
- Install RoboML: `pip install roboml`

```{note}
ROS Agents is another [ROS Sugar](https://automatika-robotics.github.io/ros-sugar/) based package used for creating interactive embodied agents that can understand, remember, and act upon contextual information from their environment.
```

After installing both packages, you can start `roboml` to serve the model later either on the robot (or your same development machine), or on another machine in the local network or in the cloud. To start a RESP server run:

```{shell}
roboml-resp
```
```{tip}
Save the IP of the machine running `roboml` as we will use it later in our model client
```


## Setting up the vision model and model client

First, we need to import the `VisionModel` class that defines the model used later in the component, and a [model client](https://automatika-robotics.github.io/ros-agents/basics.html#model-db-client) to communicate with the model which can be running on the same hardware or in the cloud. Here we will use a `RESPModelClient` from [RoboML](https://github.com/automatika-robotics/roboml/)

```python
from agents.models import VisionModel
from agents.clients.roboml import RESPModelClient
```

Now let's configure the model and the model client:

```python
object_detection = VisionModel(
    name="object_detection",
    checkpoint="rtmdet_tiny_8xb32-300e_coco",
    setup_trackers=True,
    deploy_tensorrt=True
)
roboml_detection = RESPModelClient(object_detection, host='192.168.1.1', logging_level="warn")
  # 192.168.1.1 should be replaced by the IP of the machine running roboml
```
The model is configured with a name and a checkpoint (any checkpoint from mmdet framework, see [available checkpoints](https://github.com/open-mmlab/mmdetection?tab=readme-ov-file#overview-of-benchmark-and-model-zoo)). We also set the options `setup_trackers` and `deploy_tensorrt` to `True` to enable publishing tracking information and deploying the vision model using NVIDIA TensorRT.

The `RESPModelClient` is provided with the `VisionModel` we just configured and with the host IP. This IP is set to the IP of the machine running RoboML.

```{seealso}
See all available VisionModel options [here](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.models.html), and all available model clients in agents [here](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.clients.html)
```

## Setting up the Vision Component

We start by importing the required component along with its configuration class from `agents`:

```python
from agents.components import Vision
from agents.config import VisionConfig
```

After setting up the model client, we need to select the input/output topics to configure the vision component:

```python
from agents.ros import Topic

# RGB camera input topic is set to the compressed image topic
image0 = Topic(name="/image_raw/compressed", msg_type="CompressedImage")

# Select the output topics: detections and trackings
detections_topic = Topic(name="detections", msg_type="Detections")
trackings_topic = Topic(name="trackings", msg_type="Trackings")

# Select the vision component configuration
detection_config = VisionConfig(
    threshold=0.5, enable_visualization=True
)

# Create the component
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic, trackings_topic],
    trigger=image0,
    config=detection_config,
    model_client=roboml_detection,
    component_name="detection_component",
)
```
The component inputs/outputs are defined to get the images from the camera topic and provide both detections and trackings. The component is provided theses inputs and outputs along with the previous RoboMl model RESP client. The `trigger` of the component is set to the image input topic so the component would work in an Event-Based runtype and provide a new detection/tracking on each new image.

In the component configuration, the parameter `enable_visualization` is set to `True` to get a visualization of the output on an additional pop-up window for debugging purposes. and `threshold` (confidence threshold for object detection) is set to `0.5`.

```{seealso}
Discover the different configuration options of the Vision component in the [docs](https://automatika-robotics.github.io/ros-agents/apidocs/agents/agents.models.html#agents.models.VisionModel)
```

```{seealso}
For detailed example for using the vision component check this [tutorial](https://automatika-robotics.github.io/ros-agents/examples/prompt_engineering.html)
```

## Setup your robot

We can select the robot motion model, control limits and other geometry parameters using the `RobotConfig` class

```python
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
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.2, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),
)
```

```{seealso}
See more details about the robot configuration in the point navigation [tutorial](../quick_start.md/#step-1-setup-your-robot) and in the `RobotConfig` class [details](../navigation/robot.md)
```

## Import the required components from Kompass

To implement the target following system we will use the `Controller` component to generate the tracking commands and the `DriveManager` to handle the safe communication with the robot driver

```python
from kompass.components import Controller, ControllerConfig, DriveManager
```

## Configure the VisionFollower and setup the components

We select the vision follower method parameters by importing the config class `VisionFollowerConfig` (see default parameters [here](../advanced/algorithms/vision.md)), then configure both our components:

```python
from kompass.control import VisionFollowerConfig

# Set the controller component configuration
config = ControllerConfig(loop_rate=10.0, ctrl_publish_type="Sequence", control_time_step=0.3)

# Init the controller
controller = Controller(
    component_name="my_controller", config=config
)
# Set the vision tracking input to either the detections or trackings topic
controller.inputs(vision_tracking=detections_topic)

# Set the vision follower configuration
vision_follower_config = VisionFollowerConfig(
    control_horizon=3, enable_search=False, target_search_pause=6, tolerance=0.2
)
controller.algorithms_config = vision_follower_config  # We can also configure other algorithms (DWA, DV etc.) and pass a list to the algorithms_config property

# Init the drive manager with the default parameters
driver = DriveManager(component_name="my_driver")

```
Here we selected a loop rate for the controller of `10Hz` and a control step for generating the commands of `0.3s`, and we selected to send the commands sequentially as they get computed. The vision follower is configured with a `control_horizon` equal to three future control time steps and a `target_search_pause` equal to 6 control time steps. We also chose to disable the search, meaning that the tracking action would end when the robot looses the target.

```{tip}
`target_search_pause` is implemented so the robot would pause and wait while tracking to avoid loosing the target due to quick movement and slow model response. It should be adjusted based on the inference time of the model.
```

## Add all the components to the launcher

All that is left is to add all the three previous components to the launcher and bringup the system!
```python
from kompass.launcher import Launcher

launcher = Launcher()
launcher.add_pkg(
    components=[vision],
    ros_log_level="warn",
)
launcher.kompass(
    components=[controller, driver],
)
# Set the robot config for all components
launcher.robot = my_robot
launcher.bringup()
```

Et voila! our code for the full vision-based target follower is ready and here is the complete script

```{code-block} python
:caption: vision_follower
:linenos:

import numpy as np
from agents.components import Vision
from agents.models import VisionModel
from agents.clients.roboml import RESPModelClient
from agents.config import VisionConfig
from agents.ros import Topic

from kompass.components import Controller, ControllerConfig, DriveManager
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
from kompass.control import VisionFollowerConfig
from kompass.launcher import Launcher

# RGB camera input topic is set to the compressed image topic
image0 = Topic(name="/image_raw/compressed", msg_type="CompressedImage")

# Select the output topics: detections and trackings
detections_topic = Topic(name="detections", msg_type="Detections")
trackings_topic = Topic(name="trackings", msg_type="Trackings")

object_detection = VisionModel(
    name="object_detection",
    checkpoint="rtmdet_tiny_8xb32-300e_coco",
    setup_trackers=True,
    deploy_tensorrt=True,
)

roboml_detection = RESPModelClient(
    object_detection, host="192.168.1.1", logging_level="warn"
)

# Select the vision component configuration
detection_config = VisionConfig(threshold=0.5, enable_visualization=True)

# Create the component
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic, trackings_topic],
    trigger=image0,
    config=detection_config,
    model_client=roboml_detection,
    component_name="detection_component",
)

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.2, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),
)
# Set the controller component configuration
config = ControllerConfig(
    loop_rate=10.0, ctrl_publish_type="Sequence", control_time_step=0.3
)

# Init the controller
controller = Controller(component_name="my_controller", config=config)
# Set the vision tracking input to either the detections or trackings topic
controller.inputs(vision_tracking=detections_topic)

# Set the vision follower configuration
vision_follower_config = VisionFollowerConfig(
    control_horizon=3, enable_search=False, target_search_pause=6, tolerance=0.2
)
controller.algorithms_config = vision_follower_config  # We can also configure other algorithms (DWA, DV etc.) and pass a list to the algorithms_config property

# Init the drive manager with the default parameters
driver = DriveManager(component_name="my_driver")

launcher = Launcher()
launcher.add_pkg(
    components=[vision],
    ros_log_level="warn",
)
launcher.kompass(
    components=[controller, driver],
)
# # Set the robot config for all components
launcher.robot = my_robot
launcher.bringup()


```

## Trigger the following action

After running your complete system you can send a goal to the controller's action server `/my_controller/track_vision_target` of type [`kompass_interfaces.action.TrackVisionTarget`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/action) to start tracking a selected label (`person` for example):

```shell
ros2 action send_goal /my_controller/track_vision_target kompass_interfaces/action/TrackVisionTarget "{label: 'person'}"
```

You can also re-run the previous script and activate the target search:

```python
vision_follower_config = VisionFollowerConfig(
    control_horizon=3, enable_search=True, target_search_pause=6, tolerance=0.2
)
```

```shell
ros2 action send_goal /my_controller/track_vision_target kompass_interfaces/action/TrackVisionTarget "{label: 'person', search_radius: 1.0, search_timeout: 30}"
```
