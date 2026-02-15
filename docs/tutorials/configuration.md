# Configuring Components Your Way

Kompass is built for flexibility — and that starts with how you configure your components.

Whether you're scripting in Python, editing clean and readable YAML, crafting elegant TOML files, or piping in JSON from a toolchain, **Kompass lets you do it your way**.

No more rigid formats or boilerplate structures. Just straightforward, expressive configuration — however you like to write it.

Here's how to define your components using:
- 🐍 [Python API](#-python-api)
- 📄 [YAML](#-yaml)
- 🍅 [TOML](#-toml)
- 🟨 [JSON](#-json)

Pick your flavor. Plug it in. Go.

## 🐍 Python API

Use the full power of the Pythonic API of Kompass to configure your components when you want dynamic logic, computation, or tighter control.

```python
from kompass.components import Planner, PlannerConfig
from kompass.ros import Topic
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
    RobotFrames
)
import numpy as np
import math

# Define your robot's physical and control characteristics
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,            # Type of robot motion model
    geometry_type=RobotGeometry.Type.CYLINDER,          # Shape of the robot
    geometry_params=np.array([0.1, 0.3]),                # Diameter and height of the cylinder
    ctrl_vx_limits=LinearCtrlLimits(                     # Linear velocity constraints
        max_vel=0.4,
        max_acc=1.5,
        max_decel=2.5
    ),
    ctrl_omega_limits=AngularCtrlLimits(                 # Angular velocity constraints
        max_vel=0.4,
        max_acc=2.0,
        max_decel=2.0,
        max_steer=math.pi / 3                            # Steering angle limit (radians)
    ),
)

# Define the robot's coordinates frames
my_frames = RobotFrames(
    world="map",
    odom="odom",
    robot_base="body",
    scan="lidar_link"
)

# Create the planner config using your robot setup
config = PlannerConfig(
    robot=my_robot,
    loop_rate=1.0
)

# Instantiate the Planner component
planner = Planner(
    component_name="planner",
    config=config
)

# Additionally configure the component's inputs or outputs
planner.inputs(
    map_layer=Topic(name="/map", msg_type="OccupancyGrid"),
    goal_point=Topic(name="/clicked_point", msg_type="PointStamped")
)
```

## 📄 YAML

Similar to traditional ROS2 launch, you can still maintain all your configuration parameters in a YAML file.

```yaml
/**:
  # Fames and Robot configuration can be passed under the component name
  # They can also be kep here under 'common config' to be used for multiple components
  frames:
    robot_base: "body"
    odom: "odom"
    world: "map"
    scan: "lidar_link"

  robot:
    model_type: "DIFFERENTIAL_DRIVE"
    geometry_type: "CYLINDER"
    geometry_params: [0.1, 0.3]

    ctrl_vx_limits:
      max_vel: 0.4
      max_acc: 1.5
      max_decel: 2.5

    ctrl_omega_limits:
      max_vel: 0.4
      max_acc: 2.0
      max_decel: 2.0
      max_steer: 1.0472  # ≈ π / 3

planner:
  inputs:
    map_layer:
      name: "/map"
      msg_type: "OccupancyGrid"
    goal_point:
      name: "/clicked_point"
      msg_type: "PointStamped"
  loop_rate: 1.0
```

## 🍅 TOML

Not a fan of YAML? No worries — Kompass lets you configure your components using TOML too.
TOML offers clear structure and excellent tooling support, making it perfect for clean, maintainable configs.

```toml
["/**".frames]
robot_base = "body"
odom = "odom"
world = "map"
scan = "lidar_link"

["/**".robot]
model_type = "DIFFERENTIAL_DRIVE"
geometry_type = "CYLINDER"
geometry_params = [0.1, 0.3]

["/**".robot.ctrl_vx_limits]
max_vel = 0.4
max_acc = 1.5
max_decel = 2.5

["/**".robot.ctrl_omega_limits]
max_vel = 0.4
max_acc = 2.0
max_decel = 2.0
max_steer = 1.0472  # ≈ π / 3

[planner]
loop_rate = 1.0

[planner.inputs.map_layer]
name = "/map"
msg_type = "OccupancyGrid"

[planner.inputs.goal_point]
name = "/clicked_point"
msg_type = "PointStamped"
```

## 🟨 JSON

Prefer curly braces? Or looking to pipe configs from an ML model or external toolchain?
JSON is machine-friendly and widely supported — perfect for automating your Kompass configuration with generated files.

```json
{
  "/**": {
    "frames": {
      "robot_base": "body",
      "odom": "odom",
      "world": "map",
      "scan": "lidar_link"
    },
    "robot": {
      "model_type": "DIFFERENTIAL_DRIVE",
      "geometry_type": "CYLINDER",
      "geometry_params": [0.1, 0.3],
      "ctrl_vx_limits": {
        "max_vel": 0.4,
        "max_acc": 1.5,
        "max_decel": 2.5
      },
      "ctrl_omega_limits": {
        "max_vel": 0.4,
        "max_acc": 2.0,
        "max_decel": 2.0,
        "max_steer": 1.0472
      }
    }
  },
  "planner": {
    "loop_rate": 1.0,
    "inputs": {
      "map_layer": {
        "name": "/map",
        "msg_type": "OccupancyGrid"
      },
      "goal_point": {
        "name": "/clicked_point",
        "msg_type": "PointStamped"
      }
    }
  }
}
```

```{note}
Make sure to pass your config file to the component on initialization or to the Launcher.
```
