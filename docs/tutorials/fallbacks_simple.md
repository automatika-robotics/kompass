# Making the system robust with Fallbacks

In the previous [Point Navigation](./point_navigation.md) tutorial, we built a standard point navigation stack. While functional, it is "optimistic", it assumes that all the components will keep working smoothly and no errors or failures would happen during runtime.

In reality, sensors disconnect, optimization solvers fail to converge, and dynamic obstacles block paths.

In this tutorial, we will give our *Turtlebot3* a few **Fallback** techniques, so the robot can attempt to **self-heal** and **reconfigure itself** when these inevitable failures occur.

## Handling Algorithm Failures

Algorithms used inside the navigation components can fail during runtime for many reasons. A motion control algorithm used in the `Controller` can fail, for example, due to:

* **Mathematical Singularity:** The optimization solver might fail to find a solution that satisfies all kinematic constraints (e.g., the robot is cornered).
* **Local Minima:** The robot might get "stuck" oscillating in a narrow corridor or doorway.
* **Tracking Error:** The robot has drifted too far from the path, and the controller cannot compute a valid velocity to rejoin it smoothly.

<span class="sd-text-primary" style="font-weight: bold; font-size: 1.2em;">Implementation</span>

If our primary high-performance algorithm (e.g., `DWA`) fails, we shouldn't just stop. We can switch to a "Safety" algorithm (like a simple `PurePursuit`).

- Start by importing the `Action` class required to define the Fallback routines, and the `ControllersID` enum for selecting our main controller and our fallback controller:

```python
from kompass.ros import Action
from kompass.control import ControllersID
```

- Select the primary control algorithm, and define the `Action` to switch the algorithm into our fallback choice:

```python
# Select the primary control algorithm
controller.algorithm = ControllersID.DWA

# Define the action
switch_algorithm_action =  Action(method=controller.set_algorithm, args=(ControllersID.PURE_PURSUIT,))
```

- Set the fallbacks action sequence: Try restarting the controller -> If fails again -> Switch the control algorithm
```python
controller.on_algorithm_fail(action=[Action(controller.restart), switch_algorithm_action], max_retries=1)
```

## Handling System Failures

Some component, like the `DriveManager` talks directly to the low-level hardware (micro-controller/motor drivers). And in rel-life the robot can face different system failures during runtime:

* **Cable Issues:** Loose USB/Serial cables caused by vibration.
* **EMI:** Electromagnetic interference causing packet loss on the serial bus.
* **Watchdog Trip:** The micro-controller resets itself because it didn't receive a command in time.


### Implementation

To make our system more resilient, we can implement a simple 'restart' fallback for system failures. Since hardware glitches are often transient, this simple restart fallback can re-establishes the serial handshake and be quite effective.

- We can do this by simply adding the following line to our recipe:

```python
# Not setting max_retires -> will attempt restarting infinity
driver.on_system_fail(Action(driver.restart))

```

- We can also select a general **component-level** fallback for all our component by doing:

```python
launcher.on_fail(action_name="restart")
```

## Complete Recipe

Here is the complete `turtlebot3_with_fallbacks.py` script combining the [Point Navigation](https://www.google.com/search?q=./point_navigation.md) setup with our new robustness layer.

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

# Init a launcher
launcher = Launcher()

# Pass kompass components to the launcher
launcher.kompass(
    components=[planner, controller, driver, mapper],
    activate_all_components_on_start=True,
    multi_processing=True)

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

Your robot can now heal itself from internal failures! But what about reacting to the *external* world?

In the next tutorial, we will use **Events** to make the robot reactive to its environment.

:::{button-link} events_cross_healing.html
:color: primary
:ref-type: doc
:outline:
Adding Reflexes (Events) →
:::
