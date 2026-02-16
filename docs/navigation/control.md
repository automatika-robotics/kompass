# Controller

**Motion control and dynamic obstacle avoidance.**

The Controller is the real-time "pilot" of your robot. While the [Planner](./path_planning.md) looks ahead to find a global route, the Controller deals with the immediate reality, calculating velocity commands to follow the global path (path following) or a global target point (object following) while reacting to dynamic obstacles and adhering to kinematic constraints.

It supports modular **Plugins** allowing you to switch between different control strategies (e.g., *Pure Pursuit* vs *DWA* vs *Visual Servoing*) via configuration.


## Run Types

The Controller typically runs at a high frequency (10Hz-50Hz) to ensure smooth motion. Set the run type directly from Controller 'run_type' property.

```{list-table}
:widths: 20 80
* - **{material-regular}`schedule;1.2em;sd-text-primary` Timed**
  - **Periodic Control Loop.** Computes a new velocity command periodically if all necessary inputs are available.

* - **{material-regular}`hourglass_top;1.2em;sd-text-primary` Action Server**
  - **Goal Tracking.** Offers a [`ControlPath`](https://github.com/automatika-robotics/kompass/blob/main/kompass_interfaces/action/ControlPath.action) ROS2 Action. Continuously computes control commands until the goal is reached or the action is preempted.

```

## Interface

### Inputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">plan</span>
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - `/plan`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">location</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `/odom` (`Odometry`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">sensor_data</span>
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1
  - `/scan` (`LaserScan`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">local_map</span>
  - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
  - 1
  - `/local_map/occupancy_layer`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">vision_detections</span>
  - [`automatika_embodied_agents.msg.Trackings`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg), [`automatika_embodied_agents.msg.Detections2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
  - 1
  - None, Should be provided to use the vision target tracking
```

```{tip}
Provide a 'vision_tracking' input topic to the controller to activate the creation of the a vision-based target following action server. See [this example](../tutorials/vision_tracking.md) for more details.
```

### Outputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">command</span>
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - 1
  - `/control` (`Twist`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">multi_command</span>
  - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/msg)
  - 1
  - `/control_list`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">interpolation</span>
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - `/interpolated_path`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">local_plan</span>
  - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
  - 1
  - `/local_path`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">tracked_point</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)[`automatika_embodied_agents.msg.Detection2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
  - 1
  - `/tracked_point` (`PoseStamped`)
```

## Algorithms

Kompass includes several production-ready control plugins suited for different environments.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`timeline;1.2em;sd-text-primary` PurePursuit
:link: ../advanced/algorithms/pure_pursuit
:link-type: doc
:class-card: sugar-card

**Path Tracking**
Geometric path tracking controller. Calculates the curvature to move the robot from its current position to a look-ahead point on the path.
:::

:::{grid-item-card} {material-regular}`call_split;1.2em;sd-text-primary` Stanley
:link: ../advanced/algorithms/stanley
:link-type: doc
:class-card: sugar-card

**Front-Wheel Feedback**
Geometric path tracking controller. Uses the front axle as a reference point to minimize cross-track error. Best for Ackermann steering.
:::

:::{grid-item-card} {material-regular}`grid_on;1.2em;sd-text-primary` DWA
:link: ../advanced/algorithms/dwa
:link-type: doc
:class-card: sugar-card

**Dynamic Window Approach (GPU Support)**
Sample-based collision avoidance. Considers robot kinematics to find the optimal velocity that progresses towards the goal without hitting obstacles.
:::

:::{grid-item-card} {material-regular}`bubble_chart;1.2em;sd-text-primary` DVZ
:link: ../advanced/algorithms/dvz
:link-type: doc
:class-card: sugar-card

**Deformable Virtual Zone**
Reactive collision avoidance based on risk zones. Extremely fast and efficient for crowded dynamic environments.
:::

:::{grid-item-card} {material-regular}`videocam;1.2em;sd-text-primary` VisionFollowerRGB
:link: ../advanced/algorithms/vision_follower_rgb
:link-type: doc
:class-card: sugar-card

**Visual Servoing**
Steers the robot to keep a visual target (bounding box or keypoint) centered in the camera frame.
:::

:::{grid-item-card} {material-regular}`blur_on;1.2em;sd-text-primary` VisionFollowerRGBD
:link: ../advanced/algorithms/vision_follower_rgbd
:link-type: doc
:class-card: sugar-card

**Depth-Aware Servoing**
Maintains a specific **distance** and **orientation** relative to the target using Depth/Pointcloud data. Perfect for "Follow Me" behaviors.
:::
::::

## Usage Example

```python
from kompass.components import Controller, ControllerConfig
from kompass.ros import Topic

# 1. Configuration
# Set the loop rate (Control frequency)
my_config = ControllerConfig(loop_rate=20.0)

# 2. Instantiate
# Select the plugin via the component definition or config
my_controller = Controller(component_name="controller", config=my_config)

# 3. Setup
# Change an input topic name
my_controller.inputs(plan=Topic(name='/global_path', msg_type='Path'))

# Change run type to an Action Server (for long-running goals)
my_controller.run_type = "ActionServer"

# Select the Algorithm Plugin
my_controller.algorithm = 'DWA'

```

```{seealso}
Discover the Controller's detailed configuration options in the [ControllerConfig](../apidocs/kompass/kompass.components.controller.md/#classes)
```

## See Next

The Controller relies on the **Drive Manager** to translate these velocity commands into actual motor signals for your specific hardware.

:::{button-link} driver.html
:color: primary
:ref-type: doc
:outline:
Discover the Drive Manager →
:::
