# Launcher

[Launcher](../apidocs/Kompass/Kompass.launcher.md) is a class created to provide a more pythonic way to launch and configure ROS nodes.

Launcher starts a pre-configured component or a set of components as ROS2 nodes using multi-threaded or multi-process execution. Launcher spawns an internal [Monitor](./monitor.md) node in a separate thread in both execution types.

:::{figure-md} fig-multi-thread

<img src="../_static/images/diagrams/multi_threaded.jpg" alt="Multi-threaded execution" width="500px">

Multi-threaded execution
:::

:::{figure-md} fig-multi-process

<img src="../_static/images/diagrams/multi_process.jpg" alt="Multi-process execution" width="500px">

Multi-process execution
:::

Launcher can also manage a set of Events-Actions through its internal Monitor node (See Monitor class).

## Available options:
- Enable/Disable events monitoring
- Enable/Disable multi-processing, if disabled the components are launched in threads
- Select to activate one, many or all components on start (lifecycle nodes activation)

Launcher forwards all the provided Events to its internal Monitor, when the Monitor detects an Event trigger it emits an InternalEvent back to the Launcher. Execution of the Action is done directly by the Launcher or a request is forwarded to the Monitor depending on the selected run method (multi-processes or multi-threaded).

:::{note} While Launcher supports executing standard [ROS2 launch actions](https://github.com/ros2/launch). Launcher does not support standard [ROS2 launch events](https://github.com/ros2/launch/tree/rolling/launch/launch/events) for the current version.
:::

## Usage Example

```{code-block} python
:caption: launcher test
:linenos:

from kompass.components import Planner, Controller
from kompass.actions import Actions
from kompass.event import OnLess
from kompass_interfaces.action import PlanPath
from geometry_msgs.msg import Pose
from kompass_interfaces.msg import PathTrackingError

# Create your components
planner = Planner(node_name='test_planner')
controller = Controller(node_name='test_controller')


# Configure your components here
# ....
planner.run_type = "ActionServer"

# Create your events
low_battery = OnLess(
    "low_battery",
    Topic(name="/battery_level", msg_type="Int"),
    15,
    ("data")
)

# Charging station location
charging_location = Pose()
charging_location.position.x = 10.0

# Tolerance for reaching charging station
end_tolerance = PathTrackingError()
end_tolerance.orientation_error = 0.1
end_tolernace.lateral_distance_error = 0.05

action_goal = PlanPath.Goal()
action_goal.end_tolerance = end_tolerance
action_goal.goal = charging_location

# Define action to send a goal to the planner to start a mission to the charging station
return_to_charging_station = Actions.send_action_goal(
        action_name="test_planner/plan_path",
        action_type=PlanPath,
        action_request_msg=action_goal
    )

# Events/Actions
my_events_actions: Dict[event.Event, Action] = {
    low_battery: return_to_charging_station
}

# We can add a config YAML file
path_to_yaml = 'my_config.yaml'

launcher = Launcher(
    components=[planner, controller],
    events_actions=my_events_actions,
    config_file=path_to_yaml,
    activate_all_components_on_start=True,
    multi_processing=True,
)

# Set a topic for all components
odom_topic = Topic(name="robot_odom", msg_type="Odometry")
launcher.inputs(location=odom_topic)

# If any component fails -> restart it with unlimited retries
launcher.on_component_fail(action_name="restart")

# Bring up the system
launcher.bringup(ros_log_level="info", introspect=False)
```
