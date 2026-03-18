---
title: "Advanced: Health Status, Fallbacks & Events"
---

# Advanced: Health Status, Fallbacks & Events

This guide covers the health monitoring, fallback recovery, and event systems that make Kompass components resilient. Read [Creating a Custom Component](./custom_component.md) first for the basics.

## Health Status

Every component has a `health_status` object (an instance of `Status` from `ros_sugar.core.status`) that tracks the component's operational state. The status is published on the `{node_name}/status` topic and drives the fallback system.

### Status Levels

| Code | Constant | Meaning |
|---|---|---|
| 0 | `STATUS_HEALTHY` | Running normally |
| 1 | `STATUS_FAILURE_ALGORITHM_LEVEL` | An algorithm used by the component failed |
| 2 | `STATUS_FAILURE_COMPONENT_LEVEL` | The component itself (or another component) failed |
| 3 | `STATUS_FAILURE_SYSTEM_LEVEL` | An external dependency failed (e.g. missing topic) |
| 4 | `STATUS_GENERAL_FAILURE` | Unspecified failure |

### Setting Status

Use these methods inside your component to report health:

```python
# Everything is fine
self.health_status.set_healthy()

# Algorithm failure -- include the algorithm name(s) for diagnostics
self.health_status.set_fail_algorithm(
    algorithm_names=["MyAlgorithm"]
)

# Component failure -- defaults to self, or specify other component(s)
self.health_status.set_fail_component()
self.health_status.set_fail_component(
    component_names=["other_component"]
)

# System failure -- an external dependency is unavailable
self.health_status.set_fail_system(
    topic_names=["/map", "/odom"]
)

# Generic failure (no specific category)
self.health_status.set_failure()
```

### Checking Status

```python
self.health_status.is_healthy          # bool
self.health_status.is_algorithm_fail   # bool
self.health_status.is_component_fail   # bool
self.health_status.is_system_fail      # bool
self.health_status.is_general_fail     # bool
self.health_status.value               # int (0-4)
```

### Where to Update Status

The key principle: **set unhealthy when something goes wrong, set healthy when it recovers.** Here is the pattern used by existing components:

#### In `_execution_step()` -- the most common location

```python
def _execution_step(self):
    # Check inputs are available
    if not self.got_all_inputs():
        missing = self.get_missing_inputs()
        self.health_status.set_fail_system(topic_names=missing)
        return

    # Run the algorithm
    try:
        success = self._my_algorithm.compute(self.robot_state)
    except Exception as e:
        self.get_logger().error(f"Algorithm failed: {e}")
        self.health_status.set_fail_algorithm(
            algorithm_names=["MyAlgorithm"]
        )
        return

    if not success:
        self.health_status.set_fail_algorithm(
            algorithm_names=["MyAlgorithm"]
        )
        return

    # Success -- clear any previous failure
    self.health_status.set_healthy()

    # Publish results
    self.get_publisher(TopicsKeys.GOAL_POINT).publish(result)
```

#### In lifecycle transitions -- handled automatically

The base class resets status to healthy on `configure`, `activate`, and `deactivate`. On `error`, it sets `set_fail_component()`. You generally don't need to touch status in lifecycle methods.

### Status Publishing

For `TIMED` components, the status is published automatically at the fallback check rate. For non-timed components (e.g. `ACTION_SERVER`), publish manually after updating:

```python
self.health_status.set_fail_algorithm(algorithm_names=["MyAlgorithm"])
self.health_status_publisher.publish(self.health_status())
```

## Fallbacks

Fallbacks are actions that execute automatically when the health status becomes unhealthy. They provide self-recovery without external intervention.

### Defining Fallbacks

Use the component's setter methods before launching:

```python
from ros_sugar.core import Action

my_component = MyComponent(component_name="my_node")

# On algorithm failure: restart the component (retry up to 3 times)
my_component.on_algorithm_fail(
    action=Action(my_component.restart),
    max_retries=3,
)

# On system failure: just broadcast status (let the Monitor handle it)
my_component.on_system_fail(
    action=Action(my_component.broadcast_status),
    max_retries=None,  # None = unlimited retries
)

# On component failure: try reconfiguring with a new config, then restart
my_component.on_component_fail(
    action=[
        Action(my_component.reconfigure, args=(fallback_config,)),
        Action(my_component.restart),
    ],
    max_retries=2,
)

# Catch-all for any failure type not covered above
my_component.on_fail(
    action=Action(my_component.broadcast_status),
    max_retries=None,
)

# When all fallbacks are exhausted
my_component.on_giveup(
    action=Action(my_component.stop),
    max_retries=1,
)
```

### Fallback Hierarchy

| Method | Triggers on | Priority |
|---|---|---|
| `on_algorithm_fail()` | `set_fail_algorithm()` | Checked first |
| `on_component_fail()` | `set_fail_component()` | Checked second |
| `on_system_fail()` | `set_fail_system()` | Checked third |
| `on_fail()` | Any failure without a specific handler | Catch-all |
| `on_giveup()` | All fallbacks exhausted | Last resort |

The fallback check runs on a timer (default 100 Hz) while the component is active. When `health_status` is not healthy, the system:

1. Looks for a handler matching the specific failure type.
2. Falls back to `on_fail()` if no specific handler exists.
3. Retries the current action up to `max_retries`.
4. For action lists, moves to the next action when retries are exhausted.
5. Calls `on_giveup()` when everything is exhausted.
6. If the action returns `True`, status resets to healthy automatically.

### Action Lists (Sequential Fallbacks)

When you pass a list of actions, they execute in order. Each action is retried `max_retries` times before moving to the next:

```python
# Try broadcast first (2 retries), then restart (2 retries), then reconfigure (2 retries)
my_component.on_algorithm_fail(
    action=[
        Action(my_component.broadcast_status),
        Action(my_component.restart),
        Action(my_component.reconfigure, args=(safe_config,)),
    ],
    max_retries=2,
)
```

> **Note:** For action lists, `max_retries=None` is automatically converted to `max_retries=1` to prevent getting stuck on the first action forever.

### Built-in Component Actions

These methods are available on every component and can be used as fallback actions:

| Action | Description |
|---|---|
| `start()` | Activate the component (lifecycle transition) |
| `stop()` | Deactivate the component |
| `restart(wait_time=None)` | Stop then start (optional delay between) |
| `reconfigure(new_config, keep_alive=False)` | Apply a new config (optionally while running) |
| `set_param(param_name, new_value, keep_alive=True)` | Change a single parameter |
| `set_params(params_names, new_values, keep_alive=True)` | Change multiple parameters |
| `broadcast_status()` | Publish current status (default fallback) |

### Default Behavior

If you don't configure any fallbacks, the component uses `broadcast_status()` as the default `on_fail` action with unlimited retries. This publishes the failure status so external systems (like a Monitor node) can observe and react.

## Events and Actions

Events allow components to react to data-driven conditions. An `Event` pairs a trigger condition with one or more `Action` callbacks.

### Defining Events

```python
from ros_sugar.core import Event, Action
from ros_sugar.io import Topic

# Topic-based: triggers whenever a message arrives
sensor_topic = Topic(name="/emergency", msg_type="Bool")
event = Event(event_condition=sensor_topic)

# Action-based: polls a method at a given rate
event = Event(
    event_condition=Action(my_component.check_battery),
    check_rate=1.0,  # Poll at 1 Hz
)
```

### Event Options

| Parameter | Default | Description |
|---|---|---|
| `on_change` | `False` | Only trigger when the value changes (not on every message) |
| `handle_once` | `False` | Only trigger once during the component's lifetime |
| `keep_event_delay` | `0.0` | Minimum delay (seconds) between consecutive triggers |
| `check_rate` | `None` | Poll rate (Hz) for action-based events |

### Wiring Events to Actions

Events and actions are connected at the Launcher level, not inside individual components:

```python
from ros_sugar.launch import Launcher

launcher = Launcher(namespace="robot")

# Define event + response actions
emergency_event = Event(event_condition=emergency_topic)
stop_action = Action(controller.stop)

launcher.add_pkg(
    components=[planner, controller],
    package_name="kompass",
    events_actions={emergency_event: [stop_action]},
)
```

This keeps components decoupled -- they don't need to know about each other's events.

## Putting It All Together

Here is a complete example showing a component with health status management, fallbacks, and events:

```python
from typing import Dict, Optional
from attrs import define, field
from ros_sugar.core import Action, Event
from ros_sugar.io import Topic
from ros_sugar.launch import Launcher

from kompass.components.component import Component
from kompass.components.defaults import TopicsKeys
from kompass.config import ComponentConfig, ComponentRunType


@define
class MyControllerConfig(ComponentConfig):
    max_consecutive_failures: int = field(default=5)


class MyController(Component):
    def __init__(self, component_name="my_controller", config=None, **kwargs):
        super().__init__(
            component_name=component_name,
            config=config or MyControllerConfig(),
            allowed_run_types=[ComponentRunType.TIMED],
            **kwargs,
        )

    def init_variables(self):
        self._failure_count = 0

    def _execution_step(self):
        location_cb = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        if location_cb is None or not location_cb.got_msg:
            self.health_status.set_fail_system(
                topic_names=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
            )
            return

        data = location_cb.get_output()
        success = self._run_control(data)

        if success:
            self._failure_count = 0
            self.health_status.set_healthy()
        else:
            self._failure_count += 1
            if self._failure_count > self.config.max_consecutive_failures:
                self.health_status.set_fail_algorithm(
                    algorithm_names=["MyControlAlgorithm"]
                )
            # Otherwise stay in current status -- transient failures
            # don't trigger fallbacks immediately

    def _run_control(self, data) -> bool:
        # Your control logic here
        ...


# --- Setup fallbacks and launch ---
controller = MyController()

# Recovery strategy: restart on algorithm failure, broadcast on system issues
controller.on_algorithm_fail(
    action=[
        Action(controller.restart),
        Action(controller.reconfigure, args=(MyControllerConfig(),)),
    ],
    max_retries=3,
)
controller.on_system_fail(
    action=Action(controller.broadcast_status),
    max_retries=None,
)
controller.on_giveup(
    action=Action(controller.stop),
    max_retries=1,
)

# Launch
launcher = Launcher(namespace="robot")
launcher.add_pkg(
    components=[controller],
    package_name="kompass",
)
launcher.bringup()
```

### Health Status Flow

```
_execution_step()
    │
    ├── Missing input? ──▶ set_fail_system(topic_names=[...])
    │                           │
    │                           ▼
    │                     fallback timer checks health_status
    │                           │
    │                           ├── on_system_fail defined? ──▶ execute action
    │                           └── no? ──▶ on_fail (broadcast_status)
    │
    ├── Algorithm failed? ──▶ set_fail_algorithm(algorithm_names=[...])
    │                           │
    │                           ▼
    │                     fallback timer checks health_status
    │                           │
    │                           ├── on_algorithm_fail defined? ──▶ execute action
    │                           │       │
    │                           │       ├── action returns True ──▶ set_healthy()
    │                           │       └── retries exhausted ──▶ next action or giveup
    │                           └── no? ──▶ on_fail
    │
    └── Success ──▶ set_healthy()
```
