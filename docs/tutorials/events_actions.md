# Fallbacks, Events & Actions: Design Guide

**Give your robot reflexes, not just callbacks.**

Kompass Event-Driven architecture acts as the nervous system of your robot. It allows you to define self-healing strategies through component **Fallbacks**, and to decouple monitoring (**Events**) from execution and responses (**Actions**), enabling you to define complex behaviors declaratively without writing brittle `if/else` chains inside your component code.

## 1. Fallbacks (Self-Healing)

Fallbacks are the **Self-Healing Mechanism** of a component. They define the specific set of [Actions](#actions-the-responses) to execute automatically when a failure is detected in the component's Health Status.

Instead of crashing or freezing when an error occurs, a Component can be configured to attempt intelligent recovery strategies:
* *Algorithm failed?* $\rightarrow$ **Switch** to a simpler backup.
* *Sensor data not available?* $\rightarrow$ Consume a different input source.
* *Plugin timeout?* $\rightarrow$ **Restart** the node.

### The Recovery Hierarchy

When a component reports a failure, the system checks for a registered fallback strategy in a specific order of priority:

1.  <span class="sd-text-primary" style="font-weight: bold;">System Failure</span> (`on_system_fail`): External context is broken (e.g., missing inputs).
2.  <span class="sd-text-danger" style="font-weight: bold;">Component Failure</span> (`on_component_fail`): Internal crash or hardware disconnect.
3.  <span class="sd-text-warning" style="font-weight: bold;">Algorithm Failure</span> (`on_algorithm_fail`): The logic ran but couldn't solve the problem.
4.  <span class="sd-text-secondary" style="font-weight: bold;">Catch-All</span> (`on_fail`): Generic safety net.

:::{button-link} fallbacks_simple.html
:color: primary
:ref-type: doc
:outline:
Add Fallbacks to your Recipes →
:::

## Events (The Triggers)

An Event monitors one or more **ROS2 Topics** and evaluates a logical condition in real-time. When the condition is met, the Event fires.

### Defining Events
Using the API's fluent, Pythonic syntax you can access message attributes and define triggers using standard comparison operators.

```python
from kompass.ros import Event, Topic
from ros_sugar.io import Topic

# Define Source (with optional timeout for data freshness)
battery = Topic(name="/battery_level", msg_type="Float32", data_timeout=0.5)

# Define Condition (Event)
# Triggers when battery drops below 20%
low_batt = Event(battery.msg.data < 20.0)

```

### Logic Gates & Sensor Fusion

You can compose complex behaviors by combining multiple topics using bitwise operators (`&`, `|`, `~`). The system will manage a synchronized **Blackboard** automatically to ensure all data used in the condition is fresh.

```python
# Trigger ONLY if Obstacle is Close (< 0.5m) AND Robot is in Autonomous Mode
# Uses '&' (AND), '|' (OR), '~' (NOT)
radar = Topic(name="/radar_dist", msg_type="Float32", data_timeout=0.2)
mode  = Topic(name="/mode", msg_type="String")

smart_stop = Event(
    event_condition=(lidar.msg.data < 0.5) & (mode.msg.data == "AUTO"),
    on_change=True # Only trigger on the transition from Safe -> Unsafe
)

```

### Configuration Options

* **`on_change=True`**: Edge Trigger. Fires only when the condition changes from `False` to `True`.
* **`handle_once=True`**: One-shot trigger (e.g., initialization).
* **`keep_event_delay=2.0`**: Debouncing. Prevents re-triggering for the specified seconds.

---

## Actions (The Responses)

Actions are the executable routines triggered by Events (reflexes) or Fallbacks (healing). They can be component methods, system utilities, or arbitrary Python functions.

### Dynamic Data Injection

Kompass Actions are **context-aware**. You can bind Action arguments directly to live Topic data. When the action triggers, the system fetches the latest message and injects it into the function automatically.

**Example: Context-Aware Logging**

```python
from kompass.ros import Action

# A standard python function
def log_status(voltage, mode):
    print(f"Alarm! Battery Level: {voltage}%, Mode: {mode}")

# The Action binds arguments to live topics
# At runtime, 'voltage' and 'mode' are replaced by real ROS messages
act_log = Action(
    method=log_status,
    args=(battery.msg.data, mode.msg.data)
)

```

```{tip}
Kompass includes a set of pre-defined component-level and system-level actions in `kompass.actions` module
```

```{seealso}
Dive deeper into the design concepts of [Events](https://automatika-robotics.github.io/sugarcoat/design/events.html), [Actions](https://automatika-robotics.github.io/sugarcoat/design/actions.html), [Fallbacks](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html) and the component's [Health Status](https://automatika-robotics.github.io/sugarcoat/design/status.html)
```
