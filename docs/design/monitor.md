# Monitor

Monitor is a ROS2 Node (not Lifecycle) responsible of monitoring the status of the stack (rest of the running nodes) and managing requests/responses from the Orchestrator.



## Main Functionalities:
- Creates Subscribers to registered Events. The Monitor is configured to declare an InternalEvent back to the Launcher so the corresponding Action can be executed (see source implementation in launch_actions.py)


![Monitoring events](../_static/images/diagrams/events_actions_config.jpg)
![An Event Trigger](../_static/images/diagrams/events_actions_exec.jpg)

<!-- :::{figure-md} fig-monitor_event_exec

<img src="../_static/images/diagrams/events_actions_exec.jpg" alt="An Event Trigger" width="500px">
..
::: -->


- Creates Subscribers to all registered Components health status topics
- Creates clients for all components main services and main action servers
- Creates service clients to components reconfiguration services to handle actions sent from the Launcher


## Usage Example:

:::{caution} When launching the stack using the Launcher, you do not need to configure the Monitor. The Launcher will configure and launch its own Monitor node internally. The code below shows an example of this internal configuration
:::

```python
from kompass.monitor import Monitor
from kompass import event

components = ['planner', 'controller', 'driver']

emergency_stop_event = event.OnEqual(
    "emergency_stop",
    Topic(name="/emergency_stop", msg_type="Bool"),
    True,
    ("data"),
)

monitor_node = Monitor(componenets_names=components,
                        enable_health_status_monitoring=True,
                        events=[emergency_stop_event],
                        services_components=None,
                        action_servers_components=['planner'],
                        activate_on_start=True,
                    )

```
