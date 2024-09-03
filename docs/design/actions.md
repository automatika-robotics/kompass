# Actions

Actions are methods or routines executed by a component or by the system monitor.

Actions can either be:
- Actions paired with Events: in this case the Action is executed by a system monitor when an event is detected (see more details in [Monitor](monitor.md), [Launcher](launcher.md))
- Actions paired with Fallbacks: in this case the Action is executed by a Component when a failure is detected

Actions are defined with:
- method to be executed (Callable)
- args: Arguments to be passed to the method when executing the action
- kwargs: Keyword arguments to be passed to the method when executing the action

## Usage Example:
```python
    from kompass.components.component import Component
    from kompass.config import ComponentConfig
    import logging

    def function():
        logging.info("I am executing an action!")

    my_component = Component(node_name='test_component')
    new_config = ComponentConfig(loop_rate=50.0)
    action1 = Action(method=my_component.start)
    action2 = Action(method=my_component.reconfigure, args=(new_config, True),)
    action3 = Action(method=function)
```

## Available Defined Actions:

Kompass comes with a set of pre-defined component level actions and system level actions

### Component-level Actions:
- stop: Deactivate the lifecycle Component
- start: Activate the lifecycle Component
- restart: stop then start
- reconfigure: Send new ComponentConfig class object to the Component
- update_parameter: Update the value of one parameter in the ComponentConfig
- update_parameters: Update the value of a set of parameters in the ComponentConfig

### System-level Actions:
- log: Log a message
- publish_message: Publish a ROS2 message to a given topic
- send_srv_request: Send a ROS2 service request
- send_action_goal: Send a ROS2 action goal

:::{tip} The previous pre-defined Actions are all keyword only
:::

### Usage Example:
```python
    from kompass.actions import Actions

    my_component = BaseComponent(node_name='test_component')
    action1 = Actions.start(component=my_component)
    action2 = Actions.log(msg="I am executing a cool action!")
```
