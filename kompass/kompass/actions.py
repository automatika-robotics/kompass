"""Supported Actions in Kompass

```{note}
Refer to [ROS Sugar](https://github.com/automatika-robotics/ros_sugar) for related API documentation
```

Actions are methods or routines executed by a component or by the system monitor.

Actions can either be:
- Actions paired with Events: in this case the Action is executed by a system monitor when an event is detected (see more details in [Monitor](monitor.md), [Launcher](launcher.md))
- Actions paired with Fallbacks: in this case the Action is executed by a Component when a failure is detected

Actions are defined with:
- method to be executed (Callable)
- args: Arguments to be passed to the method when executing the action
- kwargs: Keyword arguments to be passed to the method when executing the action

- Usage Example:
```python
    from kompass.components.component import Component
    from kompass.config import ComponentConfig
    import logging

    def function():
        logging.info("I am executing an action!")

    my_component = Component(component_name='test_component')
    new_config = ComponentConfig(loop_rate=50.0)
    action1 = Action(method=my_component.start)
    action2 = Action(method=my_component.reconfigure, args=(new_config, True),)
    action3 = Action(method=function)
```
"""

from ros_sugar.actions import ComponentActions, Action, ComponentLaunchAction, LogInfo

__all__ = ["Action", "ComponentActions", "ComponentLaunchAction", "LogInfo"]
