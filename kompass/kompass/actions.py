"""Supported Actions in Kompass

```{note}
Refer to [Sugarcoat](https://github.com/automatika-robotics/sugarcoat) for related API documentation
```

Actions are methods or routines executed by a component or by the system monitor.

Actions can either be:
- Actions paired with Events: based on the Action type, the Action is executed either by the concerned Component or centrally by the system monitor when an event is detected
- Actions paired with Fallbacks: in this case the Action is executed by a Component when a failure is detected

Actions are defined with:
- method to be executed (Callable)
- args: Arguments to be passed to the method when executing the action
- kwargs: Keyword arguments to be passed to the method when executing the action

- Usage Example:
```python
    from kompass.components import Controller, ControllerConfig
    from kompass.actions import Action
    import logging

    def function():
        logging.info("I am executing an action!")

    my_controller = Controller(component_name='test_component')
    new_config = ControllerConfig(loop_rate=50.0)
    action1 = Action(method=my_component.start)
    action2 = Action(method=my_component.reconfigure, args=(new_config, True),)
    action3 = Action(method=function)
```
"""

from ros_sugar.actions import ComponentActions, Action, ComponentLaunchAction, LogInfo

__all__ = ["Action", "ComponentActions", "ComponentLaunchAction", "LogInfo"]
