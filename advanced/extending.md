
# Extending the Components

## Configure inputs/outputs for your custom component

Inputs/Outputs in a Component are dictionaries with each key representing a unique input/output key name and the value equal to the [Topic](./advanced_conf/topics.md). Each component in Kompass stack is created with default inputs/outputs which can be modified, along with restrictions on the allowed message types for each input/output key and the number of obligatory/optional streams.

```{seealso}
All of Kompass stack inputs/outputs keys, default values and allowed values can be seen in detail in the [source code](https://github.com/automatika-robotics/kompass/blob/main/kompass/kompass/components/defaults.py).
```

Let's say you wish to extend the stack and create your own component that filters velocity commands from an autonomous controller, and at least one remote controller with up-to n connected remote controllers, you can design the component inputs/outputs and allowed values in a similar way as follows:



```python
    from typing import Union, List
    from kompass.ros import Topic, AllowedTopic

    # Import the parent component
    from kompass.components.component import Component
    # Import a helper method to safely update input/output values in a component
    from kompass.components.ros import update_topics

    # Default inputs: one autonomous command and 2 remote commands, all of type Twist
    default_inputs = Dict[str, Union[Topic, List[Topic]]] = {
    "autonomous_cmd": Topic(name="/cmd_autonom", msg_type="Twist"),
    "remote_cmd": [Topic(name="/cmd_remote_0", msg_type="Twist"), Topic(name="/cmd_remote_1", msg_type="Twist")],
    }

    # Default outputs: one final Twist command
    default_outputs = Dict[str, Topic] = {
    "robot_cmd": Topic(name="/cmd_vel", msg_type="Twist")
    }

    allowed_inputs: Dict[str, AllowedTopics] = {
    "autonomous_cmd": AllowedTopics(types=["Twist"]),
    "remote_cmd": AllowedTopics(
        types=["Twist"],
        number_required=1,  # One required remote command topic
        number_optional=10,     # Up to 10 optional topics for remote commands
    ),
    }

    allowed_outputs: Dict[str, AllowedTopics] = {
    "robot_cmd": AllowedTopics(types=["Twist"]),
    }

    # Write your custom component

    class MyComponent(Component):
        def __init__(
            self,
            component_name: str,
            config_file: Optional[str] = None,
            inputs: Optional[Dict[str, Topic]] = None,
            outputs: Optional[Dict[str, Topic]] = None,
            **kwargs,
        ) -> None:
            # Update defaults from custom topics if provided
            in_topics = (
                update_topics(driver_default_inputs, **inputs)
                if inputs
                else default_inputs
            )
            out_topics = (
                update_topics(driver_default_outputs, **outputs)
                if outputs
                else default_outputs
            )

            super().__init__(
                config_file=config_file,
                inputs=in_topics,
                outputs=out_topics,
                allowed_inputs=allowed_inputs,  # Pass allowed inputs to the component
                allowed_outputs=allowed_outputs,
                component_name=component_name,
                **kwargs,
            )

            # Write the rest of your functionalities ...
```

```{tip}
If the number of required and optional streams is not specified for an `AllowedTopic`, then only one stream is required with no additional optional streams (default: `number_required=1, number_optional=0`)
```
