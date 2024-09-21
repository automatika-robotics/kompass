# Inputs and Outputs

In General, Inputs/Outputs are [attrs](https://www.attrs.org/en/stable/) classes with the attribute name representing a unique input/output key name and the value equal to the [Topic](topics.md)

- *Usage Example:*
```python
    from kompass.topic import Topic, create_topics_config
    from kompass.components.component import Component

    NewTopicsClass = create_topics_config(
                        "ClassName",
                        input_1=Topic(name="/plan", msg_type="Path"),
                        input_2=Topic(name="/location", msg_type="Odometry"),
                    )
    inputs = NewTopicsClass()
    my_component = Component(component_name='my_node', inputs=inputs)
```

Components in Kompass are defined to accept only restricted types of inputs/outputs to help lock the functionality of a specific Component implementation.

- *Usage Example:*
```python
    from kompass.topic import AllowedTopic, RestrictedTopicsConfig

    class AllowedInputs(RestrictedTopicsConfig):
        PLAN = AllowedTopic(key="input_1", types=["Path"])
        LOCATION = AllowedTopic(key="input_2", types=["Odometry", "PoseStamped", "Pose"])

    my_component = Component(component_name='my_node', allowed_inputs=AllowedInputs)
```
