# Inputs and Outputs

Components in Kompass are defined to accept only restricted types of inputs/outputs to help lock the functionality of a specific Component implementation. Each input/output is associated with a unique keyword name and is set to accept one or many of ROS2 message types. Additionally, the input/output keyword in the Component can define a category of Topics rather than a single one. To see an example of this check the [DriveManager](../navigation/driver.md) Component. In this component the input [sensor_data](../navigation/driver.md/#inputs) defines any proximity sensor input (LiDAR, Radar, etc.) and can optionally take up to 10 Topics of such types to fuse it internally during execution.

Configuring an input/output of a Component is very straightforward and can be done in one line in your Python script. Below is an example for configuring the previously mentioned DriveManager:

```python
    from kompass.components import DriveManager
    from kompass.topic import Topic

    driver = DriveManager(component_name="driver")

    # Configure an input
    driver.inputs(sensor_data=[Topic(name='/scan', msg_type='LaserScan'),
                               Topics(name='radar_data', msg_type='Float64')])

    # Configure an output
    driver.outputs(emergency_stop=Topic(name='alarm', msg_type='Bool'))

```

## Configure inputs/outputs for your custom component

Internally in the component, Inputs/Outputs are [attrs](https://www.attrs.org/en/stable/) classes with the attribute name representing a unique input/output key name and the value equal to the [Topic](topics.md). Each component is created with default inputs and default outputs and restricted with AllowedInputs and AllowedOutputs classes. Custom inputs/outputs classes can be created using `create_topics_config` method which returns an `attrs` class where each keyword argument corresponds to a class attribute of type `Topic`, as shown in the example below:


```python
    from kompass.topic import Topic, create_topics_config
    from kompass.components.component import Component

    # Create the Topics class
    NewTopicsClass = create_topics_config(
                        "MyInputsClass",
                        input_1=Topic(name="/plan", msg_type="Path"),
                        input_2=Topic(name="/location", msg_type="Odometry"),
                    )
    # Get an instance
    inputs = NewTopicsClass()

    # Set as the component inputs
    my_component = Component(component_name='my_component', inputs=inputs)
```

To restrict your custom component to accept only specific types of inputs/outputs, you can create your your allowed inputs and/or allowed outputs using `RestrictedTopicsConfig` [enum](https://docs.python.org/3/library/enum.html) class. Add an enum value of type `AllowedTopic` for each category of topics and set the key name, allowed  ROS2 message types and optionally the number of required and additional optional streams.

```{tip}
If the number of required and optional streams is not specified for an `AllowedTopic`, then the only one stream is required with no additional optional streams (default: `number_required=1, number_optional=0`)
```


```python
    from kompass.topic import AllowedTopic, RestrictedTopicsConfig
    from kompass.components.component import Component

    # Set the allowed inputs
    class AllowedInputs(RestrictedTopicsConfig):
        PLAN = AllowedTopic(key="input_1", types=["Path"])
        LOCATION = AllowedTopic(key="input_2", types=["Odometry", "PoseStamped", "Pose"], number_required=1,
        number_optional=3)

    # Create the Topics class
    NewTopicsClass = create_topics_config(
                        "MyInputsClass",
                        input_1=Topic(name="/plan", msg_type="Path"),
                        input_2=Topic(name="/location", msg_type="Odometry"),
                    )

    # Get an instance for the default inputs
    inputs = NewTopicsClass()

    # Can change input_2 since it takes other allowed types and upto 3 optional inputs
    inputs.input_2 = [Topic(name="/odom", msg_type="Odometry"), Topic(name="/pose_stamped", msg_type="PoseStamped")]

    my_component = Component(component_name='my_node', inputs=inputs, allowed_inputs=AllowedInputs)
```
