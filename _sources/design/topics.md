# Topics

Kompass provides classes to configure a ROS2 topics as a Component Input/Output.

## Topic Configuration Class

- Configured using:

1. name: [str], ROS2 topic name.

2. msg_type: [Union[Kompass.data_types.SupportedType, str]], ROS2 message type, passed as a string or as a type.

3. qos_profile: [QoSConfig](../advanced/advanced_conf/qos.md), See usage in example below.

- Provides:

ros_msg_type: [type], Provides the ROS2 message type of the topic.

:::{note} Topic in Kompass only supports certain ROS2 message types found in [Supported Types](../advanced/types.md)
:::

## Usage Example

```python
from kompass.config import Topic

topic = Topic(name='/local_map', msg_type='OccupancyGrid')
```
