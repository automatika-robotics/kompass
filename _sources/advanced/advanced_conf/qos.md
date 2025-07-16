# QoS Configuration

KOMPASS wrapper for ROS2 QoS (Quality of Service) configuration.


`````{py:class} QoSConfig
:canonical: kompass.config.QoSConfig


````{py:attribute} history
:canonical: kompass.config.QoSConfig.history
:type: int
:value: >
   Configuration of samples to store (qos.HistoryPolicy)<br/>
   - Values:<br/>
   KEEP_LAST: only store up to N samples, configurable via the queue depth option.<br/>
   KEEP_ALL: store all samples, subject to the configured resource limits of the underlying middleware.
   - Default: qos.HistoryPolicy.KEEP_LAST


````

````{py:attribute} queue_size
:canonical: kompass.config.QoSConfig.queue_size
:type: int
:value: >
    Only honored if the “history” policy was set to “KEEP_LAST”<br/>
   - Values: in [5, 100]
   - Default: 10

````

````{py:attribute} reliability
:canonical: kompass.config.QoSConfig.reliability
:type: int
:value: >
   Samples deliverance guarantee (qos.ReliabilityPolicy)<br/>
   - Values:<br/>
   BEST_EFFORT: attempt to deliver samples, but may lose them if the network is not robust<br/>
   RELIABLE: guarantee that samples are delivered, may retry multiple time<br/>
   - Default: qos.ReliabilityPolicy.RELIABLE


````

````{py:attribute} durability
:canonical: kompass.config.QoSConfig.durability
:type: int
:value: >
    Controls whether or not, and how, published DDS samples are stored (qos.DurabilityPolicy)<br/>
   - Values:<br/>
    TRANSIENT_LOCAL: <br/>
    VOLATILE: <br/>
    UNKNOWN: <br/>
    SYSTEM_DEFAULT
   - Default: qos.DurabilityPolicy.VOLATILE

````

`````

:::{tip} Setup your QoSConfig and parse it into ROS2 by using 'setup_qos' method available in the Component
:::

## Usage Example

```python
from kompass.config import QoSConfig, Topic
from rclpy import qos

qos_conf = QoSConfig(
    history=qos.HistoryPolicy.KEEP_LAST,
    queue_size=20,
    reliability=qos.ReliabilityPolicy.BEST_EFFORT,
    durability=qos.DurabilityPolicy.TRANSIENT_LOCAL
)
topic = Topic(name='/local_map', msg_type='OccupancyGrid', qos_profile=qos_conf)
```
