# Topic QoS Configuration

**Fine-tune communication reliability and persistence.**

Quality of Service (QoS) profiles allow you to tune how data is handled between nodes. You can configure whether to prioritize speed (Best Effort) or data integrity (Reliable), and whether late-joining nodes should receive past data (Transient Local).


## Core Concepts

Understanding the trade-offs in ROS2 communication.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`verified` Reliability</span> - **Reliable vs. Best Effort.**
  <br>
  * **Reliable:** Guarantees delivery. Retries if packets are lost. (Good for services/control).
  * **Best Effort:** Fire and forget. Fast, but drops data if network is bad. (Good for sensor streams).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`save` Durability</span> - **Volatile vs. Transient Local.**
  <br>
  * **Volatile:** No history. Late joiners only see *new* data.
  * **Transient Local:** The publisher "saves" the last $N$ messages. Late joiners get the last known value immediately. (Good for maps/configuration).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`history` History</span> - **Keep Last vs. Keep All.**
  <br>
  * **Keep Last:** Only store a fixed queue of $N$ samples. Oldest are dropped.
  * **Keep All:** Store everything (subject to resource limits).

<br>

## Configuration Parameters

The `QoSConfig` class provides a wrapper to easily set these policies in Kompass.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`history` history</span> - **`qos.HistoryPolicy`**
  <br>Configuration of samples to store.
  <br>*Default:* `KEEP_LAST`

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`layers` queue_size</span> - **`int`**
  <br>The depth of the queue. Only honored if `history` is set to `KEEP_LAST`.
  <br>*Range:* [5, 100]
  <br>*Default:* `10`

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`verified_user` reliability</span> - **`qos.ReliabilityPolicy`**
  <br>Samples deliverance guarantee.
  <br>*Default:* `RELIABLE`

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`inventory_2` durability</span> - **`qos.DurabilityPolicy`**
  <br>Controls whether published samples are stored for late-joiners.
  <br>*Default:* `VOLATILE`


## Usage Example

```python
from kompass.ros import Topic, QoSConfig
from rclpy import qos

# 1. Define the Profile
# Example: A profile for a Map topic (Needs to be reliable and available to late joiners)
qos_map_profile = QoSConfig(
    history=qos.HistoryPolicy.KEEP_LAST,
    queue_size=1,
    reliability=qos.ReliabilityPolicy.RELIABLE,
    durability=qos.DurabilityPolicy.TRANSIENT_LOCAL
)

# 2. Apply to Topic
topic = Topic(
    name='/local_map',
    msg_type='OccupancyGrid',
    qos_profile=qos_map_profile
)

```


