"""Inputs/Outputs configuration classes"""

from attrs import define
from ros_sugar.io import Publisher
from ros_sugar.core import Event, Action
from ros_sugar.io import Topic as BaseTopic, AllowedTopics
from ros_sugar.config import QoSConfig as QoSConfigBase
from ros_sugar.supported_types import add_additional_datatypes
from ros_sugar.io import get_all_msg_types
from .launcher import Launcher
from ros_sugar import logger
from ros_sugar import actions
from . import data_types


__all__ = [
    "Launcher",
    "Publisher",
    "Topic",
    "QoSConfig",
    "AllowedTopics",
    "Event",
    "Action",
    "logger",
    "actions",
]

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


@define(kw_only=True)
class QoSConfig(QoSConfigBase):
    """
    Class for quality of service (QoS) configuration in ROS2

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **history**
      - `int`, `[qos](https://docs.ros2.org/foxy/api/rclpy/api/qos.html).HistoryPolicy.KEEP_LAST`
      - Sample store type: ALL or LAST (up to N samples, configurable via the queue depth option)

    * - **queue_size**
      - `int`, `10`
      - Used only if the “history” policy was set to “keep last”

    * - **reliability**
      - `int`, `qos.ReliabilityPolicy.RELIABLE`
      - Level of reliability in delivering samples

    * - **durability**
      - `int`, `qos.DurabilityPolicy.VOLATILE`
      - Determines if the publisher will be persisting samples for “late-joining” subscriptions (Transit Local) or not (Volatile)
    ```
    """

    pass


@define(kw_only=True)
class Topic(BaseTopic):
    """
    Class for ROS topic configuration (name, type and QoS)

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **name**
      - `str`
      - Topic name

    * - **msg_type**
      - `type | str`
      - Topic message type, can be provided as a 'type' or the type name as a string

    * - **qos_profile**
      - `QoSConfig | Dict`, `QoSConfig()`
      - QoS (Quality of Service) configuration
    ```
    """

    pass
