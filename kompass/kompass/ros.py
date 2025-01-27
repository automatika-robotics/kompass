"""Inputs/Outputs configuration classes"""

from attrs import define
from ros_sugar.io import Publisher
from ros_sugar.io import Topic as BaseTopic
from ros_sugar.config import QoSConfig
from ros_sugar.supported_types import add_additional_datatypes
from ros_sugar.io import get_all_msg_types
from . import data_types


__all__ = ["Publisher", "Topic", "QoSConfig"]

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


@define(kw_only=True)
class Topic(BaseTopic):
    """
    Overrides Topic from ros_sugar to add msg_type converter and validator from kompass
    """

    pass
