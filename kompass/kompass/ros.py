"""Inputs/Outputs configuration classes"""
from attrs import define
from ros_sugar.io import Publisher
from ros_sugar.io import Topic as BaseTopic
from ros_sugar.config import QoSConfig

__all__ = ["Publisher", "Topic", "QoSConfig"]


@define(kw_only=True)
class Topic(BaseTopic):
    """
    Overrides Topic from ros_sugar to add msg_type converter and validator from kompass
    """
    pass
