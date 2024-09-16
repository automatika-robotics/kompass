"""Inputs/Outputs configuration classes"""

from functools import partial
from typing import Any, List, Union, Optional

from attrs import define, field, make_class
from ros_sugar.io import Publisher
from ros_sugar.io import AllowedTopic as AllowedTopicBase
from ros_sugar.io import RestrictedTopicsConfig
from ros_sugar.io import Topic as BaseTopic
from ros_sugar.io import get_all_msg_types, get_msg_type

from . import data_types
from .config import BaseAttrs, BaseValidators

__all__ = [
    "Publisher",
    "AllowedTopic",
    "RestrictedTopicsConfig",
    "Topic",
    "create_topics_config",
    "update_topics_config",
]


def _get_msg_types(
    type_names: List[Union[data_types.SupportedType, str]],
) -> List[Union[data_types.SupportedType, str]]:
    """
    Gets a list of message types from supported data_types given a list of string names
    :param type_name: List of message names
    :type type_name: str

    :return: List of supported data types or None if not found
    :rtype: _type_
    """
    output_type_names = []
    for type_name in type_names:
        output = get_msg_type(type_name, msg_types_module=data_types)
        output_type_names.append(output)
    return output_type_names


@define(kw_only=True)
class Topic(BaseTopic):
    """
    Overrides Topic from ros_sugar to add msg_type converter and validator from kompass
    """

    msg_type: Union[data_types.SupportedType, str] = field(
        converter=partial(get_msg_type, msg_types_module=data_types),
        validator=BaseValidators.in_(
            partial(get_all_msg_types, msg_types_module=data_types)()
        ),
    )
    ros_msg_type: Any = field(init=False)

    @msg_type.validator
    def _update_ros_type(self, _, value):
        self.ros_msg_type = value._ros_type


@define(kw_only=True)
class AllowedTopic(AllowedTopicBase):
    """Overrides AllowedTopic from ros_sugar to add msg_type converter and validator from kompass"""

    types: List[Union[data_types.SupportedType, str]] = field(
        converter=_get_msg_types,
        validator=BaseValidators.list_contained_in(
            partial(get_all_msg_types, msg_types_module=data_types)()
        ),
    )


def _check_value_type_allowed_config(
    key, value, allowed_config: RestrictedTopicsConfig
) -> bool:
    """Checks if one value is a Topic and its message type is of allowed_config types at given key

    :param key: Value key name
    :type key: str
    :param value: Value to be checked
    :type value: _type_
    :param allowed_config: Allowed topics config
    :type allowed_config: RestrictedTopicsConfig
    :raises TypeError: If value is not a Topic
    :raises TypeError: If value message type is not of allowed types

    :return: Check passed
    :rtype: bool
    """
    # If it is a single topic
    if not isinstance(value, Topic):
        raise TypeError("Only Topic values are allowed for input/output config")

    if value.msg_type not in allowed_config.types(key):
        raise TypeError(
            f"Key {key} is of type {value.msg_type} can only be set to one of the following types: {allowed_config.types(key)}"
        )
    return True


def _get_allowed_number(
    key, value, allowed_config: RestrictedTopicsConfig
) -> Optional[int]:
    """
    Checks if a given (key, value) pair is of allowed config types

    :param key: Topic config key name
    :type key: str
    :param value: Topic object
    :type value: Any
    :param allowed_config: Dictionary with allowed config types
    :type allowed_config: dict

    :raises ValueError: If given key is not in allowed_config keys
    :raises TypeError: If given value is not a valid Topic
    :raises TypeError: If given value is a Topic with a nonvalid message type

    :return: If the given (key, values) are of allowed config types
    :rtype: bool
    """
    if key not in allowed_config.keys():
        raise ValueError(
            f"Invalid key '{key}'. Allowed keys are: {', '.join(allowed_config.keys())}"
        )

    # Check if all non optional topics are provided
    num_non_optional: int = allowed_config.required_number(key)
    if num_non_optional > 1:
        if not isinstance(value, list):
            raise ValueError(
                f"Key '{key}' requires '{num_non_optional}' topics but only '1' is provided"
            )
        elif len(value) < num_non_optional:
            raise ValueError(
                f"Key '{key}' requires '{num_non_optional}' mandatory topics but only '{len(value)}' are provided"
            )

    num_optional: int = allowed_config.optional_number(key)
    # Check if topics provided are within optional number
    if isinstance(value, list):
        if len(value) > num_optional + num_non_optional:
            raise ValueError(
                f"Key '{key}' requires '{num_non_optional}' topics and can take up to '{num_optional}' optional topics, but a total of '{len(value)}' (> '{num_non_optional + num_optional}') are provided"
            )
        for v in value:
            check_passed = _check_value_type_allowed_config(key, v, allowed_config)
            if not check_passed:
                return None
        return num_non_optional + num_optional

    if _check_value_type_allowed_config(key, value, allowed_config):
        return num_non_optional + num_optional
    return None


def create_topics_config(name: str, **kwargs) -> type[BaseAttrs]:
    """
    Creates topics config class out of given keywords and topics

    :param name: Config class name
    :type name: str

    :raises TypeError: If given kwargs values are not of type Topic

    :return: New topics configuration class
    :rtype: BaseAttrs
    """
    result_dict = {}
    if "allowed_config" in kwargs.keys():
        allowed_config = kwargs["allowed_config"]
        kwargs.pop("allowed_config")
        for key, value in kwargs.items():
            allowed_number_of_topics: Optional[int] = _get_allowed_number(
                key, value, allowed_config
            )
            if allowed_number_of_topics:
                if allowed_number_of_topics > 1 and not isinstance(value, list):
                    result_dict[str(key)] = [value]
                else:
                    result_dict[str(key)] = value
    else:
        for key, value in kwargs.items():
            if isinstance(value, Topic):
                result_dict[str(key)] = value
            else:
                raise TypeError("Input/Output config can only take Topic values")
    conf = {
        key: field(default=val, type=Topic)
        if not isinstance(val, list)
        else field(default=val, type=List[Topic])
        for key, val in result_dict.items()
    }
    attrs_class = make_class(name, conf, bases=(BaseAttrs,))
    return attrs_class


def update_topics_config(old_config_obj: BaseAttrs, **kwargs):
    """
    Updates the topics (inputs/outputs)

    :param old_config_obj: Existing topics config to be updated
    :type old_config_obj: BaseAttrs

    :return: Updated topics config
    :rtype: BaseAttrs
    """
    if "allowed_config" in kwargs.keys():
        allowed_config = kwargs["allowed_config"]
        kwargs.pop("allowed_config")
        for key, value in kwargs.items():
            if _get_allowed_number(key, value, allowed_config):
                setattr(old_config_obj, key, value)
    else:
        for key, value in kwargs.items():
            setattr(old_config_obj, key, value)
    return old_config_obj
