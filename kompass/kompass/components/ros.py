from typing import Optional, Dict, Union, List

from ros_sugar.io import AllowedTopics
from ros_sugar.io import Topic
from .defaults import TopicsKeys


def _check_value_type_allowed_config(
    key, value, allowed_config: Dict[str, AllowedTopics]
) -> bool:
    """Checks if one value is a Topic and its message type is of allowed_config types at given key

    :param key: Value key name
    :type key: str
    :param value: Value to be checked
    :type value: _type_
    :param allowed_config: Allowed topics config
    :type allowed_config: Dict[str, AllowedTopics]
    :raises TypeError: If value is not a Topic
    :raises TypeError: If value message type is not of allowed types

    :return: Check passed
    :rtype: bool
    """
    # If it is a single topic
    if not isinstance(value, Topic):
        raise TypeError("Only Topic values are allowed for input/output config")

    if value.msg_type not in allowed_config[key].types:
        raise TypeError(
            f"Key {key} is of type {value.msg_type} can only be set to one of the following types: {allowed_config[key].types}"
        )
    return True


def _get_allowed_number(
    key, value, allowed_config: Dict[str, AllowedTopics]
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
            f"Invalid key '{key}'. Allowed keys are: {allowed_config.keys()}"
        )

    # Check if all non optional topics are provided
    num_non_optional: int = allowed_config[key].number_required
    if num_non_optional > 1:
        if not isinstance(value, list):
            raise ValueError(
                f"Key '{key}' requires '{num_non_optional}' topics but only '1' is provided"
            )
        elif len(value) < num_non_optional:
            raise ValueError(
                f"Key '{key}' requires '{num_non_optional}' mandatory topics but only '{len(value)}' are provided"
            )

    num_optional: int = allowed_config[key].number_optional
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


def update_topics(
    topics_dict: Dict[TopicsKeys, Union[Topic, List[Topic], None]], **kwargs
) -> Dict[TopicsKeys, Union[Topic, List[Topic], None]]:
    """Update a dictionary of topics given keyword arguments

    :param topics_dict: Topic dictionary {topic_key_name: Topic}
    :type topics_dict: Dict[str, Topic]
    :return: Updated topics
    :rtype: Dict[str, Topic]
    """
    if "allowed_config" in kwargs.keys():
        allowed_config = kwargs["allowed_config"]
        kwargs.pop("allowed_config")
        for key, value in kwargs.items():
            if _get_allowed_number(TopicsKeys(key), value, allowed_config):
                topics_dict[TopicsKeys(key)] = value
    else:
        for key, value in kwargs.items():
            topics_dict[TopicsKeys(key)] = value
    return topics_dict
