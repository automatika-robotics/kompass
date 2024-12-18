import time
from typing import Dict, List, Optional, Union, Tuple
from omegaconf import OmegaConf
from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.tf import TFListener, TFListenerConfig
from ros_sugar.supported_types import add_additional_datatypes
from ros_sugar.io import Publisher, get_all_msg_types, AllowedTopics

from ..callbacks import GenericCallback
from ..config import ComponentConfig, RobotConfig, ComponentRunType
from .ros import Topic, update_topics
from .. import data_types
from itertools import groupby

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


def _parse_from_topics_dict(
    topics_dict: Dict[str, Union[Topic, List[Topic]]],
) -> Tuple[List[str], List[Topic]]:
    """Parse the topics dictionary into a list of topics for compatibility with ros_sugar

    :param topics_dict: Dictionary of key names and Topics
    :type topics_dict: Dict[str, Union[Topic, List[Topic]]]
    :return: Parsed keys and values
    :rtype: Tuple[List[str], List[Topic]]
    """
    keys_list = []
    values_list = []

    for key, value in topics_dict.items():
        if isinstance(value, List):
            keys_list.extend([key] * len(value))
            values_list.extend(value)
        else:
            keys_list.append(key)
            values_list.append(value)
    return (keys_list, values_list)


def _parse_to_topics_dict(
    keys: List[str], values: List[Topic]
) -> Dict[str, Union[Topic, List[Topic]]]:
    """Parse the topics key names and values into a dictionary

    :param keys: Topics key names (can have duplicate keys if the same key accepts a set of topics)
    :type keys: List[str]
    :param values: Topics values
    :type values: List[Topic]
    :return: Parsed dictionary
    :rtype: Dict[str, Union[Topic, List[Topic]]]
    """
    grouped_dict: Dict[str, Union[Topic, List[Topic]]] = {}
    for key, group in groupby(zip(keys, values)):
        values = [v for _, v in group]
        # If multiple values, store as a list
        if len(values) == 1:
            grouped_dict[key[0]] = values[0]
        else:
            grouped_dict[key[0]] = values
    return grouped_dict


class Component(BaseComponent):
    """
    Component is the main execution entity in Kompass, every Component is equivalent to a ROS2 Lifecycle Node.

    A Component requires a set of input and/or output topic(s). All the functionalities implemented in ROS2 nodes can be implemented in the Component. Inputs/Outputs are attrs classes with the attribute name representing a unique input/output key name and the value equal to the Topic.
    - Example:
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

    Components in Kompass can be defined to accept only restricted types of inputs/outputs to help lock the functionality of a specific Component implementation.
    - Example:
        ```python
            from kompass.topic import AllowedTopic, RestrictedTopicsConfig

            class AllowedInputs(RestrictedTopicsConfig):
                PLAN = AllowedTopic(key="input_1", types=["Path"])
                LOCATION = AllowedTopic(key="input_2", types=["Odometry", "PoseStamped", "Pose"])

            my_component = Component(component_name='my_node', allowed_inputs=AllowedInputs)
        ```
    """

    def __init__(
        self,
        component_name: str,
        config: Optional[ComponentConfig] = None,
        config_file: Optional[str] = None,
        inputs: Optional[Dict[str, Union[Topic, List[Topic]]]] = None,
        outputs: Optional[Dict[str, Union[Topic, List[Topic]]]] = None,
        fallbacks: Optional[ComponentFallbacks] = None,
        allowed_inputs: Optional[Dict[str, AllowedTopics]] = None,
        allowed_outputs: Optional[Dict[str, AllowedTopics]] = None,
        allowed_run_types: Optional[List[ComponentRunType]] = None,
        callback_group=None,
        **kwargs,
    ):
        """
        Init a Component

        :param node_name: _description_
        :type node_name: str
        :param config: Configuration parameters, defaults to None
        :type config: Optional[ComponentConfig], optional
        :param config_file: Path to a configuration Yaml file, defaults to None
        :type config_file: Optional[str], optional
        :param inputs: Component Inputs, defaults to None
        :type inputs: Optional[BaseAttrs], optional
        :param outputs: Component Outputs, defaults to None
        :type outputs: Optional[BaseAttrs], optional
        :param fallbacks: Fallback actions to be executed by the component when a failure is detected, defaults to None
        :type fallbacks: Optional[ComponentFallbacks], optional
        :param allowed_inputs: Restricted component inputs, defaults to None
        :type allowed_inputs: Optional[RestrictedTopicsConfig], optional
        :param allowed_outputs: Restricted component outputs, defaults to None
        :type allowed_outputs: Optional[RestrictedTopicsConfig], optional
        :param callback_group: Main callback group used in the component, defaults to None
        :type callback_group: _type_, optional
        """
        (self._inputs_keys, inputs_list) = (
            _parse_from_topics_dict(inputs) if inputs else ([], [])
        )
        (self._outputs_keys, outputs_list) = (
            _parse_from_topics_dict(outputs) if outputs else ([], [])
        )

        self.__allowed_inputs: Optional[Dict[str, AllowedTopics]] = allowed_inputs
        self.__allowed_outputs: Optional[Dict[str, AllowedTopics]] = allowed_outputs

        # Allowed run types (defaults to all types)
        self.__allowed_run_types = allowed_run_types or ComponentRunType.values()

        self.config = config or ComponentConfig()

        super().__init__(
            component_name=component_name,
            config=self.config,
            config_file=config_file,
            inputs=inputs_list,
            outputs=outputs_list,
            callback_group=callback_group,
            enable_health_broadcast=True,
            fallbacks=fallbacks,
            **kwargs,
        )

    @property
    def robot(self) -> RobotConfig:
        """
        Getter of robot config

        :return: Robot configuration
        :rtype: RobotConfig
        """
        return self.config.robot

    @robot.setter
    def robot(self, config: RobotConfig):
        """
        Setter of robot configuration

        :param config: Robot configuration
        :type config: RobotConfig
        """
        self.config.robot = config

    @property
    def run_type(self) -> ComponentRunType:
        """
        Component run type: Timed, ActionServer or Event

        :return: Timed, ActionServer or Server
        :rtype: str
        """
        return self.config.run_type

    @run_type.setter
    def run_type(self, value: Union[ComponentRunType, str]):
        """Overrides property setter to restrict to implemented controller run types

        :param value: Run type
        :type value: Union[ComponentRunType, str]
        :raises ValueError: If run_type is unsupported
        """
        if value in self.__allowed_run_types:
            raise ValueError(
                f"Component {self.node_name} cannot have run_type '{value}', can only run as one of the following: {self.__allowed_run_types}"
            )

        self.config.run_type = value

    # INPUTS/OUTPUTS AND CONFIGURATION
    def inputs(self, **kwargs):
        """
        Set component input streams (topics) : kwargs[topic_keyword]=Topic()
        """
        if self.__allowed_inputs:
            kwargs["allowed_config"] = self.__allowed_inputs
        old_dict = (
            _parse_to_topics_dict(self._inputs_keys, self.in_topics)
            if hasattr(self, "in_topics")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        (self._inputs_keys, self.in_topics) = _parse_from_topics_dict(topics_dict)

    def outputs(self, **kwargs):
        """
        Set component output streams (topics)
        """
        if self.__allowed_outputs:
            kwargs["allowed_config"] = self.__allowed_outputs
        old_dict = (
            _parse_to_topics_dict(self._outputs_keys, self.out_topics)
            if hasattr(self, "out_topics")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        (self._outputs_keys, self.out_topics) = _parse_from_topics_dict(topics_dict)

    def config_from_yaml(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """
        super().config_from_yaml(config_file)
        # Set Inputs/Outputs
        raw_config = OmegaConf.load(config_file)

        inputs_config = OmegaConf.select(raw_config, f"{self.node_name}.inputs")
        for idx, key in enumerate(self._inputs_keys):
            if hasattr(inputs_config, str(key)):
                self.callbacks.pop(self.in_topic_name(key))
                self.in_topics[idx].from_yaml(
                    config_file, f"{self.node_name}.inputs.{key}"
                )
                self.callbacks[self.in_topics[idx].name] = self.in_topics[
                    idx
                ].msg_type.callback(self.in_topics[idx], node_name=self.node_name)

        outputs_config = OmegaConf.select(raw_config, f"{self.node_name}.outputs")
        for idx, key in enumerate(self._outputs_keys):
            if hasattr(outputs_config, str(key)):
                self.publishers_dict.pop(self.out_topic_name(key))
                self.out_topics[idx].from_yaml(
                    config_file, f"{self.node_name}.outputs.{key}"
                )
                self.publishers_dict[self.out_topics[idx].name] = Publisher(
                    self.out_topics[idx], node_name=self.node_name
                )

    @property
    def odom_tf_listener(self) -> Optional[TFListener]:
        """Gets a transform listener for Odomtery (from odom to world)

        :return:
        :rtype: TFListener
        """
        if not hasattr(self, "_odom_tf_listener"):
            if self.config.frames.odom != self.config.frames.world:
                self._odom_tf_listener = self.get_transform_listener(
                    src_frame=self.config.frames.odom,
                    goal_frame=self.config.frames.world,
                )
            else:
                self._odom_tf_listener = None
        return self._odom_tf_listener

    @property
    def scan_tf_listener(self) -> TFListener:
        """Gets a transform listener for LaserScan (from scan to robot base)

        :return:
        :rtype: TFListener
        """
        if not hasattr(self, "_scan_tf_listener"):
            self._scan_tf_listener = self.get_transform_listener(
                src_frame=self.config.frames.scan,
                goal_frame=self.config.frames.robot_base,
            )
        return self._scan_tf_listener

    @property
    def depth_tf_listener(self) -> TFListener:
        """Gets a transform listener for LaserScan (from scan to robot base)

        :return:
        :rtype: TFListener
        """
        if not hasattr(self, "_depth_tf_listener"):
            self._depth_tf_listener = self.get_transform_listener(
                src_frame=self.config.frames.depth,
                goal_frame=self.config.frames.robot_base,
            )
        return self._depth_tf_listener

    def get_transform_listener(self, src_frame: str, goal_frame: str) -> TFListener:
        """Gets a transform listener

        :param src_frame: Source coordinates frame
        :type src_frame: str
        :param goal_frame: Goal coordinates frame
        :type goal_frame: str

        :return: TF listener object
        :rtype: TFListener
        """
        # Configure transform source and goal frames
        tf_config = TFListenerConfig(
            source_frame=src_frame,
            goal_frame=goal_frame,
        )
        tf_listener: TFListener = self.create_tf_listener(tf_config)
        return tf_listener

    def in_topic_name(self, key: str) -> Union[str, List[str], None]:
        """Get the topic(s) name(s) corresponding to an input key name

        :param key: Input key name
        :type key: str
        :return: Topic(s) name(s)
        :rtype: Union[str, List[str], None]
        """
        if not hasattr(self, "in_topics"):
            return None
        if self._inputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._inputs_keys):
                if k == key:
                    names.append(self.in_topics[idx].name)
            return names
        return self.in_topics[self._inputs_keys.index(key)].name

    def out_topic_name(self, key: str) -> Union[str, List[str], None]:
        """Get the topic(s) name(s) corresponding to an output key name

        :param key: Output key name
        :type key: str
        :return: Topic(s) name(s)
        :rtype: Union[str, List[str], None]
        """
        if not hasattr(self, "out_topics"):
            return None
        if self._outputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._outputs_keys):
                if k == key:
                    names.append(self.out_topics[idx].name)
            return names
        return self.out_topics[self._outputs_keys.index(key)].name

    def get_in_topic(self, key: str) -> Union[Topic, List[Topic], None]:
        """Get the topic(s) corresponding to an input key name

        :param key: Input key name
        :type key: str
        :return: Topic(s)
        :rtype: Union[Topic, List[Topic], None]
        """
        if not hasattr(self, "in_topics"):
            return None
        if self._inputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._inputs_keys):
                if k == key:
                    names.append(self.in_topics[idx])
            return names
        return self.in_topics[self._inputs_keys.index(key)]

    def get_out_topic(self, key: str) -> Union[Topic, List[Topic], None]:
        """Get the topic(s) corresponding to an output key name

        :param key: Output key name
        :type key: str
        :return: Topic(s)
        :rtype: Union[Topic, List[Topic], None]
        """
        if not hasattr(self, "out_topics"):
            return None
        if self._outputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._outputs_keys):
                if k == key:
                    names.append(self.out_topics[idx])
            return names
        return self.out_topics[self._outputs_keys.index(key)]

    def get_callback(self, key: str, idx: int = 0) -> GenericCallback:
        """Get callback with given input key name

        :param key: Input key name
        :type key: str
        :param idx: Index of the input of multiple inputs correspond to the same key, defaults to 0
        :type idx: int, optional
        :raises KeyError: If key is not found in component inputs
        :return: Callback object
        :rtype: GenericCallback
        """
        try:
            topic_names: Union[str, List[str], None] = self.in_topic_name(key)
            if isinstance(topic_names, List):
                return self.callbacks[topic_names[idx]]
            return self.callbacks[topic_names]
        except Exception as e:
            raise KeyError(
                f"Unknown key '{key}' for component '{self.node_name}' Inputs"
            ) from e

    def get_publisher(self, key: str, idx: int = 0) -> Publisher:
        """Get publisher with given output key name

        :param key: Output topic key name
        :type key: str
        :param idx: Index of the output of multiple inputs correspond to the same key, defaults to 0
        :type idx: int, optional
        :raises KeyError: If key is not found in component outputs
        :return: Publisher object
        :rtype: Publisher
        """
        try:
            topic_names: Union[str, List[str], None] = self.out_topic_name(key)
            if isinstance(topic_names, List):
                return self.publishers_dict[topic_names[idx]]
            return self.publishers_dict[self.out_topic_name(key)]
        except Exception as e:
            raise KeyError(
                f"Unknown key '{key}' for component '{self.node_name}' Outputs: {self.publishers_dict}"
            ) from e

    def callbacks_inputs_check(
        self,
        inputs_to_check: Optional[List[str]] = None,
        inputs_to_exclude: Optional[List[str]] = None,
    ) -> bool:
        """
        Check that all node inputs are provided before executing callback
        """
        input_wait_time: float = 0.0
        while not self.got_all_inputs(inputs_to_check, inputs_to_exclude):
            unavailable_topics = self.get_missing_inputs()
            self.get_logger().warn(
                f"{self.node_name} inputs: '{unavailable_topics}' are not available. Waiting for maximum of {self.config.topic_subscription_timeout}seconds...",
                once=True,
            )
            self.health_status.set_fail_system(topic_names=unavailable_topics)
            input_wait_time += 1 / self.config.loop_rate
            time.sleep(1 / self.config.loop_rate)

            if input_wait_time >= self.config.topic_subscription_timeout:
                self.get_logger().error(
                    f"{self.node_name} inputs: '{unavailable_topics}' are not available. ABORTING"
                )
                return False
            # Inputs are all available -> execute function
        return True
