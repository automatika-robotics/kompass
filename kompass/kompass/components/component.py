import time
import json
from typing import Dict, List, Optional, Union, Tuple
from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.tf import TFListener, TFListenerConfig
from ros_sugar.io import Publisher, AllowedTopics

from ..callbacks import GenericCallback
from ..config import ComponentConfig, RobotConfig, ComponentRunType
from .ros import Topic, update_topics
from itertools import groupby
from .defaults import TopicsKeys
from kompass_core import set_logging_level


def _parse_from_topics_dict(
    topics_dict: Dict[TopicsKeys, Union[Topic, List[Topic], None]],
) -> Tuple[List[TopicsKeys], List[Optional[Topic]]]:
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
    keys: List[TopicsKeys], values: List[Optional[Topic]]
) -> Dict[TopicsKeys, Union[Topic, None, List[Topic]]]:
    """Parse the topics key names and values into a dictionary

    :param keys: Topics key names (can have duplicate keys if the same key accepts a set of topics)
    :type keys: List[str]
    :param values: Topics values
    :type values: List[Topic]
    :return: Parsed dictionary
    :rtype: Dict[str, Union[Topic, List[Topic]]]
    """
    grouped_dict: Dict[TopicsKeys, Union[Topic, None, List[Topic]]] = {}
    for key, group in groupby(zip(keys, values)):
        values = [v for _, v in group]
        # If multiple values, store as a list
        if len(values) == 1:
            if not grouped_dict.get(TopicsKeys(key[0])):
                grouped_dict[TopicsKeys(key[0])] = values[0]
            elif isinstance(grouped_dict[TopicsKeys(key[0])], list):
                # Already has a value -> store as a list
                grouped_dict[TopicsKeys(key[0])].append(values[0])
            else:
                # Already has a value -> store as a list
                grouped_dict[TopicsKeys(key[0])] = [
                    grouped_dict[TopicsKeys(key[0])],
                    values[0],
                ]
        else:
            if not grouped_dict.get(TopicsKeys(key[0])):
                grouped_dict[TopicsKeys(key[0])] = [val for val in values if val]
            elif isinstance(grouped_dict[TopicsKeys(key[0])], list):
                # Already has a list -> append values
                grouped_dict[TopicsKeys(key[0])].extend([val for val in values if val])
            else:
                # Already has a value -> store as a list
                grouped_dict[TopicsKeys(key[0])] = [
                    grouped_dict[TopicsKeys(key[0])],
                    *[val for val in values if val],
                ]
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
        inputs: Optional[Dict[TopicsKeys, Union[Topic, List[Topic], None]]] = None,
        outputs: Optional[Dict[TopicsKeys, Union[Topic, List[Topic], None]]] = None,
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
        :param config_file: Path to a configuration file (yaml, json, toml), defaults to None
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
        (self._inputs_keys, self._inputs_list) = (
            _parse_from_topics_dict(inputs) if inputs else ([], [])
        )
        (self._outputs_keys, self._outputs_list) = (
            _parse_from_topics_dict(outputs) if outputs else ([], [])
        )

        self.__allowed_inputs: Optional[Dict[str, AllowedTopics]] = allowed_inputs
        self.__allowed_outputs: Optional[Dict[str, AllowedTopics]] = allowed_outputs

        # Allowed run types (defaults to all types)
        if allowed_run_types:
            self.__allowed_run_types = allowed_run_types + [
                runtype.value for runtype in allowed_run_types
            ]
        else:
            self.__allowed_run_types = ComponentRunType.values() + list(
                ComponentRunType
            )

        # Remove None values from topics before sending to sugarcoat
        inputs_list: List[Topic] = [topic for topic in self._inputs_list if topic]
        outputs_list: List[Topic] = [topic for topic in self._outputs_list if topic]

        self.config: ComponentConfig = config or ComponentConfig()

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

    def custom_on_configure(self):
        """
        Component custom configuration method to set the core debug level
        """
        super().custom_on_configure()
        # Set logging level for kompass_core
        set_logging_level(self.config.core_log_level)

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
        return self.config._run_type

    @run_type.setter
    def run_type(self, value: Union[ComponentRunType, str]):
        """Overrides property setter to restrict to implemented controller run types

        :param value: Run type
        :type value: Union[ComponentRunType, str]
        :raises ValueError: If run_type is unsupported
        """
        if value not in self.__allowed_run_types:
            raise ValueError(
                f"Component {self.node_name} cannot have run_type '{value}', can only run as one of the following: {self.__allowed_run_types}"
            )

        self.config._run_type = value

    @property
    def inputs_keys(self) -> List[TopicsKeys]:
        """
        Getter of component inputs key names that should be used in the inputs dictionary
        """
        return self._inputs_keys

    @property
    def outputs_keys(self) -> List[TopicsKeys]:
        """
        Getter of component outputs key names that should be used in the inputs dictionary
        """
        return self._outputs_keys

    # INPUTS/OUTPUTS AND CONFIGURATION
    def inputs(self, **kwargs):
        """
        Set component input streams (topics) : kwargs[topic_keyword]=Topic()
        """
        if self.__allowed_inputs:
            kwargs["allowed_config"] = self.__allowed_inputs
        old_dict = (
            _parse_to_topics_dict(self._inputs_keys, self._inputs_list)
            if hasattr(self, "_inputs_list")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        (self._inputs_keys, _inputs) = _parse_from_topics_dict(topics_dict)
        # Update the list containing all the Topics (without None values)
        self._inputs_list = self._reparse_inputs_callbacks(_inputs)
        self.in_topics = [topic for topic in self._inputs_list if topic]

    def outputs(self, **kwargs):
        """
        Set component output streams (topics)
        """
        if self.__allowed_outputs:
            kwargs["allowed_config"] = self.__allowed_outputs
        old_dict = (
            _parse_to_topics_dict(self._outputs_keys, self._outputs_list)
            if hasattr(self, "_outputs_list")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        (self._outputs_keys, _outputs) = _parse_from_topics_dict(topics_dict)
        self._outputs_list = self._reparse_outputs_converts(_outputs)
        self.out_topics = [topic for topic in self._outputs_list if topic]

    def __configure_input_from_file(
        self, idx: int, key: TopicsKeys, config_file: str
    ) -> None:
        """Configure component input topic from config file

        :param idx: Input index
        :type idx: int
        :param key: Input key name
        :type key: TopicsKeys
        :param config_file: Path to file
        :type config_file: str
        """
        topic_updated: bool = False
        topic_name = self.in_topic_name(key)
        # If the key does not correspond to a None value -> replace callback
        if isinstance(topic_name, str):
            topic_updated = self.in_topics[idx].from_file(
                config_file, f"{self.node_name}.inputs.{key}"
            )
            if topic_updated:
                self.callbacks.pop(topic_name)
        # TODO handle List case
        else:
            # Create a dummy topic to parse the value from file
            dummy_topic = Topic(name="dummy", msg_type="String")
            topic_updated = dummy_topic.from_file(
                config_file, f"{self.node_name}.inputs.{key}"
            )
            # If the key correspond to a None value -> create callback
            if topic_updated:
                self.in_topics.insert(
                    idx,
                    dummy_topic,
                )
        if topic_updated:
            self.callbacks[self.in_topics[idx].name] = self.in_topics[
                idx
            ].msg_type.callback(self.in_topics[idx], node_name=self.node_name)

    def __configure_output_from_file(
        self, idx: int, key: TopicsKeys, config_file: str
    ) -> None:
        """Configure component output topic from config file

        :param idx: Output index
        :type idx: int
        :param key: Output key name
        :type key: TopicsKeys
        :param config_file: Path to file
        :type config_file: str
        """
        # if hasattr(outputs_config, str(key)):
        topic_updated: bool = False
        topic_name = self.out_topic_name(key)
        if isinstance(topic_name, str):
            topic_updated = self.out_topics[idx].from_file(
                config_file, f"{self.node_name}.outputs.{key}"
            )
            if topic_updated:
                self.publishers_dict.pop(topic_name)
        else:
            dummy_topic = Topic(name="dummy", msg_type="String")
            topic_updated: bool = dummy_topic.from_file(
                config_file, f"{self.node_name}.outputs.{key}"
            )
            if topic_updated:
                self.out_topics.insert(
                    idx,
                    dummy_topic,
                )
        if topic_updated:
            self.publishers_dict[self.out_topics[idx].name] = Publisher(
                self.out_topics[idx], node_name=self.node_name
            )

    def config_from_file(self, config_file: str):
        """
        Configure component from file

        :param config_file: Path to file (yaml, json, toml)
        :type config_file: str
        """
        super().config_from_file(config_file)
        # Set Inputs/Outputs
        for idx, key in enumerate(self._inputs_keys):
            self.__configure_input_from_file(idx, key, config_file)

        for idx, key in enumerate(self._outputs_keys):
            self.__configure_output_from_file(idx, key, config_file)

    @property
    def odom_tf_listener(self) -> Optional[TFListener]:
        """Gets a transform listener for Odometry (from odom to world)

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

    @property
    def pc_tf_listener(self) -> TFListener:
        """Gets a transform listener for LaserScan (from scan to robot base)

        :return:
        :rtype: TFListener
        """
        if not hasattr(self, "_pc_tf_listener"):
            self._pc_tf_listener = self.get_transform_listener(
                src_frame=self.config.frames.point_cloud,
                goal_frame=self.config.frames.robot_base,
            )
        return self._pc_tf_listener

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

    def in_topic_name(self, key: Union[str, TopicsKeys]) -> Union[str, List[str], None]:
        """Get the topic(s) name(s) corresponding to an input key name

        :param key: Input key name
        :type key: str
        :return: Topic(s) name(s)
        :rtype: Union[str, List[str], None]
        """
        key = key if isinstance(key, TopicsKeys) else TopicsKeys(key)
        if self._inputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._inputs_keys):
                if k == key:
                    if self._inputs_list[idx]:
                        names.append(self._inputs_list[idx].name)
            return names
        return (
            self._inputs_list[self._inputs_keys.index(key)].name
            if self._inputs_list[self._inputs_keys.index(key)]
            else None
        )

    def out_topic_name(
        self, key: Union[str, TopicsKeys]
    ) -> Union[str, List[str], None]:
        """Get the topic(s) name(s) corresponding to an output key name

        :param key: Output key name
        :type key: str
        :return: Topic(s) name(s)
        :rtype: Union[str, List[str], None]
        """
        key = key if isinstance(key, TopicsKeys) else TopicsKeys(key)
        if self._outputs_keys.count(key) > 1:
            names: List[str] = []
            for idx, k in enumerate(self._outputs_keys):
                if k == key:
                    if self._outputs_list[idx]:
                        names.append(self._outputs_list[idx].name)
            return names
        return (
            self._outputs_list[self._outputs_keys.index(key)].name
            if self._outputs_list[self._outputs_keys.index(key)]
            else None
        )

    def get_in_topic(
        self, key: Union[str, TopicsKeys]
    ) -> Union[Topic, List[Topic], None]:
        """Get the topic(s) corresponding to an input key name

        :param key: Input key name
        :type key: str
        :return: Topic(s)
        :rtype: Union[Topic, List[Topic], None]
        """
        key = key if isinstance(key, TopicsKeys) else TopicsKeys(key)
        if self._inputs_keys.count(key) > 1:
            topics: List[Topic] = []
            for idx, k in enumerate(self._inputs_keys):
                if k == key and self._inputs_list[idx]:
                    topics.append(self._inputs_list[idx])
            return topics
        return self._inputs_list[self._inputs_keys.index(key)]

    def get_out_topic(
        self, key: Union[str, TopicsKeys]
    ) -> Union[Topic, List[Topic], None]:
        """Get the topic(s) corresponding to an output key name

        :param key: Output key name
        :type key: str
        :return: Topic(s)
        :rtype: Union[Topic, List[Topic], None]
        """
        key = key if isinstance(key, TopicsKeys) else TopicsKeys(key)
        if self._outputs_keys.count(key) > 1:
            topics: List[Topic] = []
            for idx, k in enumerate(self._outputs_keys):
                if k == key and self._outputs_list[idx]:
                    topics.append(self._outputs_list[idx])
            return topics
        return self._outputs_list[self._outputs_keys.index(key)]

    def get_callback(
        self, key: Union[str, TopicsKeys], idx: int = 0
    ) -> Optional[GenericCallback]:
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
            return self.callbacks[topic_names] if topic_names else None
        except Exception as e:
            raise KeyError(
                f"Unknown input '{key}' for component '{self.node_name}'"
            ) from e

    def get_publisher(self, key: Union[str, TopicsKeys], idx: int = 0) -> Publisher:
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
            return self.publishers_dict[topic_names]
        except Exception as e:
            raise KeyError(
                f"Unknown output '{key}' for component '{self.node_name}'"
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

    @property
    def _inputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Override Serialize component inputs to json to handle None values

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if not hasattr(self, "in_topics"):
            return "[]"
        return json.dumps([
            (topic_key.value, topic.to_json()) if topic else (topic_key.value, None)
            for topic_key, topic in zip(self._inputs_keys, self._inputs_list)
        ])

    @_inputs_json.setter
    def _inputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        topics_and_keys = json.loads(value)
        self._inputs_keys = [TopicsKeys(pair[0]) for pair in topics_and_keys]
        inputs = [
            Topic(**json.loads(pair[1])) if pair[1] else None
            for pair in topics_and_keys
        ]
        self._inputs_list = self._reparse_inputs_callbacks(inputs)
        self.in_topics = [topic for topic in self._inputs_list if topic]
        self.callbacks = {
            input.name: input.msg_type.callback(input, node_name=self.node_name)
            for input in self.in_topics
        }

    @property
    def _outputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component inputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if not hasattr(self, "out_topics"):
            return "[]"
        return json.dumps([
            (topic_key.value, topic.to_json()) if topic else (topic_key.value, None)
            for topic_key, topic in zip(self._outputs_keys, self._outputs_list)
        ])

    @_outputs_json.setter
    def _outputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        topics_and_keys = json.loads(value)
        self._outputs_keys = [TopicsKeys(pair[0]) for pair in topics_and_keys]
        outputs = [
            Topic(**json.loads(pair[1])) if pair[1] else None
            for pair in topics_and_keys
        ]
        self._outputs_list = self._reparse_outputs_converts(outputs)
        self.out_topics = [topic for topic in self._outputs_list if topic]
        self.publishers_dict = {
            output.name: Publisher(output, node_name=self.node_name)
            for output in self.out_topics
        }
