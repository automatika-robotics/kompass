import time
from typing import Dict, List, Optional, Union
import json
from ros_sugar.component import BaseComponent
from ros_sugar.fallbacks import ComponentFallbacks
from ros_sugar.tf import TFListener, TFListenerConfig

from ..callbacks import GenericCallback
from ..event import Event
from ..config import BaseAttrs, ComponentConfig, RobotConfig
from ..topic import (
    Publisher,
    RestrictedTopicsConfig,
    Topic,
    create_topics_config,
    update_topics_config,
)


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
            my_component = Component(node_name='my_node', inputs=inputs)
        ```

    Components in Kompass can be defined to accept only restricted types of inputs/outputs to help lock the functionality of a specific Component implementation.
    - Example:
        ```python
            from kompass.topic import AllowedTopic, RestrictedTopicsConfig

            class AllowedInputs(RestrictedTopicsConfig):
                PLAN = AllowedTopic(key="input_1", types=["Path"])
                LOCATION = AllowedTopic(key="input_2", types=["Odometry", "PoseStamped", "Pose"])

            my_component = Component(node_name='my_node', allowed_inputs=AllowedInputs)
        ```
    """

    def __init__(
        self,
        node_name: str,
        config: Optional[ComponentConfig] = None,
        config_file: Optional[str] = None,
        inputs: Optional[BaseAttrs] = None,
        outputs: Optional[BaseAttrs] = None,
        fallbacks: Optional[ComponentFallbacks] = None,
        allowed_inputs: Optional[type[RestrictedTopicsConfig]] = None,
        allowed_outputs: Optional[type[RestrictedTopicsConfig]] = None,
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

        self.in_topics: Optional[BaseAttrs] = inputs
        self.out_topics: Optional[BaseAttrs] = outputs

        self.__allowed_inputs: Optional[type[RestrictedTopicsConfig]] = allowed_inputs
        self.__allowed_outputs: Optional[type[RestrictedTopicsConfig]] = allowed_outputs

        self.config = config or ComponentConfig()

        super().__init__(
            node_name=node_name,
            config=self.config,
            config_file=config_file,
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

    # CREATION AND DESTRUCTION METHODS
    def create_all_subscribers(self):
        """
        Creates all node subscribers
        """
        self.get_logger().info("STARTING ALL SUBSCRIBERS")
        self.callbacks: Dict[
            str, Union[GenericCallback, Dict[str, GenericCallback]]
        ] = {}
        for key, in_topic in self.in_topics.__dict__.items():
            if isinstance(in_topic, list):
                self.callbacks[key] = {
                    in_t.name: in_t.msg_type.callback(in_t, self.node_name)
                    for in_t in in_topic
                }
            else:
                # Get callback objects from input topics message types
                self.callbacks[key] = in_topic.msg_type.callback(
                    in_topic, self.node_name
                )

        # Creates subscriber and attaches it to input callback object
        for callback in self.callbacks.values():
            if isinstance(callback, Dict):
                for callback_item in callback.values():
                    callback_item.set_subscriber(self.add_ros_subscriber(callback_item))
            else:
                callback.set_subscriber(self.add_ros_subscriber(callback))
        super().create_all_subscribers()

    def create_all_publishers(self):
        """
        Creates all node publishers
        """
        self.get_logger().info("STARTING ALL PUBLISHERS")
        self.publishers_dict = {
            key: Publisher(output) for key, output in self.out_topics.__dict__.items()
        }

        for publisher in self.publishers_dict.values():
            publisher.set_node_name(self.node_name)
            # Set ROS publisher for each output publisher
            publisher = publisher.set_publisher(self.add_ros_publisher(publisher))

        super().create_all_publishers()

    def destroy_all_subscribers(self):
        """
        Destroys all node subscribers
        """
        self.get_logger().info("DESTROYING ALL SUBSCRIBERS")
        if hasattr(self, "callbacks"):
            for callback in self.callbacks.values():
                if callback._subscriber:
                    self.destroy_subscription(callback._subscriber)
        super().destroy_all_subscribers()

    def destroy_all_publishers(self):
        """
        Destroys all node publishers
        """
        self.get_logger().info("DESTROYING ALL PUBLISHERS")
        if hasattr(self, "publishers_dict"):
            for publisher in self.publishers_dict.values():
                if publisher._publisher:
                    self.destroy_publisher(publisher._publisher)
        super().destroy_all_publishers()

    # INPUTS/OUTPUTS AND CONFIGURATION
    def inputs(self, **kwargs):
        """
        Set component input streams (topics) : kwargs[topic_keyword]=Topic()
        """
        if self.__allowed_inputs:
            kwargs["allowed_config"] = self.__allowed_inputs
        # If input topics are not configured create a new class
        if not self.in_topics:
            new_topics = create_topics_config(f"{self.node_name}Inputs", **kwargs)
            self.in_topics = new_topics()
        else:
            self.in_topics = update_topics_config(self.in_topics, **kwargs)

    def outputs(self, **kwargs):
        """
        Set component output streams (topics)
        """
        if self.__allowed_outputs:
            kwargs["allowed_config"] = self.__allowed_outputs
        if not self.out_topics:
            new_topics = create_topics_config(f"{self.node_name}Inputs", **kwargs)
            self.out_topics = new_topics()
        else:
            self.out_topics = update_topics_config(self.out_topics, **kwargs)

    def configure(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """
        self.config.from_yaml(
            config_file, nested_root_name=self.node_name, get_common=True
        )
        # TODO: handle parsing topics directly with default config if it is not initialized - TODO later as it is not needed for Kompass pre-defined components
        if self.in_topics:
            self.in_topics.from_yaml(
                config_file, nested_root_name=f"{self.node_name}.inputs"
            )

        if self.out_topics:
            self.out_topics.from_yaml(
                config_file, nested_root_name=f"{self.node_name}.outputs"
            )

    def update_cmd_args_list(self):
        """
        Updates the launch_cmd_args property (To be called after a parameter update)
        """
        self.launch_cmd_args = [
            "--component_type",
            self.__class__.__name__,
            "--config_type",
            self.config.__class__.__name__,
            "--inputs",
            self.inputs_json,
            "--outputs",
            self.outputs_json,
            "--config",
            self.config_json,
            "--node_name",
            self.node_name,
        ]
        if self._config_file:
            self.launch_cmd_args = ["--config_file", self._config_file]

        if self.events:
            self.launch_cmd_args = [
                "--events",
                self.events_json,
                "--actions",
                self.actions_json,
            ]

    def add_ros_subscriber(self, callback: GenericCallback):
        """Creates a subscriber to be attached to an input message.

        :param msg:
        :type msg: Input
        :param callback:
        :type callback: GenericCallback
        """
        _subscriber = self.create_subscription(
            msg_type=callback.input_topic.msg_type._ros_type,
            topic=callback.input_topic.name,
            qos_profile=self.setup_qos(callback.input_topic.qos_profile),
            callback=callback.callback,
            callback_group=self.callback_group,
        )
        self.get_logger().debug(
            f"Started subscriber to topic: {callback.input_topic.name} of type {callback.input_topic.msg_type}"
        )
        return _subscriber

    def add_ros_publisher(self, publisher: Publisher):
        """
        Sets the publisher attribute of a component for a given Topic
        """
        qos_profile = self.setup_qos(publisher.output_topic.qos_profile)
        return self.create_publisher(
            publisher.output_topic.msg_type._ros_type,
            publisher.output_topic.name,
            qos_profile,
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

    # MAIN ACTION SERVER HELPER METHODS AND CALLBACKS
    def _replace_input_topic(
        self, topic_name: str, new_name: str, msg_type: str
    ) -> Optional[str]:
        """
        Replace an existing input topic

        :param topic_name: Existing input topic name
        :type topic_name: str
        :param new_name: New input topic name
        :type new_name: str
        :param msg_type: New message type
        :type msg_type: str

        :return: Error message if replacement failed, else None
        :rtype: str | None
        """
        error_msg = None
        # Normalize names
        normalized_topic_name = (
            topic_name[1:] if topic_name.startswith("/") else topic_name
        )

        for topic_key, callback in self.callbacks.items():
            if callback.input_topic.name == normalized_topic_name:
                try:
                    # Create new topic
                    new_topic = Topic(name=new_name, msg_type=msg_type)
                except Exception as e:
                    error_msg = f"Invalid topic parameters: {e}"
                    return error_msg

                # Destroy subscription if it is already activated and create new callback
                if callback._subscriber:
                    self.get_logger().info(
                        f"Destroying subscriber for topic '{topic_name}'"
                    )
                    self.destroy_subscription(callback._subscriber)

                    callback.set_subscriber(self.add_ros_subscriber(callback))

                    # Update value in in_topic register
                    setattr(self.in_topics, topic_key, new_topic)
                    return None

        error_msg = f"Input topic {topic_name} does not exist"
        return error_msg

    # END OF EVENTS/ACTIONS RELATED SERVICES

    # TODO move to utils
    @classmethod
    def _exclude_keys_from_dict(cls, input_dict: Dict, key_list: List) -> Dict:
        return {key: value for key, value in input_dict.items() if key not in key_list}

    @classmethod
    def _restrict_keys_from_dict(cls, input_dict: Dict, key_list: List) -> Dict:
        return {key: value for key, value in input_dict.items() if key in key_list}

    def got_all_inputs(
        self,
        inputs_to_check: Optional[List[str]] = None,
        inputs_to_exclude: Optional[List[str]] = None,
    ) -> bool:
        """
        Check if all input topics are being published

        :param inputs_to_check: List of input keys to check, defaults to None
        :type inputs_to_check: list[str] | None, optional

        :param inputs_to_exclude: List of input keys to exclude from check, defaults to None
        :type inputs_to_exclude: list[str] | None, optional

        :return: If all inputs are published
        :rtype: bool
        """

        if inputs_to_exclude:
            # If a non valid key is provided raise an error
            if not all(item in self.callbacks.keys() for item in inputs_to_exclude):
                raise ValueError(
                    f"Checking inputs is trying to exclude a non existing topic key(s): {inputs_to_exclude}. Available keys: {self.callbacks.keys()}"
                )
            inputs_dict_to_check = self._exclude_keys_from_dict(
                self.callbacks, inputs_to_exclude
            )

        elif inputs_to_check:
            # If a non valid key is provided raise an error
            if not all(item in self.callbacks.keys() for item in inputs_to_check):
                raise ValueError(
                    f"Checking inputs is trying to restrict check to non existing topic key(s): {inputs_to_check}. Available keys: {self.callbacks.keys()}"
                )
            inputs_dict_to_check = self._restrict_keys_from_dict(
                self.callbacks, inputs_to_check
            )

        else:
            inputs_dict_to_check = self.callbacks

        # Check if all callbacks of the selected topics got input messages
        for callback in inputs_dict_to_check.values():
            if not callback.got_msg:
                return False
        return True

    def get_missing_inputs(self) -> list[str]:
        """
        Get a list of input topic names not being published

        :return: List of unpublished topics
        :rtype: list[str]
        """
        unpublished_topics = []
        for callback in self.callbacks.values():
            if not callback.got_msg:
                unpublished_topics.append(callback.input_topic.name)
        return unpublished_topics

    def callbacks_inputs_check(
        self,
        inputs_to_check: Optional[List[str]] = None,
        inputs_to_execlude: Optional[List[str]] = None,
    ) -> bool:
        """
        Check that all node inputs are provided before executing callback
        """
        input_wait_time: float = 0.0
        while not self.got_all_inputs(inputs_to_check, inputs_to_execlude):
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

    # DUNDER METHODS
    def __matmul__(self, stream) -> Optional[Topic]:
        """
        @

        :param stream: _description_
        :type stream: _type_
        :raises TypeError: _description_
        :return: _description_
        :rtype: _type_
        """
        if isinstance(stream, str):
            if self.in_topics and stream in self.in_topics.asdict().keys():
                return self.in_topics.asdict()[stream]
            elif self.out_topics and stream in self.out_topics.asdict().keys():
                return self.out_topics.asdict()[stream]
            else:
                return None
        else:
            raise TypeError(
                "Component method '@' can only be used with a topic defined by a string key name or a Topic instance"
            )

    # TODO: Implement dunder for a more intuitive interface
    def __le__(self, stream: Topic):
        raise NotImplementedError

    # TO USE FOR ROS LAUNCH
    @property
    def inputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component inputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if self.in_topics:
            return self.in_topics.to_json()
        return "{}"

    @inputs_json.setter
    def inputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        if self.in_topics:
            self.in_topics.from_json(value)

    @property
    def outputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component outputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if self.out_topics:
            return self.out_topics.to_json()
        return "{}"

    @outputs_json.setter
    def outputs_json(self, value):
        if self.out_topics:
            self.out_topics.from_json(value)

    @property
    def events_json(self) -> Union[str, bytes]:
        """Getter of serialized component Events

        :return: Serialized Events List
        :rtype: Union[str, bytes]
        """
        events_list = []
        if self.events:
            for event in self.events:
                events_list.append(event.json)
        return json.dumps(events_list)

    @events_json.setter
    def events_json(self, events_serialized: Union[str, bytes]):
        """Setter of component events from JSON serialized events

        :param events_serialized: Serialized Events List
        :type events_serialized: Union[str, bytes]
        """
        self.events = Event.json_to_events_list(
            events_serialized,
            topic_template=Topic(name="dummy_template", msg_type="Bool"),
        )
