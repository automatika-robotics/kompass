import time
from typing import Dict, List, Optional, Union
import json
from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.events import json_to_events_list
from ros_sugar.tf import TFListener, TFListenerConfig

from ..callbacks import GenericCallback
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

        self._input_topics: Optional[BaseAttrs] = inputs
        self._output_topics: Optional[BaseAttrs] = outputs

        self.__allowed_inputs: Optional[type[RestrictedTopicsConfig]] = allowed_inputs
        self.__allowed_outputs: Optional[type[RestrictedTopicsConfig]] = allowed_outputs

        self.config = config or ComponentConfig()

        super().__init__(
            component_name=component_name,
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
        for key, in_topic in self._input_topics.__dict__.items():
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
                    callback_item.set_subscriber(
                        self._add_ros_subscriber(callback_item)
                    )
            else:
                callback.set_subscriber(self._add_ros_subscriber(callback))

    def create_all_publishers(self):
        """
        Creates all node publishers
        """
        self.publishers_dict = {
            key: Publisher(output)
            for key, output in self._output_topics.__dict__.items()
        }
        super().create_all_publishers()

    # INPUTS/OUTPUTS AND CONFIGURATION
    def inputs(self, **kwargs):
        """
        Set component input streams (topics) : kwargs[topic_keyword]=Topic()
        """
        if self.__allowed_inputs:
            kwargs["allowed_config"] = self.__allowed_inputs
        # If input topics are not configured create a new class
        if not self._input_topics:
            new_topics = create_topics_config(f"{self.node_name}Inputs", **kwargs)
            self._input_topics = new_topics()
        else:
            self._input_topics = update_topics_config(self._input_topics, **kwargs)

    def outputs(self, **kwargs):
        """
        Set component output streams (topics)
        """
        if self.__allowed_outputs:
            kwargs["allowed_config"] = self.__allowed_outputs
        if not self._output_topics:
            new_topics = create_topics_config(f"{self.node_name}Inputs", **kwargs)
            self._output_topics = new_topics()
        else:
            self._output_topics = update_topics_config(self._output_topics, **kwargs)

    def configure(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """
        super().configure(config_file)
        # TODO: handle parsing topics directly with default config if it is not initialized - TODO later as it is not needed for Kompass pre-defined components
        if self._input_topics:
            self._input_topics.from_yaml(
                config_file, nested_root_name=f"{self.node_name}.inputs"
            )

        if self._output_topics:
            self._output_topics.from_yaml(
                config_file, nested_root_name=f"{self.node_name}.outputs"
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

    # TO USE FOR ROS LAUNCH
    @property
    def _inputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component inputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if self._input_topics:
            return self._input_topics.to_json()
        return "{}"

    @_inputs_json.setter
    def _inputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        if self._input_topics:
            self._input_topics.from_json(value)

    @property
    def _outputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component outputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if self._output_topics:
            return self._output_topics.to_json()
        return "{}"

    @_outputs_json.setter
    def _outputs_json(self, value):
        if self._output_topics:
            self._output_topics.from_json(value)

    @property
    def _events_json(self) -> Union[str, bytes]:
        """Getter of serialized component Events

        :return: Serialized Events List
        :rtype: Union[str, bytes]
        """
        events_list = []
        if self.events:
            for event in self.events:
                events_list.append(event.json)
        return json.dumps(events_list)

    @_events_json.setter
    def _events_json(self, events_serialized: Union[str, bytes]):
        """Setter of component events from JSON serialized events

        :param events_serialized: Serialized Events List
        :type events_serialized: Union[str, bytes]
        """
        self.events = json_to_events_list(
            events_serialized,
            topic_template=Topic(name="dummy_template", msg_type="Bool"),
        )
