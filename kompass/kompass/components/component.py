import time
import json
from typing import Dict, List, Optional, Union
from omegaconf import OmegaConf
from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.tf import TFListener, TFListenerConfig
from ros_sugar.supported_types import add_additional_datatypes

from ..callbacks import GenericCallback
from ..config import ComponentConfig, RobotConfig, BaseAttrs
from ..topic import RestrictedTopicsConfig, Topic, update_topics, get_all_msg_types
from .. import data_types

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


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
        inputs: Optional[Dict[str, Topic]] = None,
        outputs: Optional[Dict[str, Topic]] = None,
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

        self._inputs_keys: List[str] = list(inputs) if inputs else []
        self._outputs_keys: List[str] = list(outputs) if outputs else []

        outputs = list(outputs.values()) if outputs else []
        inputs = list(inputs.values()) if inputs else []

        self.__allowed_inputs: Optional[type[RestrictedTopicsConfig]] = allowed_inputs
        self.__allowed_outputs: Optional[type[RestrictedTopicsConfig]] = allowed_outputs

        self.config = config or ComponentConfig()

        super().__init__(
            component_name=component_name,
            config=self.config,
            config_file=config_file,
            inputs=inputs,
            outputs=outputs,
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

    # INPUTS/OUTPUTS AND CONFIGURATION
    def inputs(self, **kwargs):
        """
        Set component input streams (topics) : kwargs[topic_keyword]=Topic()
        """
        if self.__allowed_inputs:
            kwargs["allowed_config"] = self.__allowed_inputs
        # If input topics are not configured create a new class
        old_dict = (
            dict(zip(self._inputs_keys, self.in_topics))
            if hasattr(self, "in_topics")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        self.in_topics = list(topics_dict.values())
        self._inputs_keys = list(topics_dict)

    def outputs(self, **kwargs):
        """
        Set component output streams (topics)
        """
        if self.__allowed_outputs:
            kwargs["allowed_config"] = self.__allowed_outputs
        old_dict = (
            dict(zip(self._outputs_keys, self.out_topics))
            if hasattr(self, "out_topics")
            else {}
        )
        topics_dict = update_topics(old_dict, **kwargs)
        self.out_topics = list(topics_dict.values())
        self._outputs_keys = list(topics_dict)

    def configure(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """
        super().configure(config_file)
        # Set Inputs/Outputs
        raw_config = OmegaConf.load(config_file)

        inputs_config = OmegaConf.select(raw_config, f"{self.node_name}.inputs")
        for idx, key in enumerate(self._inputs_keys):
            if hasattr(inputs_config, key):
                self.in_topics[idx].from_yaml(
                    config_file, f"{self.node_name}.inputs.{key}"
                )

        outputs_config = OmegaConf.select(raw_config, f"{self.node_name}.outputs")
        for idx, key in enumerate(self._outputs_keys):
            if hasattr(outputs_config, key):
                self.out_topics[idx].from_yaml(
                    config_file, f"{self.node_name}.outputs.{key}"
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

    def in_topic_name(self, key: str) -> Optional[str]:
        return (
            self.in_topics[self._inputs_keys.index(key)].name
            if hasattr(self, "in_topics")
            else None
        )

    def out_topic_name(self, key: str) -> Optional[str]:
        return (
            self.out_topics[self._outputs_keys.index(key)].name
            if hasattr(self, "out_topics")
            else None
        )

    def get_in_topic(self, key: str) -> Optional[Topic]:
        return (
            self.in_topics[self._inputs_keys.index(key)]
            if hasattr(self, "in_topics")
            else None
        )

    def get_out_topic(self, key: str) -> Optional[Topic]:
        return (
            self.out_topics[self._outputs_keys.index(key)]
            if hasattr(self, "out_topics")
            else None
        )

    def get_callback(self, key: str) -> GenericCallback:
        try:
            return self.callbacks[self.in_topic_name(key)]
        except Exception as e:
            raise KeyError(
                f"Unknown key '{key}' for component '{self.node_name}' Inputs"
            ) from e

    def get_publisher(self, key: str) -> GenericCallback:
        try:
            return self.publishers_dict[self.out_topic_name(key)]
        except Exception as e:
            raise KeyError(
                f"Unknown key '{key}' for component '{self.node_name}' Outputs"
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
