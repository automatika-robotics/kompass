"""System Launcher extending the base Launcher in [Sugarcoat🍬](https://github.com/automatika-robotics/sugarcoat)"""

from typing import Dict, List, Optional, Union
from launch.action import Action as ROSLaunchAction

from ros_sugar.core import Event, Action
from ros_sugar.launch.launcher import Launcher as BaseLauncher
from ros_sugar import logger


from .components.component import Component
from .components.defaults import TopicsKeys
from .config import RobotConfig, RobotFrames


class Launcher(BaseLauncher):
    """
    System Launcher extending the base Launcher in [Sugarcoat🍬](https://github.com/automatika-robotics/sugarcoat)
    """

    def __init__(
        self,
        namespace: str = "",
        config_file: Optional[str] = None,
        activation_timeout: Optional[float] = None,
        robot_plugin: Optional[str] = None,
        **kwargs,
    ) -> None:
        """Initialize a launcher to manager components launch in ROS2

        :param namespace: ROS2 namespace for all the nodes, defaults to ""
        :type namespace: str, optional
        :param config_file: Path to configuration file, defaults to None
        :type config_file: str | None, optional
        :param activation_timeout: Timeout (seconds) for waiting on ROS2 nodes to come up for activation, defaults to None
        :type activation_timeout: float, optional
        :param robot_plugin: Name of the robot plugin package for compatibility handling, defaults to None
        :type robot_plugin: str, optional
        """
        super().__init__(
            namespace=namespace,
            config_file=config_file,
            activation_timeout=activation_timeout,
            robot_plugin=robot_plugin,
            **kwargs,
        )

    def kompass(
        self,
        components: List[Component],
        events_actions: Optional[
            Dict[
                Event,
                Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]],
            ]
        ] = None,
        multiprocessing: bool = True,
        activate_all_components_on_start: bool = True,
        components_to_activate_on_start: Optional[List[Component]] = None,
        ros_log_level: Optional[str] = None,
        rclpy_log_level: Optional[str] = None,
    ):
        """Adds Kompass components to the launcher

        :param components: Kompass components
        :type components: List[Component]
        :param events_actions: Events/actions dictionary, defaults to None
        :type events_actions: Dict[ Event, Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]] ] | None, optional
        :param multi_processing: Run components in multiple processes, defaults to True
        :type multi_processing: bool, optional
        :param activate_all_components_on_start: Activate all components on start, defaults to True
        :type activate_all_components_on_start: bool, optional
        :param components_to_activate_on_start: List of component to activate on start, defaults to None
        :type components_to_activate_on_start: Optional[List[Component]], optional
        :param ros_log_level: Components logging level, defaults to None
        :type ros_log_level: Optional[str], optional
        :param rclpy_log_level: RCLPY logging level, defaults to None
        :type rclpy_log_level: Optional[str], optional
        """
        self.add_pkg(
            components=components,
            package_name="kompass",
            executable_entry_point="executable",
            events_actions=events_actions,
            multiprocessing=multiprocessing,
            activate_all_components_on_start=activate_all_components_on_start,
            components_to_activate_on_start=components_to_activate_on_start,
            ros_log_level=ros_log_level,
            rclpy_log_level=rclpy_log_level,
        )

    @property
    def robot(self) -> Dict[str, RobotConfig]:
        """
        Getter of robot config for all components

        :return: Robot configuration
        :rtype: RobotConfig
        """
        robot_config_dict = {}
        for component in self._components:
            if hasattr(component.config, "robot"):
                robot_config_dict[component.node_name] = component.config.robot
        return robot_config_dict

    @robot.setter
    def robot(self, config: RobotConfig) -> None:
        """
        Setter of robot configuration for all components

        :param config: Robot configuration
        :type config: RobotConfig
        """
        for component in self._components:
            if hasattr(component.config, "robot"):
                component.config.robot = config

    @property
    def frames(self) -> Dict[str, RobotFrames]:
        """
        Getter of robot frames for all components

        :return: Robot frames configuration
        :rtype: RobotFrames
        """
        robot_config_dict = {}
        for component in self._components:
            if hasattr(component.config, "frames"):
                robot_config_dict[component.node_name] = component.config.frames
        return robot_config_dict

    @frames.setter
    def frames(self, frames_config: RobotFrames) -> None:
        """
        Setter of robot frames for all components

        :param frames_config: Robot frames configuration
        :type frames_config: RobotConfig
        """
        for component in self._components:
            if hasattr(component.config, "frames"):
                component.config.frames = frames_config

    def inputs(self, **kwargs):
        """
        Update input in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their inputs keys
            for idx, component in enumerate(self._components):
                pkg_name, _ = self._pkg_executable[idx]
                if (
                    pkg_name == "kompass"
                    and hasattr(component, "in_topics")
                    and TopicsKeys(key) in component._inputs_keys
                ):
                    # Update input value
                    component.inputs(**{key: value})
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key

        for key, items in components_keys_updated.items():
            logger.info(f"Input '{key}' updated for components: {items}")

    def outputs(self, **kwargs):
        """
        Update output in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their output keys
            for idx, component in enumerate(self._components):
                pkg_name, _ = self._pkg_executable[idx]
                if (
                    pkg_name == "kompass"
                    and hasattr(component, "out_topics")
                    and key in component._outputs_keys
                ):
                    # Update input value
                    component.outputs(**{key: value})
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key
        for key, items in components_keys_updated.items():
            logger.info(f"Output '{key}' updated for components: {items}")
