"""System Launcher extending the base Launcher in [ROS Sugar](https://github.com/automatika-robotics/ros_sugar)"""

from typing import Dict, List, Optional

from .actions import Action
from .event import Event
from ros_sugar import Launcher as BaseLauncher
from ros_sugar import logger

from .components.component import Component
from .config import RobotConfig, RobotFrames


class Launcher(BaseLauncher):
    """
    System Launcher extending the base Launcher in [ROS Sugar](https://github.com/automatika-robotics/ros_sugar)

    Launcher is a class created to provide a more pythonic way to launch and configure ROS nodes.

    Launcher starts a pre-configured component or a set of components as ROS2 nodes using multi-threaded or multi-process execution. Launcher spawns an internal [Monitor](./monitor.md) node in a separate thread in both execution types.


    Launcher can also manage a set of Events-Actions through its internal Monitor node (See Monitor class).

    ## Available options:
    - Enable/Disable events monitoring
    - Enable/Disable multi-processing, if disabled the components are launched in threads
    - Select to activate one, many or all components on start (lifecycle nodes activation)

    Launcher forwards all the provided Events to its internal Monitor, when the Monitor detects an Event trigger it emits an InternalEvent back to the Launcher. Execution of the Action is done directly by the Launcher or a request is forwarded to the Monitor depending on the selected run method (multi-processes or multi-threaded).
    """

    def __init__(
        self,
        components: List[Component],
        events_actions: Dict[Event, Action] | None = None,
        namespace: str = "",
        config_file: str | None = None,
        enable_monitoring: bool = True,
        multi_processing: bool = True,
        activate_all_components_on_start: bool = False,
        components_to_activate_on_start: Optional[List[Component]] = None,
    ) -> None:
        """__init__.

        :param components:
        :type components: List[Component]
        :param events_actions:
        :type events_actions: Dict[Event, Action] | None
        :param namespace:
        :type namespace: str
        :param config_file:
        :type config_file: str | None
        :param enable_monitoring:
        :type enable_monitoring: bool
        :param multi_processing:
        :type multi_processing: bool
        :param activate_all_components_on_start:
        :type activate_all_components_on_start: bool
        :param components_to_activate_on_start:
        :type components_to_activate_on_start: Optional[List[Component]]
        :rtype: None
        """
        super().__init__(
            components=components,
            events_actions=events_actions,
            namespace=namespace,
            config_file=config_file,
            enable_monitoring=enable_monitoring,
            multi_processing=multi_processing,
            activate_all_components_on_start=activate_all_components_on_start,
            components_to_activate_on_start=components_to_activate_on_start,
            package_name="kompass",
            executable_entry_point="executable",
        )
        self.components = components

    def _update_components_launch_cmd_args(self):
        """_update_components_launch_cmd_args."""
        for component in self.components:
            component.update_cmd_args_list()

    @property
    def robot(self) -> Dict[str, RobotConfig]:
        """
        Getter of robot config for all components

        :return: Robot configuration
        :rtype: RobotConfig
        """
        robot_config_dict = {}
        for component in self.components:
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
        for component in self.components:
            if hasattr(component.config, "robot"):
                component.config.robot = config
        self._update_components_launch_cmd_args()

    @property
    def frames(self) -> Dict[str, RobotFrames]:
        """
        Getter of robot frames for all components

        :return: Robot frames configuration
        :rtype: RobotFrames
        """
        robot_config_dict = {}
        for component in self.components:
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
        for component in self.components:
            if hasattr(component.config, "frames"):
                component.config.frames = frames_config
        self._update_components_launch_cmd_args()

    def inputs(self, **kwargs):
        """
        Update input in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their inputs keys
            for component in self.components:
                if (
                    component._input_topics
                    and key in component._input_topics.asdict().keys()
                ):
                    # Update input value
                    component.inputs(**{key: value})
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key
        logger.info(
            f"The following components got updated for each provided input key: {components_keys_updated}"
        )
        self._update_components_launch_cmd_args()
        return

    def outputs(self, **kwargs):
        """
        Update output in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their output keys
            for component in self.components:
                if (
                    component._output_topics
                    and key in component._output_topics.asdict().keys()
                ):
                    # Update input value
                    component.outputs(**{key: value})
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key
        logger.info(
            f"The following components got updated for each provided output key: {components_keys_updated}"
        )
        self._update_components_launch_cmd_args()
        return
