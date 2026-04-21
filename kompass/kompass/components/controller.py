from typing import Optional, Union, List, Dict, Any
import time
from attrs import define, field
from queue import Queue, Empty
import numpy as np
from ..utils import StrEnum

from rclpy.logging import get_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ROS MSGS
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped

# KOMPASS
from kompass_core.models import Robot, RobotCtrlLimits, RobotState
from kompass_core.datatypes import LaserScanData, PointCloudData, Bbox2D
from kompass_core.utils.geometry import from_euler_to_quaternion
from kompass_core.control import (
    ControlClasses,
    ControlConfigClasses,
    ControllersID,
    ControllerType,
)

# KOMPASS MSGS/SRVS/ACTIONS
import kompass_interfaces.msg as kompass_msgs
from kompass_interfaces.srv import StopVisionTracking
from kompass_interfaces.action import ControlPath, TrackVisionTarget

from ..config import BaseValidators, ComponentConfig, ComponentRunType

# KOMPASS ROS
from .ros import (
    Topic,
    update_topics,
)
from ..utils import component_action
from ..callbacks import PointCloudCallback

# KOMPASS MSGS/SRVS/ACTIONS
from .component import Component, TFListener
from .utils import init_twist_array_msg
from .defaults import (
    controller_allowed_inputs,
    controller_allowed_outputs,
    controller_default_inputs,
    controller_default_outputs,
    TopicsKeys,
)


class CmdPublishType(StrEnum):
    """
    Control command publishing method:

    ```{list-table}
    :widths: 20 70
    :header-rows: 1

    * - Value
      - Description

    * - **TWIST_SEQUENCE (Literal "Sequence")**
      - the controller publishes a Twist message in the same thread running the control algorithm. If a series of commands is computed (up to the control horizon), the controller publishes the commands one by one before running the control algorithm again

    * - **TWIST_PARALLEL (Literal "Parallel")**
      - the controller handles publishing a Twist message in a new thread. If a series of commands is computed (up to the control horizon), the controller publishes the commands one by one in parallel while running the control algorithm again

    * - **TWIST_ARRAY (Literal "Array")**
      - the controller publishes a TwistArray msg of all the computed control commands (up to the control horizon)
    ```

    """

    TWIST_SEQUENCE = "Sequence"
    TWIST_PARALLEL = "Parallel"
    TWIST_ARRAY = "Array"


class ControllerMode(StrEnum):
    PATH_FOLLOWER = "path_follower"
    VISION_FOLLOWER = "vision_object_follower"


class PathControlStatus(StrEnum):
    """Outcome of a single ``_path_control`` step."""

    IDLE = "idle"                    # no controller or no path set
    WAITING_INPUTS = "waiting"       # robot state / TF not yet available
    RUNNING = "running"              # command computed and published
    GOAL_REACHED = "goal_reached"    # robot inside goal tolerance
    FAILED = "failed"                # algorithm could not produce a command


@define
class ControllerConfig(ComponentConfig):
    """
    Controller component configuration parameters

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **algorithm**
      - `ControllersID| str`, `DWA`
      - Algorithm used to compute the control command

    * - **control_time_step**
      - `float`, `0.1`
      - Time step in MPC-like controllers (s)

    * - **debug**
      - `bool`, `False`
      - Turn on debug mode to published additional data for visualization

    * - **use_direct_sensor**
      - `bool`, `False`
      - To used direct sensor information, otherwise the node subscriber to a local map

    * - **ctrl_publish_type**
      - `CmdPublishType | str`, `TWIST_ARRAY`
      - How to publish the control commands
    ```
    """

    algorithm: Union[ControllersID, str] = field(
        default=ControllersID.DWA,
        converter=lambda value: (
            ControllersID(value) if isinstance(value, str) else value
        ),
    )
    control_time_step: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Time step between two control commands
    debug: bool = field(
        default=False
    )  # Turn on debug mode -> published additional data visualization
    use_direct_sensor: bool = field(default=False)
    ctrl_publish_type: Union[str, CmdPublishType] = field(
        default=CmdPublishType.TWIST_ARRAY,
        converter=lambda value: (
            CmdPublishType(value) if isinstance(value, str) else value
        ),
    )
    _mode: Union[str, ControllerMode] = field(
        default=ControllerMode.PATH_FOLLOWER,
        converter=lambda value: (
            ControllerMode(value) if isinstance(value, str) else value
        ),
    )


class Controller(Component):
    """
    Controller component used for path tracking and control around dynamic obstacles during navigation.


    ## Inputs:
    ```{list-table}
    :widths: 10 40 10 40
    :header-rows: 1
    * - Key Name
      - Allowed Types
      - Number
      - Default

    * - plan
      - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
      - 1
      - `Topic(name="/plan", msg_type="Path")`

    * - location
      - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
      - 1
      - `Topic(name="/odom", msg_type="Odometry")`

    * - sensor_data
      - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
      - 1
      - `Topic(name="/scan", msg_type="LaserScan")`

    * - local_map
      - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
      - 1
      - `Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid")`

    * - vision_tracking
      - [`automatika_embodied_agents.msg.Trackings`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg), [`automatika_embodied_agents.msg.Detections2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
      - 1
      - `Topic(name="/trackings", msg_type="Trackings")`
    ```

    ## Outputs:

    ```{list-table}
    :widths: 10 40 10 40
    :header-rows: 1
    * - Key Name
      - Allowed Types
      - Number
      - Default

    * - command
      - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
      - 1
      - ```Topic(name="/control", msg_type="Twist")```
    * - multi_command
      - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/msg)
      - 1
      - ```Topic(name="/control_list", msg_type="TwistArray")```
    * - interpolation
      - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
      - 1
      - ```Topic(name="/interpolated_path", msg_type="Path")```
    * - local_plan
      - [`nav_msgs.msg.Path`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)
      - 1
      - ```Topic(name="/local_path", msg_type="Path")```
    * - tracked_point
      - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)[`automatika_embodied_agents.msg.Detection2D`](https://github.com/automatika-robotics/ros-agents/tree/main/agents_interfaces/msg)
      - 1
      - ```Topic(name="/tracked_point", msg_type="PoseStamped")```
    ```

    ## Available Run Types:
    Set directly from Controller 'run_type' property.

    - *TIMED*: Compute a new control command periodically if all inputs are available.
    - *ACTIONSERVER*: Offers a ControlPath ROS action and continuously computes a new control once an action request is received until goal point is reached

    ## Usage Example:
    ```python
    from kompass.components import ControllerConfig, Controller
    from kompass.topic import Topic

    # Setup custom configuration
    my_config = ControllerConfig(loop_rate=10.0)

    # Init a controller object
    my_controller = Controller(component_name="controller", config=my_config)

    # Change an input
    my_controller.inputs(plan=Topic(name='/global_path', msg_type='Path'))

    # Change run type (default "Timed")
    my_controller.run_type = "ActionServer"

    # Change plugin
    my_controller.plugin = 'DWA'
    ```
    """

    def __init__(
        self,
        *,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[ControllerConfig] = None,
        inputs: Optional[Dict[TopicsKeys, Topic]] = None,
        outputs: Optional[Dict[TopicsKeys, Topic]] = None,
        **kwargs,
    ) -> None:
        self.config: ControllerConfig = config or ControllerConfig()

        # Update defaults from custom topics if provided
        in_topics = (
            update_topics(controller_default_inputs, **inputs)
            if inputs
            else controller_default_inputs
        )
        out_topics = (
            update_topics(controller_default_outputs, **outputs)
            if outputs
            else controller_default_outputs
        )

        super().__init__(
            config=self.config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=controller_allowed_inputs,
            allowed_outputs=controller_allowed_outputs,
            component_name=component_name,
            allowed_run_types=[ComponentRunType.TIMED, ComponentRunType.ACTION_SERVER],
            **kwargs,
        )

        # Set action type
        self.action_type = ControlPath
        self.main_action_name = "control_static_path"

    def custom_on_activate(self):
        """
        Component custom activation method to add activation based on the control mode
        """
        if (
            self.config._mode == ControllerMode.VISION_FOLLOWER
            and not self.get_in_topic(TopicsKeys.VISION_DETECTIONS)
        ):
            # Vision mode is set but the trackings topic is not provided
            raise AttributeError(
                f"Cannot use Vision Follower in {self.node_name} component without setting the 'TopicsKeys.VISION_DETECTIONS' input"
            )

        elif self.config._mode == ControllerMode.VISION_FOLLOWER:
            self._activate_vision_mode()
        else:
            self._activate_follower_mode()

    def create_all_action_servers(self):
        pass

    def custom_create_all_action_servers(self):
        super().create_all_action_servers()

    def create_all_subscribers(self):
        pass

    def custom_create_all_subscribers(self):
        """
        Overrides BaseComponent create_all_subscribers to implement controller mode change
        """
        self.get_logger().info("STARTING ALL SUBSCRIBERS")
        self.callbacks = {
            input.name: input.msg_type.callback(input, node_name=self.node_name)
            for input in self.in_topics
        }
        # Create subscribers
        for callback in self.callbacks.values():
            # In path follower mode -> skip all vision inputs
            if (
                self.config._mode == ControllerMode.PATH_FOLLOWER
                and callback.input_topic.name in self.__vision_mode_inputs()
            ):
                # skip
                continue

            # In vision follower mode skip the plan
            if (
                self.config._mode == ControllerMode.VISION_FOLLOWER
                and callback.input_topic.name
                == self.in_topic_name(TopicsKeys.GLOBAL_PLAN)
            ):
                # skip
                continue

            # Skip map is using direct sensor data
            if (
                self.config.use_direct_sensor
                and callback.input_topic.name
                == self.in_topic_name(TopicsKeys.LOCAL_MAP)
            ):
                # skip local map for direct sensor
                continue
            # Skip sensor data if using map
            if (
                not self.config.use_direct_sensor
                and callback.input_topic.name
                == self.in_topic_name(TopicsKeys.SPATIAL_SENSOR)
            ):
                # skip direct sensor
                continue

            callback.set_node_name(self.node_name)
            callback.set_subscriber(self._add_ros_subscriber(callback))
        # attach on_callback methods
        self._attach_callbacks()

    def create_all_timers(self):
        """Overrides create_all_timers from BaseComponent to add timers for commands execution and tracking publishing"""
        super().create_all_timers()

        # Create timer for publishing commands if parallel publishing is enabled
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_PARALLEL:
            self.get_logger().debug(
                f"Creating execution timer with step: {self.config.control_time_step}"
            )
            self.__cmd_execution_timer = self.create_timer(
                self.config.control_time_step,
                self._cmds_publishing_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        # Create timer to publish additional control tracking info
        self.__info_publishing_timer = self.create_timer(
            1 / self.config.loop_rate,
            self._info_publishing_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def destroy_all_timers(self):
        """Overrides destroy_all_timers from BaseComponent to destroy the timers for commands execution and tracking publishing"""
        super().destroy_all_timers()
        # Destroy execution timer/rate
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_PARALLEL:
            self.destroy_timer(self.__cmd_execution_timer)

        self.destroy_timer(self.__info_publishing_timer)

    def create_all_services(self):
        """
        Creates all node services
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            self.__vision_tracking_srv = self.create_service(
                StopVisionTracking,
                f"{self.node_name}/end_vision_tracking",
                self._end_vision_tracking_srv_callback,
            )
        super().create_all_services()

    def destroy_all_services(self):
        """
        Destroys all node services
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            self.destroy_service(self.__vision_tracking_srv)
        super().destroy_all_services()

    def inspect_component(self) -> str:
        """
        Method to return a string representation of the component configuration, used for inspection and LLM-based reasoning about the component

        :return: String representation of the component configuration
        :rtype: str
        """
        base_info = super().inspect_component()
        # Get all control algorithms
        try:
            from kompass_core.control import ControllersID

            control_algorithms = "Available control algorithms are:\n"
            for item in ControllersID:
                control_algorithms += f"- {item.value}\n"
        except ImportError:
            control_algorithms = "Could not retrieve available control algorithms"

        # Control Modes
        modes_info = (
            "Controller modes:\n"
            f"- {ControllerMode.PATH_FOLLOWER.value}: Follow a global plan while avoiding dynamic obstacles. "
            "In this mode, the Controller is driven by the Planner component which generates the path. "
            "Do NOT send navigation goals directly to the Controller — use the Planner's action server instead. "
            "The Planner will generate a plan and forward it to the Controller automatically.\n"
            f"- {ControllerMode.VISION_FOLLOWER.value}: Start follow a vision target."
            "This mode uses the track vision target action server. To trigger it, First set_algorithm to a vision based algorithm (VisionRGBFollower or VisionRGBDFollower), then send a goal with:\n"
            "  - 'label': the object class to track (e.g. 'person', 'cup') as specified by the user.\n"
            "  - 'search_radius': how far (meters) to search for the target. Use a reasonable default if not specified.\n"
            "  - 'search_timeout': max time (seconds) to search before giving up.\n"
            "  - 'pose_x', 'pose_y': pixel coordinates of the target in the image frame if known, otherwise set to 0.\n"
            "The mode is determined by the selected algorithm — setting a vision-based algorithm switches to vision mode, "
            "and a path-following algorithm switches to path follower mode."
        )
        return base_info + "\n" + control_algorithms + "\n" + modes_info

    def get_ros_entrypoints(self) -> Dict[str, Dict[str, Any]]:
        """Get the component ROS entry points (additional services and actions) as a dictionary."""
        entry_points = {"services": {}, "actions": {}}
        # Register the vision tracking action server as an additional entry point
        entry_points["actions"].update({
            "track_vision_target": TrackVisionTarget,
        })
        return entry_points

    def _cmds_publishing_callback(self):
        """Commands execution timer callback"""
        # If end is reached do not publish new command
        if self.__reached_end:
            self.get_logger().debug("End of action reached -> Not executing commands")
            self._cmds_queue.queue.clear()
            return

        try:
            cmd = self._cmds_queue.get_nowait()
        except Empty:
            self.get_logger().debug("No command to execute")
            return

        # Publish one twist message
        self.get_publisher(TopicsKeys.INTERMEDIATE_CMD).publish(cmd)

    def _info_publishing_callback(self):
        """Tracking publishing timer callback"""
        # Publish tracked point on the global path
        if self.mode == ControllerMode.VISION_FOLLOWER:
            return

        if self.tracked_point is not None:
            self.get_publisher(TopicsKeys.TRACKED_POINT).publish(
                self.tracked_point,
                frame_id=self.config.frames.world,
                time_stamp=self.get_ros_time(),
            )
        # Publish local plan
        if self.local_plan:
            self.get_publisher(TopicsKeys.LOCAL_PLAN).publish(
                self.local_plan,
                frame_id=self.config.frames.world,
                time_stamp=self.get_ros_time(),
            )

        if not self.config.debug:
            return

        # PUBLISH DEBUG DATA
        # Publish interpolated path
        if self.interpolated_path:
            self.get_publisher(TopicsKeys.INTERPOLATED_PATH).publish(
                self.interpolated_path,
                frame_id=self.config.frames.world,
                time_stamp=self.get_ros_time(),
            )

        debug_paths = self.local_plan_debug
        if debug_paths:
            self.get_publisher(TopicsKeys.PATH_SAMPLES).publish(
                debug_paths,
                frame_id=self.config.frames.world,
                time_stamp=self.get_ros_time(),
            )
            self.get_logger().warning("Sleeping for 10seconds to plot in debug mode")
            time.sleep(10.0)

    @property
    def tracked_point(self) -> Optional[np.ndarray]:
        """
        Getter of tracked pose on the reference path if a path is set to the controller

        :return: _description_
        :rtype: StrEnum
        """
        if not self.__path_controller:
            return None

        tracked_state: Optional[RobotState] = self.__path_controller.tracked_state

        if not tracked_state:
            return None

        position = [tracked_state.x, tracked_state.y, 0.0]
        orientation = from_euler_to_quaternion(
            yaw=tracked_state.yaw, pitch=0.0, roll=0.0
        )
        position.extend(orientation)
        return np.array(position)

    @property
    def local_plan(self) -> Optional[Path]:
        """
        Getter of controller local plan

        :return: _description_
        :rtype: StrEnum
        """
        # NOTE: For now only DWA provides a local plan
        if self.algorithm != ControllersID.DWA or not self.__path_controller:
            return None

        kompass_cpp_path = self.__path_controller.optimal_path()
        if not kompass_cpp_path:
            return None

        ros_path = Path()
        parsed_points = []

        for point_x, point_y in zip(kompass_cpp_path.x, kompass_cpp_path.y):
            ros_point = PoseStamped()
            ros_point.pose.position.x = np.float64(point_x)
            ros_point.pose.position.y = np.float64(point_y)
            parsed_points.append(ros_point)

        ros_path.poses = parsed_points
        return ros_path

    @property
    def local_plan_debug(self) -> Optional[Path]:
        """
        Getter of controller local plan debug (collected samples)

        :return: _description_
        :rtype: StrEnum
        """
        # NOTE: For now only DWA provides a local plan
        if self.algorithm != ControllersID.DWA or not self.__path_controller:
            return None

        # If result is not processed -> no debug is available yet
        if not self.__path_controller.has_result():
            return

        (paths_x, paths_y) = self.__path_controller.planner.get_debugging_samples()

        if paths_x is None:
            return None

        ros_path = Path()
        ros_points = []

        for idx in range(paths_x.shape[0]):
            # Extract the x and y coordinates for the current path
            path_x_i = paths_x[idx, :]
            path_y_i = paths_y[idx, :]

            # Create an array of PoseStamped messages using vectorized operations
            points_xy = np.column_stack((path_x_i, path_y_i))
            for x, y in points_xy:
                ros_point = PoseStamped()
                ros_point.pose.position.x = np.float64(x)
                ros_point.pose.position.y = np.float64(y)
                ros_points.append(ros_point)

        ros_path.poses = ros_points
        return ros_path

    @property
    def interpolated_path(self) -> Optional[Path]:
        """Getter of interpolated global path

        :return: Path Interpolation
        :rtype: Optional[Path]
        """
        if not self.__path_controller:
            return None
        kompass_cpp_path = self.__path_controller.interpolated_path()
        if not kompass_cpp_path:
            return None
        ros_path = Path()
        parsed_points = []
        for point in kompass_cpp_path.points:
            ros_point = PoseStamped()
            ros_point.pose.position.x = np.float64(point[0])
            ros_point.pose.position.y = np.float64(point[1])
            parsed_points.append(ros_point)

        ros_path.poses = parsed_points
        return ros_path

    @property
    def direct_sensor(self) -> bool:
        """Getter of flag to use direct sensor data in the controller, if False the controller uses the local map

        :return: Use direct sensor data flag
        :rtype: bool
        """
        return self.config.use_direct_sensor

    @direct_sensor.setter
    def direct_sensor(self, value: bool) -> None:
        """Setter of flag to use direct sensor data in the controller, if False the controller uses the local map

        :param value: Use direct sensor data flag
        :type value: bool
        """
        # Note: Only DWA takes local map
        if (
            self.algorithm
            not in [
                ControllersID.DWA,
                ControllersID.VISION_DEPTH,
                ControllersID.PURE_PURSUIT,
            ]
            and not value
        ):
            get_logger(self.node_name).warning(
                f"Cannot use Local Map with {self.algorithm} - Setting 'direct_sensor' to True"
            )
            self.config.use_direct_sensor = True
            return
        self.config.use_direct_sensor = value

    @property
    def algorithm(self) -> ControllersID:
        """
        Getter of controller algorithm

        :return: _description_
        :rtype: StrEnum
        """
        return self.config.algorithm

    @algorithm.setter
    def algorithm(self, value: Union[str, ControllersID]) -> None:
        """
        Setter of algorithm with int value or enum value from ControllersID

        :param value: algorithm value
        :type value: Union[str, ControllersID]

        :raises ValueError: If given int value is not one of the values in ControllersID
        """
        self.config.algorithm = value
        # Select mode based on the algorithm
        if value in [
            ControllersID.VISION_DEPTH,
            ControllersID.VISION_IMG,
            ControllersID.VISION_DEPTH.value,
            ControllersID.VISION_IMG.value,
        ]:
            self._activate_vision_mode()
        else:
            self._activate_follower_mode()

    @property
    def mode(self) -> ControllerMode:
        """Get the current control mode

        :return: control mode ControllerMode.PATH_FOLLOWER or ControllerMode.VISION_FOLLOWER
        :rtype: ControllerMode
        """
        return self.config._mode

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "set_algorithm",
                "description": "Set the controller algorithm used to compute motion control commands. "
                "Available algorithms: 'DWA' (Dynamic Window Approach for path following with obstacle avoidance), "
                "'Stanley' (Stanley steering controller for path following), "
                "'DVZ' (Deformable Virtual Zone for reactive obstacle avoidance), "
                "'PurePursuit' (Pure Pursuit geometric path follower), "
                "'VisionRGBFollower' (vision-based target following using RGB images), "
                "'VisionRGBDFollower' (vision-based target following using RGB-D depth images). "
                "Use when the user asks to change or switch the control algorithm.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "algorithm_value": {
                            "type": "string",
                            "description": "The controller algorithm to use. Must be one of: 'DWA', 'Stanley', 'DVZ', 'PurePursuit', 'VisionRGBFollower', 'VisionRGBDFollower'.",
                            "enum": [
                                "DWA",
                                "Stanley",
                                "DVZ",
                                "PurePursuit",
                                "VisionRGBFollower",
                                "VisionRGBDFollower",
                            ],
                        },
                    },
                    "required": ["algorithm_value"],
                },
            },
        }
    )
    def set_algorithm(self, algorithm_value: Union[str, ControllersID], **_) -> bool:
        """
        Component action - Set controller algorithm action

        :param algorithm_value: algorithm value
        :type algorithm_value: Union[str, ControllersID]

        :raises Exception: Exception while updating algorithm value

        :return: Success
        :rtype: bool
        """
        if self.algorithm in [ControllersID(algorithm_value), algorithm_value]:
            return True
        try:
            self.algorithm = algorithm_value
        except Exception as e:
            self.get_logger().error(
                f"Failed to set controller algorithm to '{algorithm_value}': {e}"
            )
            return False

    def _activate_vision_mode(self):
        """Activate object following mode using vision detections"""
        # Activate vision subscriber
        if not self.in_topic_name(TopicsKeys.VISION_DETECTIONS):
            raise ValueError(
                f"Error activating vision tracking mode. No input topic is provided for '{TopicsKeys.VISION_DETECTIONS}'"
            )

        self.action_type = TrackVisionTarget
        self.main_action_name = "track_vision_target"

        self.config._mode = ControllerMode.VISION_FOLLOWER

        if self.run_type != ComponentRunType.ACTION_SERVER:
            self._old_run_type = self.run_type
            self.run_type = ComponentRunType.ACTION_SERVER

        if not self.is_node_initialized():
            return

        if self.algorithm not in [ControllersID.VISION_IMG, ControllersID.VISION_DEPTH]:
            self.get_logger().warning(
                f"Vision Tracking algorithm is not set, setting to default '{ControllersID.VISION_DEPTH}'"
            )
            self.algorithm = ControllersID.VISION_DEPTH

        self.custom_create_all_subscribers()
        self.custom_create_all_action_servers()

    def _activate_follower_mode(self):
        """Activate path following mode by creating all missing subscriptions"""
        # Set the main action type to path control
        self.action_type = ControlPath

        self.config._mode = ControllerMode.PATH_FOLLOWER
        if hasattr(self, "_old_run_type"):
            self.run_type = self._old_run_type

        if not self.is_node_initialized():
            return

        if self.algorithm in [ControllersID.VISION_IMG, ControllersID.VISION_DEPTH]:
            self.get_logger().warning(
                f"Path control algorithm is not set, setting to default '{ControllersID.DWA}'"
            )
            self.algorithm = ControllersID.DWA

        self.custom_create_all_subscribers()
        self.custom_create_all_action_servers()

    def _update_vision(self) -> None:
        """Update vision detections and depth image from the vision callback"""
        vision_callback = self.get_callback(TopicsKeys.VISION_DETECTIONS)
        self.vision_detections = (
            vision_callback.get_output(clear_last=True) if vision_callback else None
        )
        self.depth_image = vision_callback.depth_image if vision_callback else None
        # Depth image cam info
        depth_img_info_callback = self.get_callback(TopicsKeys.DEPTH_CAM_INFO)
        self.depth_image_info = (
            depth_img_info_callback.get_output() if depth_img_info_callback else None
        )

    def _update_state(self, block: bool = True) -> None:
        """
        Updates node inputs from associated callbacks.

        :param block: If True, blocks up to ``topic_subscription_timeout`` waiting
            for the odom->world TF when the two frames differ. Pass ``False`` from
            fast control loops to read the latest cached transform without blocking;
            if the TF is not yet available the update is skipped for this tick.
        """
        if self.config._mode == ControllerMode.PATH_FOLLOWER:
            plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
            self.plan: Optional[Path] = (
                plan_callback.get_output() if plan_callback else None
            )

        state_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)

        if self.config.frames.odom == self.config.frames.world:
            self.robot_state = (
                state_callback.get_output(get_front=True, clear_last=True)
                if state_callback
                else None
            )
        else:
            if block and not self.odom_tf_listener.transform:
                timeout = 0.0
                while (
                    not self.odom_tf_listener.transform
                    and timeout < self.config.topic_subscription_timeout
                ):
                    self.get_logger().warning(
                        f"Waiting to get TF from {self.config.frames.odom} frame to {self.config.frames.world} frame...",
                        once=True,
                    )
                    timeout += 1 / self.config.loop_rate
                    time.sleep(1 / self.config.loop_rate)
                if not self.odom_tf_listener.transform:
                    self.get_logger().error(
                        f"Could not get TF from {self.config.frames.odom} frame to {self.config.frames.world} frame after {self.config.topic_subscription_timeout} seconds"
                    )
            if not self.odom_tf_listener.transform:
                self.robot_state = None
            else:
                self.robot_state = (
                    state_callback.get_output(
                        transformation=self.odom_tf_listener.transform,
                        get_front=True,
                        clear_last=True,
                    )
                    if state_callback
                    else None
                )

        if self.direct_sensor:
            sensor_callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
            self.sensor_data = (
                sensor_callback.get_output(
                    transformation=self.__sensor_tf_listener.transform
                    if self.__sensor_tf_listener
                    else None
                )
                if sensor_callback
                else None
            )
        else:
            map_callback = self.get_callback(TopicsKeys.LOCAL_MAP)
            if map_callback:
                self.local_map: Optional[np.ndarray] = map_callback.get_output()
                _metadata = map_callback.get_output(get_metadata=True)
                self.local_map_resolution = (
                    _metadata["resolution"] if _metadata else None
                )
            else:
                self.local_map = None
                self.local_map_resolution = None

    def _attach_callbacks(self) -> None:
        """
        Attaches method to set received plan to the controller
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            return
        # Adds callback to set the path in the controller when a new plan is received
        plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
        if plan_callback:
            plan_callback.on_callback_execute(self._set_path_to_controller)

        if self.direct_sensor:
            # If direct sensor information is used set maximum range for PointCloud data
            sensor_callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
            if isinstance(sensor_callback, PointCloudCallback):
                sensor_callback.max_range = (
                    self.config.prediction_horizon * self.robot.ctrl_vx_limits.max_vel
                )
                self.get_logger().info(
                    f"Setting PointCloud max range to robot max forward horizon '{sensor_callback.max_range}' to limit computations"
                )

    def _set_path_to_controller(self, msg, **_) -> None:
        """
        Set a new plan to the controller/follower
        """
        self.__reached_end = False
        if self.__path_controller:
            self.__path_controller.set_path(global_path=msg)
        if len(msg.poses) > 1:
            self.__goal_point = RobotState(
                x=msg.poses[-1].pose.position.x, y=msg.poses[-1].pose.position.y
            )
        else:
            self.__goal_point = None

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        # reached end path / end of control action
        self.__reached_end: bool = False

        self.robot_state: Optional[RobotState] = (
            None  # robot current state - to be updated from odom
        )
        self.plan: Optional[Path] = None  # robot plan (global path)

        # INIT PATH CONTROLLER
        self.__robot = Robot(
            robot_type=self.config.robot.model_type,
            geometry_type=self.config.robot.geometry_type,
            geometry_params=self.config.robot.geometry_params,
            state=self.robot_state or RobotState(),
        )

        # SET robot control limits
        self.__robot_ctr_limits = RobotCtrlLimits(
            vx_limits=self.config.robot.ctrl_vx_limits,
            vy_limits=self.config.robot.ctrl_vy_limits,
            omega_limits=self.config.robot.ctrl_omega_limits,
        )

        self.__reached_end = False
        self.__lat_dist_error: float = 0.0
        self.__ori_error: float = 0.0
        self._tracked_center: Optional[np.ndarray] = None

        # Command queue to send controller command list to the robot
        self._cmds_queue: Queue = Queue()

        if self.direct_sensor:
            sensor_topic = self._inputs_list[
                self._inputs_keys.index(TopicsKeys.SPATIAL_SENSOR)
            ]
            # Setup transform listener
            self.__sensor_tf_listener: TFListener = (
                self.pc_tf_listener
                if sensor_topic.msg_type._ros_type == PointCloud2
                else self.scan_tf_listener
            )

            self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = None

        self.__path_controller: Optional[ControllerType] = None
        self.__vision_controller: Optional[ControllerType] = None

        if self.config._mode == ControllerMode.PATH_FOLLOWER:
            config_kwargs = {}
            if self.direct_sensor and self.__sensor_tf_listener.got_transform:
                config_kwargs["proximity_sensor_position_to_robot"] = (
                    self.__sensor_tf_listener.translation
                )
                config_kwargs["proximity_sensor_rotation_to_robot"] = (
                    self.__sensor_tf_listener.rotation
                )
                config_kwargs["control_time_step"] = self.config.control_time_step
            # Get default controller configuration and update it from user defined config
            _controller_config = self._configure_algorithm(
                ControlConfigClasses[self.algorithm](**config_kwargs)
            )

            self.__path_controller = ControlClasses[self.algorithm](
                robot=self.__robot,
                config=_controller_config,
                ctrl_limits=self.__robot_ctr_limits,
                config_file=self._config_file,
                config_root_name=f"{self.node_name}.{self.config.algorithm}",
                control_time_step=self.config.control_time_step,
            )

        # Vision Follower variables
        self.vision_detections: Optional[Bbox2D] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_image_info: Optional[Dict] = None

    def _stop_robot(self):
        """
        Publishes a zero velocity command to stop the robot
        """
        # Clear all commands to stop the robot
        self._cmds_queue.queue.clear()
        # send zero command to stop the robot
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_ARRAY:
            _cmd_vel_array = init_twist_array_msg(1)
            self.get_publisher(TopicsKeys.INTERMEDIATE_CMD_LIST).publish(_cmd_vel_array)
        else:
            self.get_publisher(TopicsKeys.INTERMEDIATE_CMD).publish([0.0, 0.0, 0.0])

    def _publish(
        self,
        commands_vx: List[float],
        commands_vy: List[float],
        commands_omega: List[float],
    ):
        # TWIST_PARALLEL : Set to queue to publish in parallel
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_PARALLEL:
            # Empty the commands queue
            self._cmds_queue.queue.clear()

            # Put new control commands to the queue
            [
                self._cmds_queue.put((float(vx), float(vy), float(omega)))
                for (vx, vy, omega) in zip(
                    commands_vx,
                    commands_vy,
                    commands_omega,
                )
            ]
            return

        # TWIST_ARRAY: Publish all
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_ARRAY:
            # publish a Twist Array
            _cmd_vel_array = init_twist_array_msg(
                number_of_cmds=len(commands_vx),
                linear_x=commands_vx,
                linear_y=commands_vy,
                angular=commands_omega,
            )
            _cmd_vel_array.time_step = self.config.control_time_step

            self.get_publisher(TopicsKeys.INTERMEDIATE_CMD_LIST).publish(_cmd_vel_array)
            return

        # TWIST_SEQUENCE: Publish one-by-one in a blocking loop
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_SEQUENCE:
            for vx, vy, omega in zip(commands_vx, commands_vy, commands_omega):
                self.get_publisher(TopicsKeys.INTERMEDIATE_CMD).publish([
                    float(vx),
                    float(vy),
                    float(omega),
                ])
                time.sleep(self.config.control_time_step)

    def _path_control(self) -> PathControlStatus:
        """
        Run one step of the path-following controller.

        This method reports the outcome via ``PathControlStatus``;
        callers own the lifecycle transition (setting ``__reached_end``,
        clearing the plan callback, aborting the action, etc.).
        """
        if self.__path_controller is None:
            return PathControlStatus.IDLE

        if not self.__path_controller.path:
            # No global path -> nothing to control toward
            return PathControlStatus.IDLE

        self._update_state(block=False)

        waited = 0.0
        while not self.robot_state and waited < self.config.topic_subscription_timeout:
            self.get_logger().warning(
                "Waiting to get all new incoming data...", once=True
            )
            time.sleep(1 / self.config.loop_rate)
            waited += 1 / self.config.loop_rate
            self._update_state(block=False)

        if not self.robot_state:
            self.get_logger().warning(
                f"Robot state unavailable after {self.config.topic_subscription_timeout}s -> skipping control step",
                throttle_duration_sec=5.0,
            )
            return PathControlStatus.WAITING_INPUTS

        laser_scan = None
        point_cloud = None
        local_map = None

        if self.direct_sensor:
            if isinstance(self.sensor_data, LaserScanData):
                laser_scan = self.sensor_data
            else:
                point_cloud = self.sensor_data
        else:
            local_map = self.local_map

        if self.reached_point(self.__goal_point):
            self._stop_robot()
            return PathControlStatus.GOAL_REACHED

        cmd_found: bool = self.__path_controller.loop_step(
            current_state=self.robot_state,  # type: ignore
            laser_scan=laser_scan,
            point_cloud=point_cloud,
            local_map=local_map,
            local_map_resolution=getattr(self, "local_map_resolution", None),
            debug=self.config.debug,
        )

        # LOG CONTROLLER INFO
        self.get_logger().debug(f"{self.__path_controller.logging_info()}")

        if not cmd_found:
            return PathControlStatus.FAILED

        self.health_status.set_healthy()

        # Update controller path tracking info (errors)
        self.__lat_dist_error = self.__path_controller.distance_error
        self.__ori_error = self.__path_controller.orientation_error

        self._publish(
            self.__path_controller.linear_x_control,
            self.__path_controller.linear_y_control,
            self.__path_controller.angular_control,
        )

        return PathControlStatus.RUNNING

    def __vision_mode_inputs(self) -> List[str]:
        """Helper method to get the names of the topics required for the vision follower mode

        :return: _description_
        :rtype: List[str]
        """
        return [
            self.in_topic_name(TopicsKeys.VISION_DETECTIONS),
            self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO),
        ]

    def _end_vision_tracking_srv_callback(
        self, _: StopVisionTracking.Request, response: StopVisionTracking.Response
    ) -> StopVisionTracking.Response:
        """Service callback to end Vision target tracking action

        :param request: Request to end the vision tracking
        :type request: StopVisionTracking.Request
        :param response: Service response
        :type response: StopVisionTracking.Response

        :return: Service response
        :rtype: StopVisionTracking.Response
        """
        # Vision tracking mode is already not active
        if (
            self.config._mode == ControllerMode.PATH_FOLLOWER
            or self._tracked_center is None
        ):
            self.get_logger().warning("No ongoing vision tracking")
            response.success = True
            return response

        self.get_logger().warning("Ending Vision Tracking Action for target")

        # Set reached end to true so the action server can end following
        self.__reached_end = True

        # Wait for the action server to reset
        _counter: int = 0
        while _counter < 10:
            time.sleep(1 / self.config.loop_rate)
            if not self.__reached_end:
                response.success = True
                self._tracked_center = None
                return response

        response.success = False
        return response

    def __setup_vision_controller(self) -> Any:
        """Setup and configure a VisionDWA controller instance

        :return: Configured controller
        :rtype: VisionDWA
        """
        timeout = 0.0
        # Get the depth image transform if the input is provided
        if self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO):
            while (
                not (self.depth_tf_listener.got_transform and self.depth_image_info)
                and timeout < self.config.topic_subscription_timeout
            ):
                self._update_vision()
                self.get_logger().info(
                    "Waiting to get Depth camera to body TF to initialize Vision Follower...",
                    once=True,
                )
                time.sleep(1 / self.config.loop_rate)
                timeout += 1 / self.config.loop_rate

        if timeout >= self.config.topic_subscription_timeout:
            return None

        self.get_logger().info(
            "Got Depth camera to body TF -> Setting up Vision Follower controller"
        )

        # Determine whether global localization is available by waiting for
        # odometry and (if needed) the odom->world TF, using the same timeout
        # pattern as the depth TF wait above.
        state_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        loc_timeout = 0.0
        while loc_timeout < self.config.topic_subscription_timeout:
            self._update_state()
            has_odom = state_callback is not None and self.robot_state is not None
            has_world_tf = (self.config.frames.odom == self.config.frames.world) or (
                self.odom_tf_listener is not None
                and self.odom_tf_listener.transform is not None
            )
            if has_odom and has_world_tf:
                break
            self.get_logger().info(
                "Waiting for robot localization to determine tracking mode...",
                once=True,
            )
            time.sleep(1 / self.config.loop_rate)
            loc_timeout += 1 / self.config.loop_rate

        has_global_localization = has_odom and has_world_tf
        use_local = not has_global_localization

        if use_local:
            self.get_logger().info(
                "No global localization available — vision follower will "
                "operate in local (robot-relative) frame"
            )
        else:
            self.get_logger().info(
                "Global localization available — vision follower will "
                "operate in world frame with velocity tracking"
            )

        config = ControlConfigClasses[self.algorithm](
            control_time_step=self.config.control_time_step,
            camera_position_to_robot=self.depth_tf_listener.translation,
            camera_rotation_to_robot=self.depth_tf_listener.rotation,
            _use_local_coordinates=use_local,
        )

        _controller_config = self._configure_algorithm(config)

        # Apply the (possibly user-overridden) buffer size to the detections callback
        detections_callback = self.get_callback(TopicsKeys.VISION_DETECTIONS)
        if detections_callback:
            detections_callback.set_buffer_size(_controller_config.buffer_size)

        # re-read the file and overwrite any programmatic overrides.
        controller = ControlClasses[self.algorithm](
            robot=self.__robot,
            ctrl_limits=self.__robot_ctr_limits,
            config=_controller_config,
            camera_focal_length=self.depth_image_info["focal_length"]
            if self.depth_image_info
            else None,
            camera_principal_point=self.depth_image_info["principal_point"]
            if self.depth_image_info
            else None,
        )

        self.get_logger().info(
            f"Vision Controller '{self.algorithm}' is initialized and ready to use"
        )
        return controller

    def __setup_initial_tracking_target(
        self, controller: Any, label: str, pose_x: int, pose_y: int
    ) -> bool:
        """Set the initial target to the vision follower controller to start tracking

        :param controller: Vision Follower controller object
        :type controller: Any
        :param label: Target label
        :type label: str
        :param pose_x: Target center pixel x coordinates in the image plane
        :type pose_x: int
        :param pose_y: Target center pixel y coordinates in the image plane
        :type pose_y: int
        :return: If target is set successfully to the controller
        :rtype: bool
        """
        timeout = 0.0
        while (not self.robot_state or self.depth_image is None) and (
            timeout < self.config.topic_subscription_timeout
        ):
            self._update_state()
            self._update_vision()
            timeout += 1 / self.config.loop_rate
            time.sleep(1 / self.config.loop_rate)
        if not self.robot_state or self.depth_image is None:
            self.get_logger().error(
                f"Could not get robot state and depth image to set initial tracking target after {self.config.topic_subscription_timeout} seconds"
            )
            return False
        if label != "":
            target_2d = None
            vision_callback = self.get_callback(TopicsKeys.VISION_DETECTIONS)
            timeout = 0.0
            while not target_2d and timeout < self.config.topic_subscription_timeout:
                self._update_vision()
                target_2d = (
                    vision_callback.get_output(label=label) if vision_callback else None
                )
                self.get_logger().info(
                    f"Waiting to get target {label} from vision detections...",
                    once=True,
                )
                timeout += 1 / self.config.loop_rate
                time.sleep(1 / self.config.loop_rate)
            if not target_2d:
                self.get_logger().error(
                    f"Could not find target {label} in vision detections"
                )
                return False
            if self.depth_image is None:
                self.get_logger().warning(
                    f"Target '{label}' found but no aligned depth image is available; "
                    "check that the detector is publishing depth with detections"
                )
            self.get_logger().info(
                f"Got initial target {label}, setting to controller..."
            )
            found_target = controller.set_initial_tracking_2d_target(
                target_box=target_2d[0],
                current_state=self.robot_state,
                aligned_depth_image=self.depth_image,
            )
        else:
            if self.algorithm == ControllersID.VISION_IMG:
                self.get_logger().error(
                    "Cannot use Vision RGB Follower without providing a target label"
                )
                self.health_status.set_fail_algorithm(
                    algorithm_names=[self.algorithm.value]
                )
                return False

            # If no label is provided, use the provided pixel coordinates (Only works with depth)
            found_target = controller.set_initial_tracking_image(
                self.robot_state,
                pose_x,
                pose_y,
                self.vision_detections,
                self.depth_image,
            )
        return found_target

    def __gather_local_obstacles(self):
        """Collect latest obstacle inputs for the vision follower's loop_step."""
        laser_scan = None
        point_cloud = None
        local_map = None
        if self.direct_sensor:
            if isinstance(self.sensor_data, LaserScanData):
                laser_scan = self.sensor_data
            else:
                point_cloud = self.sensor_data
        else:
            local_map = self.local_map
        return laser_scan, point_cloud, local_map

    def __terminate_vision_action(
        self,
        goal_handle,
        result: TrackVisionTarget.Result,
        start_time: float,
        status: str,
    ) -> TrackVisionTarget.Result:
        """Finalize a vision tracking action (succeed/abort/cancel), stop robot, reset state."""
        result.tracked_duration = time.time() - start_time
        result.success = status == "succeed"
        with self._main_goal_lock:
            if status == "cancel" and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif status == "succeed":
                goal_handle.succeed()
            elif goal_handle.is_active:
                goal_handle.abort()
        self.__reached_end = True
        self._tracked_center = None
        self.vision_detections = None
        self._stop_robot()
        return result

    def _vision_tracking_callback(self, goal_handle) -> TrackVisionTarget.Result:
        """Vision target tracking action callback

        :param goal_handle: Vision target tracking action goal message
        :type goal_handle: TrackVisionTarget.Goal
        :return: Vision target tracking action result
        :rtype: TrackVisionTarget.Result
        """
        self.__reached_end = False
        request_msg = goal_handle.request
        feedback_msg = TrackVisionTarget.Feedback()
        result = TrackVisionTarget.Result()
        start_time = time.time()

        if not self.__vision_controller:
            self.__vision_controller = self.__setup_vision_controller()
        if not self.__vision_controller:
            self.get_logger().error(
                "Could not initialize controller -> ABORTING ACTION"
            )
            return self.__terminate_vision_action(
                goal_handle, result, start_time, "abort"
            )

        if not self.__setup_initial_tracking_target(
            self.__vision_controller,
            request_msg.label,
            request_msg.pose_x,
            request_msg.pose_y,
        ):
            self.get_logger().error("No Target found on image -> ABORTING ACTION")
            return self.__terminate_vision_action(
                goal_handle, result, start_time, "abort"
            )

        self._tracked_center = np.array([request_msg.pose_x, request_msg.pose_y])

        while (
            self.config._mode == ControllerMode.VISION_FOLLOWER
            and not self.__reached_end
        ):
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info("Vision Following Action Canceled!")
                return self.__terminate_vision_action(
                    goal_handle, result, start_time, "cancel"
                )

            # Refresh inputs non-blocking; follower tolerates missing frames via
            # its internal target_wait_timeout / enable_search.
            self._update_vision()
            self._update_state(block=False)
            laser_scan, point_cloud, local_map = self.__gather_local_obstacles()

            found_ctrl = self.__vision_controller.loop_step(
                detections_2d=self.vision_detections or [],
                current_state=self.robot_state,
                depth_image=self.depth_image,
                laser_scan=laser_scan,
                point_cloud=point_cloud,
                local_map=local_map,
                local_map_resolution=getattr(self, "local_map_resolution", None),
            )

            if not found_ctrl:
                self.get_logger().warning(
                    "Vision follower lost the target -> Aborting action"
                )
                return self.__terminate_vision_action(
                    goal_handle, result, start_time, "abort"
                )

            feedback_msg.distance_error = float(self.__vision_controller.dist_error)
            feedback_msg.orientation_error = float(
                self.__vision_controller.orientation_error
            )
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().debug(
                f"Following tracked target with control: {self.__vision_controller.linear_x_control}, {self.__vision_controller.angular_control}"
            )
            self._publish(
                self.__vision_controller.linear_x_control,
                self.__vision_controller.linear_y_control,
                self.__vision_controller.angular_control,
            )

            time.sleep(1 / self.config.loop_rate)

        self.get_logger().warning("Ending tracking action")
        return self.__terminate_vision_action(
            goal_handle, result, start_time, "succeed"
        )

    def _path_tracking_callback(self, goal_handle) -> ControlPath.Result:
        """
        Executes the control action
        Controller keeps computing robot commands until the end of path is reached

        :param goal_handle: Action request
        :type goal_handle: ControlPath

        :return: Action result
        :rtype: ControlPath.Result
        """
        self.get_logger().info("Started control action...")

        # SETUP ACTION REQUEST/FEEDBACK/RESULT
        request_msg = goal_handle.request

        feedback_msg = ControlPath.Feedback()
        feedback_msg.control_feedback = kompass_msgs.ControllerInfo()

        result = ControlPath.Result()
        result.destination_error = kompass_msgs.PathTrackingError()

        # Check for specified controller algorithm
        if request_msg.algorithm_name:
            try:
                self.algorithm = request_msg.algorithm_name
            except Exception:
                self.get_logger().error(
                    "Invalid Algorithm in control action request -> Aborting"
                )
                goal_handle.abort()
                return result

            # Get default controller configuration and update it from user defined config
            _controller_config = self._configure_algorithm(
                ControlConfigClasses[self.algorithm]()
            )

            self.__path_controller = ControlClasses[self.algorithm](
                robot=self.__robot,
                config=_controller_config,
                ctrl_limits=self.__robot_ctr_limits,
                config_file=self._config_file,
                config_root_name=f"{self.node_name}.{self.config.algorithm}",
            )
            self.get_logger().info(f"Initialized '{self.algorithm}' controller")
        else:
            self.get_logger().warning(
                f"No Algorithm is provided in control action request -> Using node configured algorithm '{self.algorithm}'"
            )

        # Check if inputs are available until timeout
        if not self.callbacks_inputs_check(
            inputs_to_exclude=self.__vision_mode_inputs()
        ):
            self.get_logger().error(
                "Requested action inputs are not available -> Aborting"
            )
            goal_handle.abort()
            return result

        # Set current mode to path following
        # Note: This will automatically end the vision target tracking action if it is ongoing
        self.config._mode = ControllerMode.PATH_FOLLOWER

        self.__path_controller.set_path(self.plan)  # type: ignore

        self.__reached_end: bool = False

        while True:
            status: PathControlStatus = self._path_control()

            if status == PathControlStatus.GOAL_REACHED:
                self.__reached_end = True
                break
            if status in (PathControlStatus.FAILED, PathControlStatus.IDLE):
                break

            # RUNNING or WAITING_INPUTS: keep the action alive, publish feedback
            # TODO: Add an option to update the computation time using timeit and send it in feedback, for controller performance tracking
            feedback_msg.control_list = init_twist_array_msg(
                number_of_cmds=len(self.__path_controller.linear_x_control),
                linear_x=self.__path_controller.linear_x_control,
                linear_y=self.__path_controller.linear_y_control,
                angular=self.__path_controller.angular_control,
            )

            feedback_msg.global_path_deviation.lateral_distance_error = (
                self.__lat_dist_error
            )
            feedback_msg.global_path_deviation.orientation_error = self.__ori_error
            feedback_msg.prediction_horizon = self.config.prediction_horizon

            self.get_logger().info(
                "Controlling Path: {0}".format(feedback_msg.control_feedback)
            )

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1 / self.config.loop_rate)

        if self.__reached_end:
            # WHEN PATH TRACKER SERVICE RETURNS RESULT
            self.get_logger().info("Reached end of path!")
            # Update the result msg
            result.destination_error.lateral_distance_error = self.__lat_dist_error
            result.destination_error.orientation_error = self.__ori_error
            goal_handle.succeed()

        else:
            self.get_logger().warning("Failed to reach end of path!... Aborting")
            goal_handle.abort()

        self._stop_robot()

        return result

    def main_action_callback(
        self, goal_handle
    ) -> Union[ControlPath.Result, TrackVisionTarget.Result]:
        """
        Executes the selected control action

        :param goal_handle: Action request
        :type goal_handle: ControlPath | TrackVisionTarget

        :return: Action result
        :rtype: Union[ControlPath.Result, TrackVisionTarget.Result]
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            return self._vision_tracking_callback(goal_handle)
        else:
            return self._path_tracking_callback(goal_handle)

    def reached_point(self, goal_point: Optional[RobotState]) -> bool:
        """
        Checks if the current robot state is close to a given goal point

        :param goal_point: Goal point
        :type goal_point: RobotState | None
        :param tolerance: Tolerance to goal
        :type tolerance: PathTrackingError

        :return: If the distance to the goal is less than the given tolerance
        :rtype: bool
        """
        if not self.robot_state or not self.__path_controller:
            return False
        if not goal_point:
            return True
        dist: float = self.robot_state.distance(goal_point)
        return dist <= self.__path_controller._config.goal_dist_tolerance

    def _execution_once(self):
        """Intialize controller post activation"""
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            # Setup the controller to avoid overhead when using the action server
            self.__vision_controller = self.__setup_vision_controller()

    def _execution_step(self):
        """
        Controller main execution step
        At each time step the controller:
        1- Updates all inputs
        2- Computes the commands using the specified algorithm
        3- Publish commands
        Robot is stopped when the end of the path is reached
        """
        self.get_logger().debug("In execution step")
        # PATH FOLLOWER MODE
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            # If vision mode is activated -> do nothing
            return

        if self.__reached_end:
            return
        try:
            status = self._path_control()
        except Exception as e:
            self.get_logger().error(
                f"Path control step failed for algorithm '{self.algorithm}': {e}"
            )
            self.health_status.set_fail_algorithm(algorithm_names=[self.algorithm])
            return

        if status == PathControlStatus.GOAL_REACHED:
            self.__reached_end = True
            plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
            if plan_callback:
                plan_callback.clear_last_msg()
        elif status == PathControlStatus.FAILED:
            self.health_status.set_fail_algorithm(
                algorithm_names=[str(ControlClasses[self.algorithm])]
            )
