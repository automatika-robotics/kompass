from typing import Optional, Union, List, Dict, Any
import time
from attrs import define, field, fields
from queue import Queue, Empty
import numpy as np

from rclpy.logging import get_logger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# ROS MSGS
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# KOMPASS
from kompass_core.models import Robot, RobotState
from ros_sugar.io import (
    LaserScanData,
    PointCloudCallback,
    LaserScanCallback,
    PointCloudData,
)
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

# KOMPASS MSGS/SRVS/ACTIONS
from .component import Component, TFListener
from ._modes import ControllerMode, FrameMode, PathControlStatus, CmdPublishType
from ._vision_follower import VisionFollower
from .utils import init_twist_array_msg
from .defaults import (
    controller_allowed_inputs,
    controller_allowed_outputs,
    controller_default_inputs,
    controller_default_outputs,
    TopicsKeys,
)


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
    _frame_mode: Union[str, FrameMode] = field(
        default=FrameMode.GLOBAL,
        converter=lambda value: FrameMode(value) if isinstance(value, str) else value,
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
        # Refresh only non-external callback slots; entries in _external_topics
        # were swapped by the robot plugin and must not be overwritten.
        for input in self.in_topics:
            if input.name in self._external_topics:
                continue
            self.callbacks[input.name] = input.msg_type.callback(
                input, node_name=self.node_name
            )
        # Create subscribers
        for callback in self.callbacks.values():
            # Callbacks are rebuilt above, so the resolvers behind
            # transform_inputs_to have to be re-attached to the new ones.
            # Frame handling applies to plugin-fed non-ROS inputs too
            self._attach_transform_provider(callback.input_topic.name, callback)

            # Inputs bound to a non-ROS robot plugin transport are fed through
            # the feedback bus, not a ROS subscription
            if callback.input_topic.name in self._external_topics:
                continue

            # In path follower mode -> skip all vision inputs
            if (
                self.config._mode == ControllerMode.PATH_FOLLOWER
                and callback.input_topic.name in self._vision_mode_inputs()
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
            self._cmd_execution_timer = self.create_timer(
                self.config.control_time_step,
                self._cmds_publishing_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        # Create timer to publish additional control tracking info
        self._info_publishing_timer = self.create_timer(
            1 / self.config.loop_rate,
            self._info_publishing_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def destroy_all_timers(self):
        """Overrides destroy_all_timers from BaseComponent to destroy the timers for commands execution and tracking publishing"""
        super().destroy_all_timers()
        # Destroy execution timer/rate
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_PARALLEL:
            self.destroy_timer(self._cmd_execution_timer)

        self.destroy_timer(self._info_publishing_timer)

    def create_all_services(self):
        """
        Creates all node services
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            self._vision_tracking_srv = self.create_service(
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
            self.destroy_service(self._vision_tracking_srv)
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
        if self._reached_end:
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
        if not self._path_controller:
            return None

        tracked_state: Optional[RobotState] = self._path_controller.tracked_state

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
        if self.algorithm == ControllersID.DWA and self._path_controller:
            kompass_cpp_path = self._path_controller.optimal_path()
            if not kompass_cpp_path:
                return None

        elif self.algorithm == ControllersID.VISION_DEPTH:
            kompass_cpp_path = self._vision_follower.optimal_path()
            if not kompass_cpp_path:
                return None

        # Only DWA and VISION_DEPTH provide a local plan for now, if other algorithm is selected return None
        else:
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
        if self.algorithm != ControllersID.DWA or not self._path_controller:
            return None

        # If result is not processed -> no debug is available yet
        if not self._path_controller.has_result():
            return

        (paths_x, paths_y) = self._path_controller.planner.get_debugging_samples()

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
        if not self._path_controller:
            return None
        kompass_cpp_path = self._path_controller.interpolated_path()
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
        # Reset sensor mount pose if the direct sensor flag is changed
        if value != self.config.use_direct_sensor:
            self._sensor_mount_pose_set = False

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
        return True

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
        # Path following always runs in the world frame; frame_mode is
        # only configurable in vision follower mode.
        self.config._frame_mode = FrameMode.GLOBAL
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

    @property
    def _sensor_tf_listener(self) -> Optional[TFListener]:
        """Transform listener from the proximity sensor frame to the robot body.

        Resolved from the sensor data's own frame, so it is None until the
        first message arrives. This is the mount pose the core needs to place a
        laser scan; it is not used to transform the data here.
        """
        return self.input_tf_listener(
            TopicsKeys.SPATIAL_SENSOR, self.config.frames.robot_base, static_tf=True
        )

    def _update_sensor_data(self) -> bool:
        """Update sensor data from the sensor callback.

        A laser scan is left in the sensor frame, since the core places it from
        the mount pose. Cartesian obstacles -- a point cloud or local map cells
        -- are asked for in the world frame, and dropped rather than handed over
        unconverted if they cannot be put there.

        :return: Whether the obstacle input could be delivered in the frame the
            core expects. False leaves the controller without obstacles for this
            step rather than placing them wrongly
        :rtype: bool
        """
        if self.direct_sensor:
            self.local_map = None
            sensor_callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
            if not sensor_callback:
                self.sensor_data = None
                return True

            if not isinstance(sensor_callback, PointCloudCallback):
                # A laser scan belongs in the sensor frame
                self.sensor_data = sensor_callback.get_output()
                return True

            deliverable, transform = self.resolve_input_tf(TopicsKeys.SPATIAL_SENSOR)
            if not deliverable:
                self.sensor_data = None
                self.get_logger().error(
                    f"Point cloud is published in the '{sensor_callback.frame_id}' "
                    f"frame but its transform to '{self.config.frames.world}' is "
                    "not available -> dropping it rather than placing obstacles "
                    "in the wrong frame",
                    throttle_duration_sec=5.0,
                )
                return False
            self.sensor_data = sensor_callback.get_output(transformation=transform)
            return True

        self.sensor_data = None
        map_callback = self.get_callback(TopicsKeys.LOCAL_MAP)
        if not map_callback:
            self.local_map = None
            self.local_map_resolution = None
            return True

        # Metadata is read off the message itself, so it needs no transform
        _metadata = map_callback.get_output(get_metadata=True)
        self.local_map_resolution = _metadata["resolution"] if _metadata else None

        deliverable, transform = self.resolve_input_tf(TopicsKeys.LOCAL_MAP)
        if not deliverable:
            self.local_map = None
            self.get_logger().error(
                f"Local map is published in the '{map_callback.frame_id}' frame "
                f"but its transform to '{self.config.frames.world}' is not "
                "available -> dropping it rather than placing obstacles in the "
                "wrong frame",
                throttle_duration_sec=5.0,
            )
            return False
        self.local_map: Optional[np.ndarray] = map_callback.get_output(
            transformation=transform
        )
        return True

    def _read_robot_state(self) -> Optional[RobotState]:
        """Helper method to read the robot state from the location topic"""
        state_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        if not state_callback:
            return None
        kw = {"get_front": True, "clear_last": True}
        kw["transformation"] = (
            self.odom_tf_listener.transform if self.odom_tf_listener else None
        )
        return state_callback.get_output(**kw)

    def _update_state(self, block: bool = True) -> None:
        """
        Updates node inputs from associated callbacks.

        :param block: If True, blocks up to ``topic_subscription_timeout`` waiting
            for a robot location message and then for the transform from its own
            frame to the world frame. Pass ``False`` from fast control loops to
            read the latest cached transform without blocking; if the TF is not
            yet available the update is skipped for this tick.
        """
        if self.config._mode == ControllerMode.PATH_FOLLOWER:
            self.plan: Optional[Path] = self._read_plan()

        # In LOCAL frame mode robot state is irrelevant: sensor data and
        # tracked targets are reasoned about robot-relative.
        if block and self.config._frame_mode == FrameMode.GLOBAL:
            timeout = 0.0
            step = 1 / self.config.loop_rate
            state_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)

            # The location frame is carried by the messages, so wait for one to
            # arrive before waiting for its transform. Both waits share a single
            # timeout budget.
            while (
                state_callback
                and not state_callback.got_msg
                and timeout < self.config.topic_subscription_timeout
            ):
                self.get_logger().warning(
                    "Waiting for a robot location message...", once=True
                )
                timeout += step
                time.sleep(step)

            if (
                state_callback
                and state_callback.got_msg
                and not state_callback.frame_id
            ):
                # A bare Pose carries no header, so there is no frame to look up
                # and nothing to transform: take the data as it arrived.
                self.get_logger().warning(
                    "Robot location messages carry no frame_id -> treating them as "
                    f"already expressed in the '{self.config.frames.world}' frame",
                    once=True,
                )
            else:
                odom_listener = self.odom_tf_listener
                while (
                    odom_listener
                    and not odom_listener.transform
                    and timeout < self.config.topic_subscription_timeout
                ):
                    self.get_logger().warning(
                        f"Waiting to get TF from {odom_listener.config.source_frame} "
                        f"frame to {self.config.frames.world} frame...",
                        once=True,
                    )
                    timeout += step
                    time.sleep(step)

                if not odom_listener or not odom_listener.transform:
                    self.get_logger().error(
                        "Could not get TF from the robot location frame to "
                        f"'{self.config.frames.world}' frame after "
                        f"{self.config.topic_subscription_timeout} seconds"
                    )
                    return
        self.robot_state = self._read_robot_state()
        if block:
            waited = 0.0
            while (
                not self.robot_state and waited < self.config.topic_subscription_timeout
            ):
                time.sleep(1 / self.config.loop_rate)
                waited += 1 / self.config.loop_rate
                self.robot_state = self._read_robot_state()

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

    def _read_plan(self) -> Optional[Path]:
        """Read the global plan, in the world frame the core tracks against.

        The single place a plan is fetched, so every consumer gets the same
        guarantee: either the plan is in the world frame, or there is no plan.

        :return: The plan in the world frame, or None if there is none or it
            cannot be brought there yet
        :rtype: Optional[Path]
        """
        plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
        if not plan_callback:
            return None

        deliverable, transform = self.resolve_input_tf(TopicsKeys.GLOBAL_PLAN)
        if not deliverable:
            self.get_logger().error(
                f"Global plan is published in the '{plan_callback.frame_id}' "
                f"frame but its transform to '{self.config.frames.world}' is "
                "not available -> dropping this plan",
                throttle_duration_sec=5.0,
            )
            return None
        return plan_callback.get_output(transformation=transform)

    def _set_path_to_controller(self, output, **_) -> None:
        """
        Set a new plan to the controller/follower

        The plan arrives already expressed in the world frame: the component
        asks for this input in ``frames.world`` and the callback transforms it.
        """
        deliverable, _ = self.resolve_input_tf(TopicsKeys.GLOBAL_PLAN)
        if output is None or not deliverable:
            self.get_logger().error(
                "Global plan cannot be expressed in the "
                f"'{self.config.frames.world}' frame -> dropping this plan",
                throttle_duration_sec=5.0,
            )
            return
        self._install_plan(output)

    def _install_plan(self, plan: Path) -> bool:
        """Hand a world-frame plan to the core and take the goal point from it.

        Shared by the arrival hook and the retry in ``_path_control`` so the
        two cannot drift: a plan that reaches the core must always bring its
        goal point with it, or ``reached_point`` judges against a stale one.

        :param plan: Global plan, already in the world frame
        :type plan: Path

        :return: Whether the core came away with a path to track
        :rtype: bool
        """
        self.plan = plan
        self._reached_end = False

        if len(plan.poses) > 1:
            self._goal_point = RobotState(
                x=plan.poses[-1].pose.position.x, y=plan.poses[-1].pose.position.y
            )
        else:
            # Fewer than two poses is "no goal"
            self._goal_point = None

        if self._path_controller is None:
            return False

        # Handed over whatever its length: the core clears its own current path
        # when given fewer than two poses, and that is the only way a stale
        # path gets dropped. Skipping the call leaves the robot tracking it
        self._path_controller.set_path(global_path=plan)
        return bool(self._path_controller.path)

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        # reached end path / end of control action
        self._reached_end: bool = False

        self.robot_state: Optional[RobotState] = (
            None  # robot current state - to be updated from odom
        )
        self.plan: Optional[Path] = None  # robot plan (global path)

        # The plan is tracked against the robot state, which is brought into
        # the world frame, so the two have to agree. The plan carries its own
        # frame in its messages, so nothing has to be configured
        self.transform_inputs_to(TopicsKeys.GLOBAL_PLAN, self.config.frames.world)

        # INIT PATH CONTROLLER
        self._robot = Robot(
            robot_type=self.robot.model_type,
            geometry_type=self.robot_geometry_type,
            geometry_params=self.robot.geometry_params,
            state=self.robot_state or RobotState(),
        )

        # SET robot control limits
        self._robot_ctr_limits = self.robot_ctrl_limits

        self._reached_end = False
        self._lat_dist_error: float = 0.0
        self._ori_error: float = 0.0

        # Vision tracking lifecycle helper (owns _vision_controller, detections,
        # depth image and tracked-target state). Always instantiated; dormant
        # in path follower mode.
        self._vision_follower = VisionFollower(self)

        # Command queue to send controller command list to the robot
        self._cmds_queue: Queue = Queue()

        self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = None

        # Obstacle inputs are handed to the core in the frame it expects for
        # that input type, and the core does the rest:
        #
        #  - a laser scan stays in the **sensor frame**. The core lifts it into
        #    the world itself, from the mount pose it is given at construction
        #    and the robot's world pose. So no transform is requested here.
        #  - cartesian obstacles, whether a decoded point cloud or local map
        #    cells, are handed over in the **world frame** and used untouched.
        #
        # Both are registered regardless of `use_direct_sensor`, since it can be
        # flipped at runtime and only the subscribed input ever resolves one.
        if isinstance(self.get_callback(TopicsKeys.SPATIAL_SENSOR), PointCloudCallback):
            self.transform_inputs_to(
                TopicsKeys.SPATIAL_SENSOR, self.config.frames.world
            )
        self.transform_inputs_to(TopicsKeys.LOCAL_MAP, self.config.frames.world)

        self._path_controller: Optional[ControllerType] = None
        # The sensor mount pose is only knowable once a scan has arrived, and
        # the core takes it at construction -> see _apply_sensor_mount_pose
        self._sensor_mount_pose_set: bool = False

        if self.config._mode == ControllerMode.PATH_FOLLOWER:
            self._build_path_controller()

    def _build_path_controller(self, **config_kwargs) -> None:
        """(Re)build the core algorithm object.

        :param config_kwargs: Algorithm config overrides, used to pass the
            sensor mount pose once it is known
        """
        config_class = ControlConfigClasses[self.algorithm]
        # Check for acceptance since the config classes do not all share the same common set of fields
        accepted = {attribute.name.lstrip("_") for attribute in fields(config_class)}
        dropped = set(config_kwargs) - accepted
        if dropped:
            self.get_logger().debug(
                f"'{self.algorithm}' config takes no {sorted(dropped)} -> ignored"
            )
        # Get default controller configuration and update it from user defined config
        _controller_config = self._configure_algorithm(config_class())

        # Update with given config arguments, if any.
        # This is used to pass the sensor mount pose once it is known.
        for name, value in config_kwargs.items():
            if name in accepted:
                setattr(_controller_config, name, value)

        self._path_controller = ControlClasses[self.algorithm](
            robot=self._robot,
            config=_controller_config,
            ctrl_limits=self._robot_ctr_limits,
            config_file=self._config_file,
            config_root_name=f"{self.node_name}.{self.config.algorithm}",
            control_time_step=self.config.control_time_step,
        )

    def _apply_sensor_mount_pose(self) -> None:
        """Hand the sensor-to-body mount pose to the core, once it is known.

        Only a laser scan needs it: it reaches the core in the sensor frame, so
        the core cannot place it without the mount pose. The pose comes from the
        scan's own ``frame_id``, so it cannot be resolved before the first
        message, and the core takes it at construction -- hence the one-time
        rebuild here rather than at ``init_variables`` time.
        """
        if self._sensor_mount_pose_set or self._path_controller is None:
            return

        if not self.direct_sensor or not isinstance(
            self.get_callback(TopicsKeys.SPATIAL_SENSOR), LaserScanCallback
        ):
            # If a map is being used or direct sensor data is not laserscan, the core does not need a mount pose and can be built once at init time
            self._sensor_mount_pose_set = True
            return

        sensor_tf = self._sensor_tf_listener
        if not sensor_tf or not sensor_tf.got_transform:
            self.get_logger().warning(
                "Waiting for the transform from the laser scan frame to "
                f"'{self.config.frames.robot_base}' to place the scan...",
                throttle_duration_sec=5.0,
            )
            return

        self._build_path_controller(
            proximity_sensor_position_to_robot=sensor_tf.translation,
            proximity_sensor_rotation_to_robot=sensor_tf.rotation,
        )
        self._sensor_mount_pose_set = True
        # The rebuild dropped the path the previous object was tracking.
        # `self.plan` is the guarded world-frame read refreshed by
        # `_update_state` just above, so this cannot install a wrong-frame path
        if self.plan is not None:
            self._path_controller.set_path(global_path=self.plan)
        else:
            self.get_logger().warning(
                "No global plan available while applying the scan mount pose "
                "-> the rebuilt controller starts without a path"
            )

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
        if self._path_controller is None:
            return PathControlStatus.IDLE

        if not self._path_controller.path:
            plan = self._read_plan()
            # No plan is set to the controller -> read plan from callback
            if (plan is None) or (not self._install_plan(plan)):
                # Plan is not available or rejected by the core, which needs at least two poses
                return PathControlStatus.IDLE

        self._update_state(block=True)
        obstacles_deliverable = self._update_sensor_data()
        self._apply_sensor_mount_pose()

        if not obstacles_deliverable or not self.robot_state:
            self.get_logger().warning(
                            f"State or sensor data unavailable after {self.config.topic_subscription_timeout}s -> skipping control step",
                            throttle_duration_sec=5.0,
                        )
            # Obstacles exist but could not be put in the frame the core reads
            return PathControlStatus.WAITING_INPUTS

        ranges = None
        angles = None
        points = None
        local_map = None

        if self.direct_sensor:
            if isinstance(self.sensor_data, LaserScanData):
                ranges = self.sensor_data.ranges
                angles = self.sensor_data.angles
            elif isinstance(self.sensor_data, PointCloudData):
                # The controller works on cartesian obstacle points, decoded
                # from the raw buffer by the container and cached there
                points = self.sensor_data.xyz
        else:
            local_map = self.local_map

        if self.reached_point(self._goal_point):
            self._stop_robot()
            return PathControlStatus.GOAL_REACHED

        cmd_found: bool = self._path_controller.loop_step(
            current_state=self.robot_state,  # type: ignore
            ranges=ranges,
            angles=angles,
            points=points,
            local_map=local_map,
            local_map_resolution=getattr(self, "local_map_resolution", None),
            debug=self.config.debug,
        )

        # LOG CONTROLLER INFO
        self.get_logger().debug(f"{self._path_controller.logging_info()}")

        if not cmd_found:
            return PathControlStatus.FAILED

        self.health_status.set_healthy()

        # Update controller path tracking info (errors)
        self._lat_dist_error = self._path_controller.distance_error
        self._ori_error = self._path_controller.orientation_error

        self._publish(
            self._path_controller.linear_x_control,
            self._path_controller.linear_y_control,
            self._path_controller.angular_control,
        )

        return PathControlStatus.RUNNING

    def _vision_mode_inputs(self) -> List[str]:
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
        """Service callback to end Vision target tracking action."""
        response.success = self._vision_follower.request_stop()
        return response

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

            # A different algorithm means a fresh object, so the mount pose has
            # to be handed to it again on the next control step
            self._sensor_mount_pose_set = False
            self._build_path_controller()
            self.get_logger().info(f"Initialized '{self.algorithm}' controller")
        else:
            self.get_logger().warning(
                f"No Algorithm is provided in control action request -> Using node configured algorithm '{self.algorithm}'"
            )

        # Check if inputs are available until timeout
        if not self.callbacks_inputs_check(
            inputs_to_exclude=self._vision_mode_inputs()
        ):
            self.get_logger().error(
                "Requested action inputs are not available -> Aborting"
            )
            goal_handle.abort()
            return result

        # Set current mode to path following
        # Note: This will automatically end the vision target tracking action if it is ongoing
        self.config._mode = ControllerMode.PATH_FOLLOWER

        deliverable, _ = self.resolve_input_tf(TopicsKeys.GLOBAL_PLAN)
        if not deliverable:
            self.get_logger().error(
                "Global plan is not available in the world frame -> Aborting"
            )
            goal_handle.abort()
            return result
        # Refresh where possible: in ACTION_SERVER mode nothing has run
        # _update_state yet, so `self.plan` may be stale at this point
        if (_plan := self._read_plan()) is not None:
            self.plan = _plan
        self._path_controller.set_path(self.plan)  # type: ignore

        self._reached_end: bool = False

        while True:
            status: PathControlStatus = self._path_control()

            if status == PathControlStatus.GOAL_REACHED:
                self._reached_end = True
                break
            if status in (PathControlStatus.FAILED, PathControlStatus.IDLE):
                break

            # RUNNING or WAITING_INPUTS: keep the action alive, publish feedback
            # TODO: Add an option to update the computation time using timeit and send it in feedback, for controller performance tracking
            feedback_msg.control_list = init_twist_array_msg(
                number_of_cmds=len(self._path_controller.linear_x_control),
                linear_x=self._path_controller.linear_x_control,
                linear_y=self._path_controller.linear_y_control,
                angular=self._path_controller.angular_control,
            )

            feedback_msg.global_path_deviation.lateral_distance_error = (
                self._lat_dist_error
            )
            feedback_msg.global_path_deviation.orientation_error = self._ori_error

            self.get_logger().info(
                "Controlling Path — lat_err=%.3f, ori_err=%.3f"
                % (self._lat_dist_error, self._ori_error)
            )

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(1 / self.config.loop_rate)

        if self._reached_end:
            # WHEN PATH TRACKER SERVICE RETURNS RESULT
            self.get_logger().info("Reached end of path!")
            # Update the result msg
            result.destination_error.lateral_distance_error = self._lat_dist_error
            result.destination_error.orientation_error = self._ori_error
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
            return self._vision_follower.execute_action(goal_handle)
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
        if not self.robot_state or not self._path_controller:
            return False
        if not goal_point:
            return True
        dist: float = self.robot_state.distance(goal_point)
        return dist <= self._path_controller._config.goal_dist_tolerance

    def _execute_once(self):
        """Initialize controller post activation"""
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            # Set up the vision controller eagerly to avoid first-call overhead
            # in the action server.
            self._vision_follower.setup()

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

        if self._reached_end:
            return
        try:
            status = self._path_control()
        except Exception as e:
            self.get_logger().error(
                f"Path control step failed for algorithm '{self.algorithm}': {e}"
            )
            self.health_status.set_fail_algorithm(
                algorithm_names=[self.algorithm.value]
            )
            return

        if status == PathControlStatus.GOAL_REACHED:
            self._reached_end = True
            plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
            if plan_callback:
                plan_callback.clear_last_msg()
        elif status == PathControlStatus.FAILED:
            self.health_status.set_fail_algorithm(
                algorithm_names=[str(ControlClasses[self.algorithm])]
            )
