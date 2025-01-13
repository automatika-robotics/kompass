from typing import Optional, Union, List, Dict
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
from kompass_core.datatypes import LaserScanData, PointCloudData, TrackingData
from kompass_core.utils.geometry import from_euler_to_quaternion
from kompass_core.control import (
    ControlClasses,
    ControlConfigClasses,
    ControllersID,
    ControllerType,
)
from kompass_core.control.vision_follower import VisionFollower, VisionFollowerConfig

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

    * - **prediction_horizon**
      - `float`, `1.0`
      - Prediction horizon in MPC-like controllers (s)

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
        converter=lambda value: ControllersID(value)
        if isinstance(value, str)
        else value,
    )
    control_time_step: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Time step between two control commands
    prediction_horizon: float = field(
        default=1.0, validator=BaseValidators.in_range(min_value=1, max_value=1e9)
    )  # future time horizon to compute at each loop step
    use_direct_sensor: bool = field(default=False)
    ctrl_publish_type: Union[str, CmdPublishType] = field(
        default=CmdPublishType.TWIST_ARRAY,
        converter=lambda value: CmdPublishType(value)
        if isinstance(value, str)
        else value,
    )
    _mode: Union[str, ControllerMode] = field(
        default=ControllerMode.PATH_FOLLOWER,
        converter=lambda value: ControllerMode(value)
        if isinstance(value, str)
        else value,
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
    Set from ControllerConfig class or directly from Controller 'run_type' property.

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

    def create_all_action_servers(self):
        """
        Action servers creation
        """
        if (
            self.config._mode == ControllerMode.VISION_FOLLOWER
            and not self.get_in_topic(TopicsKeys.VISION_TRACKINGS)
        ):
            # Vision mode is set but the trackings topic is not provided
            raise AttributeError(
                f"Cannot use Vision Follower in {self.node_name} component without setting the 'TopicsKeys.VISION_TRACKINGS' input"
            )

        elif self.config._mode == ControllerMode.VISION_FOLLOWER:
            self.action_type = TrackVisionTarget
            self._deactivate_follower_mode()
            self._activate_vision_mode()
        else:
            self.action_type = ControlPath
            self._deactivate_vision_mode()
            self._activate_follower_mode()

        super().create_all_action_servers()

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
                self._execution_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        elif self.config.ctrl_publish_type == CmdPublishType.TWIST_SEQUENCE:
            self.get_logger().debug(
                f"Creating execution rate with freq: {1 / self.config.control_time_step}"
            )
            # Create rate to publish in sequence during control
            self.__cmd_execution_rate = self.create_rate(
                1 / self.config.control_time_step
            )
        # Create timer to publish additional control tracking info
        self.__info_publishing_timer = self.create_timer(
            1 / self.config.loop_rate,
            self._publishing_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def destroy_all_timers(self):
        """Overrides destroy_all_timers from BaseComponent to destroy the timers for commands execution and tracking publishing"""
        super().destroy_all_timers()
        # Destroy execution timer/rate
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_PARALLEL:
            self.destroy_timer(self.__cmd_execution_timer)
        elif self.config.ctrl_publish_type == CmdPublishType.TWIST_SEQUENCE:
            self.destroy_rate(self.__cmd_execution_rate)

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

    def _execution_callback(self):
        """Commands execution timer callback"""
        try:
            cmd = self._cmds_queue.get_nowait()
        except Empty:
            self.get_logger().debug("No command to execute")
            return
        # If end is reached do not publish new command
        if self.__reached_end:
            self.get_logger().debug("No command to execute")
            return

        self._publish_one_control(
            linear_ctr_x=cmd[0],
            linear_ctr_y=cmd[1],
            angular_ctr=cmd[2],
        )

    def _publishing_callback(self):
        """Tracking publishing timer callback"""
        # Publish tracked point on the global path
        if (
            self.tracked_point is not None
            and self.config._mode == ControllerMode.PATH_FOLLOWER
        ):
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

        # Publish interpolated path
        if self.interpolated_path:
            self.get_publisher(TopicsKeys.INTERPOLATED_PATH).publish(
                self.interpolated_path,
                frame_id=self.config.frames.world,
                time_stamp=self.get_ros_time(),
            )

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
        for point in kompass_cpp_path.points:
            ros_point = PoseStamped()
            ros_point.pose.position.x = point.x
            ros_point.pose.position.y = point.y
            parsed_points.append(ros_point)

        ros_path.poses = parsed_points
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
            ros_point.pose.position.x = point.x
            ros_point.pose.position.y = point.y
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
        if self.algorithm != ControllersID.DWA and not value:
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
        if value in [ControllersID.VISION, ControllersID.VISION.value]:
            self.config._mode = ControllerMode.VISION_FOLLOWER
            self.run_type = ComponentRunType.ACTION_SERVER
        else:
            self.config._mode = ControllerMode.PATH_FOLLOWER

    @component_action
    def set_algorithm(
        self,
        algorithm_value: Union[str, ControllersID],
    ) -> bool:
        """
        Component action - Set controller algorithm action

        :param algorithm_value: algorithm value
        :type algorithm_value: Union[str, ControllersID]

        :raises Exception: Exception while updating algorithm value

        :return: Success
        :rtype: bool
        """
        try:
            self.stop()
            self.algorithm = algorithm_value
            self.start()
        except Exception:
            raise
        return True

    def _activate_vision_mode(self):
        """Activate object following mode using vision detections"""
        # Activate vision subscriber
        if not self.in_topic_name(TopicsKeys.VISION_TRACKINGS):
            raise ValueError(
                f"Error activating vision tracking mode. No input topic is provided for '{TopicsKeys.VISION_TRACKINGS}'"
            )

        callback = self.get_callback(TopicsKeys.VISION_TRACKINGS)
        if callback:
            callback.set_subscriber(self._add_ros_subscriber(callback))

    def _deactivate_vision_mode(self):
        """Deactivate object following mode using vision detections"""
        # Delete vision subscriber if exists
        try:
            import logging

            logging.info("deactivating vision")
            callback = self.get_callback(TopicsKeys.VISION_TRACKINGS)
            logging.info(f"got callback {callback}")
            if callback and callback._subscriber:
                self.destroy_subscription(callback._subscriber)
                callback._subscriber = None
        except Exception:
            # Vision mode is not active
            return

    def _activate_follower_mode(self):
        """Activate path following mode by creating all missing subscriptions"""
        # Create all path following subscriptons if not available
        for callback in self.callbacks.values():
            if (
                callback.input_topic.name
                != self.in_topic_name(TopicsKeys.VISION_TRACKINGS)
                and not callback._subscriber
            ):
                callback.set_subscriber(self._add_ros_subscriber(callback))

    def _deactivate_follower_mode(self):
        """Deactivate path following mode by removing all missing subscriptions"""
        # Remove all path subscriptons if exists
        for callback in self.callbacks.values():
            if (
                callback.input_topic.name
                != self.in_topic_name(TopicsKeys.VISION_TRACKINGS)
                and callback._subscriber
            ):
                self.destroy_subscription(callback._subscriber)
                callback._subscriber = None

    def _update_state(self, vision_track_id: Optional[int] = None) -> None:
        """
        Updates node inputs from associated callbacks
        """
        if self.config._mode == ControllerMode.VISION_FOLLOWER and self.__tracked_label:
            # If vision mode is ON and a label is getting tracked
            vision_callback = self.get_callback(TopicsKeys.VISION_TRACKINGS)
            self.vision_trackings = (
                vision_callback.get_output(
                    label=self.__tracked_label, id=vision_track_id, clear_last=True
                )
                if vision_callback
                else None
            )
            return

        plan_callback = self.get_callback(TopicsKeys.GLOBAL_PLAN)
        self.plan: Optional[Path] = (
            plan_callback.get_output() if plan_callback else None
        )

        state_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        self.robot_state: Optional[RobotState] = (
            state_callback.get_output(
                transformation=self.odom_tf_listener.transform
                if self.odom_tf_listener
                else None
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
            self.local_map: Optional[np.ndarray] = (
                map_callback.get_output() if map_callback else None
            )

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
            # Remove callback for the local map and destroy subscriber
            _callback = self.callbacks.pop(self.in_topic_name(TopicsKeys.LOCAL_MAP))
            if _callback._subscriber:
                self.destroy_subscription(_callback._subscriber)

            # If direct sensor information is used set maximum range for PointCloud data
            sensor_callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR)
            if isinstance(sensor_callback, PointCloudCallback):
                sensor_callback.max_range = (
                    self.config.prediction_horizon * self.robot.ctrl_vx_limits.max_vel
                )
                self.get_logger().info(
                    f"Setting PointCloud max range to robot max forward horizon '{sensor_callback.max_range}' to limit computations"
                )
        else:
            # Remove callback for the sensor data and destroy subscriber
            _callback = self.callbacks.pop(
                self.in_topic_name(TopicsKeys.SPATIAL_SENSOR)
            )
            if _callback._subscriber:
                self.destroy_subscription(_callback._subscriber)

    def _set_path_to_controller(self, msg, **_) -> None:
        """
        Set a new plan to the controller/follower
        """
        self.__reached_end = False
        if self.__path_controller:
            self.__path_controller.set_path(global_path=msg)
        self.__goal_point = RobotState(
            x=msg.poses[-1].pose.position.x, y=msg.poses[-1].pose.position.y
        )

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

        self.__path_controller: Optional[ControllerType] = None

        if self.config._mode == ControllerMode.PATH_FOLLOWER:
            # Get default controller configuration and update it from user defined config
            _controller_config = self._configure_algorithm(
                ControlConfigClasses[self.algorithm]()
            )

            self.__path_controller = ControlClasses[self.algorithm](
                robot=self.__robot,
                config=_controller_config,
                ctrl_limits=self.__robot_ctr_limits,
                config_file=self._config_file,
                config_yaml_root_name=f"{self.node_name}.{self.config.algorithm}",
                control_time_step=self.config.control_time_step,
                prediction_horizon=self.config.prediction_horizon,
            )

        self.__reached_end = False
        self.__lat_dist_error: float = 0.0
        self.__ori_error: float = 0.0
        self.__tracked_label: Optional[str] = None

        # Command queue to send controller command list to the robot
        self._cmds_queue: Queue = Queue()

        if self.direct_sensor:
            sensor_topic = self._inputs_list[
                self._inputs_keys.index(TopicsKeys.SPATIAL_SENSOR)
            ]
            # Setup transform listener
            self.__sensor_tf_listener: TFListener = (
                self.depth_tf_listener
                if sensor_topic.msg_type._ros_type == PointCloud2
                else self.scan_tf_listener
            )

            self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = None

        self.vision_trackings: Optional[TrackingData] = None

        self._attach_callbacks()

    def _stop_robot(self):
        """
        Publishes a zero velocity command to stop the robot
        """
        # send zero command to stop the robot
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_ARRAY:
            _cmd_vel_array = init_twist_array_msg(
                int(self.config.prediction_horizon / self.config.control_time_step)
            )
            self.get_publisher(TopicsKeys.INTERMEDIATE_CMD_LIST).publish(_cmd_vel_array)
        else:
            self.get_publisher(TopicsKeys.INTERMEDIATE_CMD).publish(Twist())

    def _publish(
        self,
        commands_vx: List[float],
        commands_vy: List[float],
        commands_omega: List[float],
    ):
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_ARRAY:
            # publish a Twist Array
            self._publish_multi_control(
                linear_ctr_x=commands_vx,
                linear_ctr_y=commands_vy,
                angular_ctr=commands_omega,
            )
            return

        # Empty the commands queue
        self._cmds_queue.queue.clear()

        # Put new control commands to the queue
        [
            self._cmds_queue.put(i)
            for i in zip(
                commands_vx,
                commands_vy,
                commands_omega,
            )
        ]

        # If commands are to be published in sequence
        if self.config.ctrl_publish_type == CmdPublishType.TWIST_SEQUENCE:
            # While queue is not empty
            while self._cmds_queue.qsize() > 0:
                self._execution_callback()
                self.__cmd_execution_rate.sleep()

    def _publish_one_control(
        self, linear_ctr_x: float, linear_ctr_y: float, angular_ctr: float
    ):
        """
        Publish a Twist command to ROS

        :param linear_ctr_x: Linear control command (m/s)
        :type linear_ctr_x: float
        :param linear_ctr_y: Linear control command (m/s)
        :type linear_ctr_y: float
        :param angular_ctr: Angular control command (rad/s)
        :type angular_ctr: float
        """
        _cmd_vel = Twist()
        _cmd_vel.linear.x = linear_ctr_x
        _cmd_vel.linear.y = linear_ctr_y
        _cmd_vel.angular.z = angular_ctr

        self.get_publisher(TopicsKeys.INTERMEDIATE_CMD).publish(_cmd_vel)

    def _publish_multi_control(
        self,
        linear_ctr_x: List[float],
        linear_ctr_y: List[float],
        angular_ctr: List[float],
    ):
        """
        Publish a set of command (TwistArray) to ROS

        :param linear_ctr_x: Linear control commands (m/s)
        :type linear_ctr_x: List[float]
        :param linear_ctr_y: Linear control commands (m/s)
        :type linear_ctr_y: List[float]
        :param angular_ctr: Angular control commands (rad/s)
        :type angular_ctr: List[float]
        """
        _cmd_vel_array = init_twist_array_msg(
            number_of_cmds=len(linear_ctr_x),
            linear_x=linear_ctr_x,
            linear_y=linear_ctr_y,
            angular=angular_ctr,
        )
        _cmd_vel_array.time_step = self.config.control_time_step

        self.get_publisher(TopicsKeys.INTERMEDIATE_CMD_LIST).publish(_cmd_vel_array)

    def _path_control(self) -> bool:
        """
        Gets robot control command using reference commands

        :return: If robot reached end of reference plan
        :rtype: bool
        """
        if not self.__path_controller:
            self.get_logger().warning("Path controller is not defined")
            return False

        if not self.__path_controller.path:
            self.get_logger().warning("Global plan is not available to controller")
            return False

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

        self.__reached_end = self.reached_point(self.__goal_point)

        # If end is reached stop the robot
        if self.__reached_end:
            # Stop robot
            self._stop_robot()
            # Clear path
            self.get_callback(TopicsKeys.GLOBAL_PLAN).clear_last_msg()
            # Unset reached_end for new incoming paths
            self.__reached_end = False
            return True

        cmd_found: bool = self.__path_controller.loop_step(
            current_state=self.robot_state,  # type: ignore
            laser_scan=laser_scan,
            point_cloud=point_cloud,
            local_map=local_map,
        )

        # LOG CONTROLLER INFO
        self.get_logger().debug(f"{cmd_found}: {self.__path_controller.logging_info()}")

        # PUBLISH CONTROL TO ROBOT CMD TOPIC
        if not cmd_found:
            self.get_logger().warn("Control command not found")
            self.health_status.set_fail_algorithm(
                algorithm_names=[str(ControlClasses[self.algorithm])]
            )
            self.__reached_end = False
            return False

        self.health_status.set_healthy()

        # Update controller path tracking info (errors/end_reached)
        self.__lat_dist_error = self.__path_controller.distance_error
        self.__ori_error = self.__path_controller.orientation_error

        self._publish(
            self.__path_controller.linear_x_control,
            self.__path_controller.linear_y_control,
            self.__path_controller.angular_control,
        )

        return True

    def _end_vision_tracking_srv_callback(
        self, request: StopVisionTracking.Request, response: StopVisionTracking.Response
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
            or not self.__tracked_label
        ):
            response.success = True
            return response

        # There is a label in the request and it is not being tracked
        if request.label != "" and self.__tracked_label != request.label:
            response.success = True
            return response

        self.get_logger().warn(
            f"Ending Vision Tracking Action for tracked label '{self.__tracked_label}'"
        )

        # Wait for the action server to unset the tracked label
        _counter: int = 0
        while self.__tracked_label and _counter < 10:
            time.sleep(1 / self.config.loop_rate)
            if not self.__tracked_label:
                response.success = True
                return response

        response.success = False
        return response

    def __wait_for_trackings(
        self, tracked_id: Optional[int] = None, max_wait_time: Optional[float] = None
    ):
        """Wait to receive vision_trackings or until timeout

        :param tracked_id: Check if id is available in trackings, defaults to None
        :type tracked_id: Optional[int], optional
        """
        self._update_state(vision_track_id=tracked_id)
        max_wait_time = max_wait_time or self.config.topic_subscription_timeout
        wait_time = 0.0
        while not self.vision_trackings and wait_time <= max_wait_time:
            self.get_logger().info(
                f"Waiting for vision target information, timeout in {round(max_wait_time, 2)}s...",
                once=True,
            )
            self._update_state(vision_track_id=tracked_id)
            wait_time += 1 / self.config.loop_rate
            time.sleep(1 / self.config.loop_rate)

    def _vision_tracking_callback(self, goal_handle) -> TrackVisionTarget.Result:
        """Vision target tracking action callback

        :param goal_handle: Vision target tracking action goal message
        :type goal_handle: TrackVisionTarget.Goal
        :return: Vision target tracking action result
        :rtype: TrackVisionTarget.Result
        """
        self.get_logger().info(
            f"Started vision tracking control action... {self.config.control_time_step}"
        )

        # SETUP ACTION REQUEST/FEEDBACK/RESULT
        request_msg = goal_handle.request

        self.__tracked_label: Optional[str] = request_msg.label

        feedback_msg = TrackVisionTarget.Feedback()

        result = TrackVisionTarget.Result()

        # Wait to get the first tracking
        self.__wait_for_trackings()

        if not self.vision_trackings:
            self.get_logger().error(
                f"Tracking information is not available for requested label '{self.__tracked_label}' -> Aborting"
            )
            goal_handle.abort()
            return result

        # Get ID of the label from the current detection and set as the tracked ID
        _tracked_id = self.vision_trackings.id

        config = VisionFollowerConfig(
            control_time_step=self.config.control_time_step,
            target_distance=self.vision_trackings.depth,
            target_search_timeout=int(
                request_msg.search_timeout / self.config.control_time_step
            ),
            target_search_radius=request_msg.search_radius,
        )
        config = self._configure_algorithm(config)

        _controller = VisionFollower(
            robot=self.__robot,
            ctrl_limits=self.__robot_ctr_limits,
            config=config,
            config_file=self._config_file,
            config_yaml_root_name=f"{self.node_name}.VisionFollower",
        )

        # time the tracking period
        start_time = time.time()

        # While the vision tracking mode is not unset by a service call or an event action
        while self.config._mode == ControllerMode.VISION_FOLLOWER:
            # Wait to get tracking
            self.__wait_for_trackings(
                _tracked_id,
                max_wait_time=config.target_search_pause * config.control_time_step,
            )

            found_ctrl = _controller.loop_step(tracking=self.vision_trackings)

            if not found_ctrl:
                self.get_logger().info(
                    "Unable to find tracked target -> Aborting action"
                )
                goal_handle.abort()
                return result

            # Publish feedback
            feedback_msg.center_xy = (
                self.vision_trackings.center_xy if self.vision_trackings else [0.0, 0.0]
            )
            feedback_msg.size_xy = (
                self.vision_trackings.size_xy if self.vision_trackings else [0.0, 0.0]
            )
            feedback_msg.distance = 0.0  # TODO
            feedback_msg.orientation = 0.0  # TODO

            # Publish control
            self._publish(
                _controller.linear_x_control,
                _controller.linear_y_control,
                _controller.angular_control,
            )
            if self.vision_trackings:
                self.get_logger().info(
                    f"Following tracked target: {_controller.linear_x_control}, {_controller.angular_control}"
                )
                self.get_logger().info(f"tracked target: {self.vision_trackings}")
            else:
                self.get_logger().info(
                    f"Searching for tracked target with control: {_controller.linear_x_control}, {_controller.angular_control}"
                )

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(self.config.control_time_step)

        end_time = time.time()

        if self.vision_trackings:
            # WHEN PATH TRACKER SERVICE RETURNS RESULT
            self.get_logger().warning("Ending tracking action")
            # Update the result msg
            result.success = True
            result.tracked_duration = end_time - start_time
            goal_handle.succeed()

        else:
            self.get_logger().warning(
                "Vision target is lost -> Aborting tracking action"
            )
            result.success = False
            result.tracked_duration = end_time - start_time
            goal_handle.abort()

        # Unset the tracked label
        self.__tracked_label = None

        self._stop_robot()

        return result

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
                config_yaml_root_name=f"{self.node_name}.{self.config.algorithm}",
            )
            self.get_logger().info(f"Initialized '{self.algorithm}' controller")
        else:
            self.get_logger().warn(
                f"No Algorithm is provided in control action request -> Using node configured algorithm '{self.algorithm}'"
            )

        # Check if inputs are available until timeout
        if not self.callbacks_inputs_check(
            inputs_to_exclude=[self.in_topic_name(TopicsKeys.VISION_TRACKINGS)]
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

        while not self.__reached_end:
            self._update_state()

            cmd_found: bool = self._path_control()

            if not cmd_found:
                break

            # Send controller feedback
            # TODO: Add an option to update the computation time using timeit and send it in feedback, for controller performance tracking
            feedback_msg.control_list = init_twist_array_msg(
                number_of_cmds=len(self.__path_controller.linear_x_control),
                linear_x=self.__path_controller.linear_x_control,
                linear_y=self.__path_controller.linear_x_control,
                angular=self.__path_controller.linear_x_control,
            )

            feedback_msg.global_path_deviation.orientation_error = self.__lat_dist_error
            feedback_msg.global_path_deviation.lateral_distance_error = self.__ori_error
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
            result.destination_error.orientation_error = self.__lat_dist_error
            result.destination_error.lateral_distance_error = self.__ori_error

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

    def reached_point(self, goal_point: RobotState) -> bool:
        """
        Checks if the current robot state is close to a given goal point

        :param goal_point: Goal point
        :type goal_point: RobotState
        :param tolerance: Tolerance to goal
        :type tolerance: PathTrackingError

        :return: If the distance to the goal is less than the given tolerance
        :rtype: bool
        """
        if not self.robot_state or not self.__path_controller:
            return False
        dist: float = self.robot_state.distance(goal_point)
        return dist <= self.__path_controller._config.goal_dist_tolerance

    def _execution_step(self):
        """
        Controller main execution step
        At each time step the controller:
        1- Updates all inputs
        2- Computes the commands using the specified algorithm
        3- Publish commands
        Robot is stopped when the end of the path is reached
        """
        # PATH FOLLOWER MODE
        if self.config._mode == ControllerMode.VISION_FOLLOWER:
            # If vision mode is activated -> do nothing
            return

        # Get inputs from callbacks
        self._update_state()

        got_all: bool = self.got_all_inputs()

        # Check if all inputs are available
        if got_all and not self.__reached_end:
            self._path_control()
