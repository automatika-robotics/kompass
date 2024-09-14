from typing import Optional, Union
from attrs import define, field
from queue import Queue, Empty
import numpy as np

from rclpy.logging import get_logger
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# ROS MSGS
from nav_msgs.msg import Path
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist

# KOMPASS
from kompass_core.models import Robot, RobotCtrlLimits, RobotState
from kompass_core.datatypes.laserscan import LaserScanData
from kompass_core.datatypes.pointcloud import PointCloudData
from kompass_core.utils.geometry import from_euler_to_quaternion
from kompass_core.control import ControlClasses, LocalPlannersID, ControllerType

# KOMPASS MSGS/SRVS/ACTIONS
import kompass_interfaces.msg as kompass_msgs
from kompass_interfaces.action import ControlPath

from ..config import BaseValidators, ComponentConfig, ComponentRunType

# KOMPASS ROS
from ..topic import (
    AllowedTopic,
    RestrictedTopicsConfig,
    Topic,
    create_topics_config,
    update_topics_config,
)
from ..utils import component_action
from ..callbacks import PointCloudCallback

# KOMPASS MSGS/SRVS/ACTIONS
from .component import Component, TFListener
from .utils import init_twist_array_msg, send_control_feedback


class ControllerInputs(RestrictedTopicsConfig):
    # Restricted Topics Config for Controller component authorized input topics

    PLAN: AllowedTopic = AllowedTopic(key="plan", types=["Path"])
    SENSOR_DATA: AllowedTopic = AllowedTopic(
        key="sensor_data", types=["LaserScan", "PointCloud2"]
    )
    LOCAL_MAP: AllowedTopic = AllowedTopic(key="local_map", types=["OccupancyGrid"])
    LOCATION: AllowedTopic = AllowedTopic(key="location", types=["Odometry"])


class ControllerOutputs(RestrictedTopicsConfig):
    # Restricted Topics Config for Controller component authorized output topics

    CMD = AllowedTopic(key="command", types=["Twist"])
    MULTI_CMD = AllowedTopic(key="multi_command", types=["TwistArray"])
    INTERPOLATION = AllowedTopic(key="interpolation", types=["Path"])
    LOCAL_PLAN = AllowedTopic(key="local_plan", types=["Path"])
    TRACKING = AllowedTopic(
        key="tracked_point", types=["Odometry", "PoseStamped", "Pose"]
    )


# Create default inputs - Used if no inputs config is provided to the controller
_controller_default_inputs = create_topics_config(
    "ControllerInputs",
    plan=Topic(name="/plan", msg_type="Path"),
    sensor_data=Topic(name="/scan", msg_type="LaserScan"),
    local_map=Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid"),
    location=Topic(name="/odom", msg_type="Odometry"),
)

# Create default outputs - Used if no outputs config is provided to the controller
_controller_default_outputs = create_topics_config(
    "ControllerOutputs",
    command=Topic(name="/control", msg_type="Twist"),
    multi_command=Topic(name="/control_list", msg_type="TwistArray"),
    interpolation=Topic(name="/interpolated_path", msg_type="Path"),
    tracked_point=Topic(name="/tracked_point", msg_type="PoseStamped"),
    local_plan=Topic(name="/local_path", msg_type="Path"),
)


def _set_algorithm(
    value: Union[str, LocalPlannersID],
) -> LocalPlannersID:
    """
    Setter of algorithm with str value or enum value from FollowersID

    :param value: Algorithm value
    :type value: Optional[LocalPlannersID]

    :raises ValueError: If given str value is not one of the values in LocalPlannersID or FollowersID
    """
    if isinstance(value, str):
        if val := LocalPlannersID.get_enum(value):
            return val

        else:
            raise ValueError(
                f"Controller algorithm cannot be set to '{value}'. Valid algorithm values are: '{LocalPlannersID.values()}'"
            )
    return value


@define
class ControllerConfig(ComponentConfig):
    """
    Controller parameters
    """

    use_cmd_list: bool = field(
        default=False
    )  # To send a list of future commands to the drive manager (if True) or a single commands (if False)
    prediction_horizon: int = field(
        default=10, validator=BaseValidators.in_range(min_value=1, max_value=1e9)
    )  # Number of future control commands to compute at each loop step
    control_time_step: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Time step between two control commands

    algorithm: Union[LocalPlannersID, str] = field(
        default=LocalPlannersID.DWA, converter=lambda value: _set_algorithm(value)
    )
    closed_loop: bool = field(default=False)
    cmd_tolerance: float = field(default=0.1)
    use_direct_sensor: bool = field(default=False)
    publish_ctrl_in_parallel: bool = field(init=False)

    def __attrs_post_init__(self):
        """Class post init
        Sets publish in parallel to true for DWA controller
        """
        self.publish_ctrl_in_parallel = self.algorithm == LocalPlannersID.DWA


class Controller(Component):
    """
    Controller component used for path tracking and control around dynamic obstacles during navigation.

    ## Input Topics:
    - *plan*: Global path from current to goal location.<br />  Default ```Topic(name="/plan", msg_type="Path")```
    - *sensor_data*: Sensory information for direct sensor-based control.<br />  Default ```Topic(name="/scan", msg_type="LaserScan")```
    - *location*: the robot current location.<br /> Default ```Topic(name="/odom", msg_type="Odometry")```

    ## Output Topics:
    - *command*: Control command.<br />  Default ```Topic(name="/control", msg_type="Twist")```
    - *multi_command*: Set of future control commands.<br />  Default ```Topic(name="/control_list", msg_type="TwistArray")```
    - *interpolation*: Interpolated path.<br />  Default ```Topic(name="/interpolated_path", msg_type="Path")```
    - *tracked_point*: Tracked path point.<br />  Default ```Topic(name="/tracked_point", msg_type="PoseStamped")```

    ## Available Run Types:
    Set from ControllerConfig class or directly from Controller 'run_type' property.

    - *TIMED*: Compute a new control command periodically if all inputs are available.
    - *ACTIONSERVER*: Offers a ControlPath ROS action and continuously computes a new control once an action request is received until goal point is reached

    ## Usage Example:
    ```
        # Setup custom configuration
        my_config = ControllerConfig(loop_rate=10.0, control_horizon_number_of_steps=7)

        # Init a controller object
        my_controller = Controller(node_name="controller", config=my_config)

        # Change an input
        my_controller.inputs(plan=Topic(name='/global_path', msg_type='Path'))

        # Change run type (default "Timed")
        my_controller.run_type = "ActionServer"

        # Change algorithm
        my_controller.algorithm = LocalPlannersID.DWA_PLANNER
    ```
    """

    def __init__(
        self,
        *,
        node_name: str,
        config_file: Optional[str] = None,
        config: Optional[ControllerConfig] = None,
        inputs=None,
        outputs=None,
        **kwargs,
    ) -> None:
        self.config: ControllerConfig = config or ControllerConfig()

        # Get default component inputs/outputs
        in_topics = _controller_default_inputs()
        out_topics = _controller_default_outputs()

        if inputs:
            in_topics = update_topics_config(in_topics, **inputs)

        if outputs:
            out_topics = update_topics_config(out_topics, **outputs)

        super().__init__(
            config=self.config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=ControllerInputs,
            allowed_outputs=ControllerOutputs,
            node_name=node_name,
            **kwargs,
        )

        self._block_types()

    def create_all_timers(self):
        """Overrides create_all_timers from BaseComponent to add timers for commands execution and tracking publishing"""
        super().create_all_timers()

        # Create timer for publishing commands if parallel publishing is enabled
        if self.config.publish_ctrl_in_parallel:
            self.get_logger().info(
                f"Creating execution timer with step: {self.config.control_time_step}"
            )
            self.__cmd_execution_timer = self.create_timer(
                self.config.control_time_step,
                self._execution_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        else:
            self.get_logger().info(
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
        if self.config.publish_ctrl_in_parallel:
            self.destroy_timer(self.__cmd_execution_timer)
        else:
            self.destroy_rate(self.__cmd_execution_rate)

        self.destroy_timer(self.__info_publishing_timer)

    def _execution_callback(self):
        """Commands execution timer callback"""
        try:
            cmd = self._cmds_queue.get_nowait()
        except Empty:
            self.get_logger().info("No command to execute")
            return
        # If end is reached do not publish new command
        if self.__reached_end:
            self.get_logger().info("No command to execute")
            return

        self.get_logger().info(f"Publishing new command: {cmd[0]}, {cmd[1]}, {cmd[2]}")
        # Send command to execute
        if self.config.closed_loop:
            # Execute in closed loop
            self.execute_cmd(cmd[0], cmd[1], cmd[2])
        else:
            # Publish once
            self._publish_control(
                linear_ctr_x=cmd[0],
                linear_ctr_y=cmd[1],
                angular_ctr=cmd[2],
            )
            self.get_logger().info("Done Publishing")

    def _publishing_callback(self):
        """Tracking publishing timer callback"""
        # Publish tracked point on the global path
        if self.tracked_point:
            self.publishers_dict[ControllerOutputs.TRACKING.key].publish(
                self.tracked_point
            )
        # Publish local plan
        if self.local_plan:
            self.publishers_dict[ControllerOutputs.LOCAL_PLAN.key].publish(
                self.local_plan
            )

        # Publish interpolated path
        if self.interpolated_path:
            self.publishers_dict[ControllerOutputs.INTERPOLATION.key].publish(
                self.interpolated_path
            )

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
        if value in [
            ComponentRunType.SERVER,
            ComponentRunType.EVENT,
            ComponentRunType.SERVER.value,
            ComponentRunType.EVENT.value,
        ]:
            raise ValueError(
                "Controller component can only run as TIMED or ACTION_SERVER"
            )

        self.config.run_type = value

    @property
    def tracked_point(self) -> Optional[PoseStamped]:
        """
        Getter of tracked pose on the reference path if a path is set to the controller

        :return: _description_
        :rtype: StrEnum
        """
        tracked_state: Optional[RobotState] = self.__controller.tracked_state

        if not tracked_state:
            return None

        msg_header = Header()
        msg_header.frame_id = self.config.frames.world
        msg_header.stamp = self.get_ros_time()

        pose_stamped = PoseStamped()
        pose_stamped.header = msg_header
        pose_stamped.pose.position.x = tracked_state.x
        pose_stamped.pose.position.y = tracked_state.y

        q_rot = from_euler_to_quaternion(yaw=tracked_state.yaw, pitch=0.0, roll=0.0)
        pose_stamped.pose.orientation.z = q_rot[3]
        pose_stamped.pose.orientation.w = q_rot[0]
        return pose_stamped

    @property
    def local_plan(self) -> Optional[Path]:
        """
        Getter of controller local plan

        :return: _description_
        :rtype: StrEnum
        """
        # NOTE: Only DWA provides a local plan
        if self.algorithm != LocalPlannersID.DWA:
            return None

        msg_header = Header()
        msg_header.frame_id = self.config.frames.world
        msg_header.stamp = self.get_ros_time()

        local_path = self.__controller.optimal_path(msg_header)
        return local_path

    @property
    def interpolated_path(self) -> Optional[Path]:
        """Getter of interpolated global path

        :return: Path Interpolation
        :rtype: Optional[Path]
        """
        msg_header = Header()
        msg_header.frame_id = self.config.frames.world
        msg_header.stamp = self.get_ros_time()

        interpolated_path = self.__controller.interpolated_path(msg_header)
        return interpolated_path

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
        if self.algorithm != LocalPlannersID.DWA and not value:
            get_logger(self.node_name).warning(
                f"Cannot use Local Map with {self.algorithm} - Setting 'direct_sensor' to True"
            )
            self.config.use_direct_sensor = True
            return
        self.config.use_direct_sensor = value

    @property
    def algorithm(self) -> LocalPlannersID:
        """
        Getter of controller algorithm

        :return: _description_
        :rtype: StrEnum
        """
        return self.config.algorithm

    @algorithm.setter
    def algorithm(self, value: Union[str, LocalPlannersID]) -> None:
        """
        Setter of algorithm with int value or enum value from LocalPlannersID/FollowersID

        :param value: algorithm value
        :type value: Union[str, FollowersID, LocalPlannersID]

        :raises ValueError: If given int value is not one of the values in LocalPlannersID or FollowersID
        """
        self.config.algorithm = value
        self.config.publish_ctrl_in_parallel = value == LocalPlannersID.DWA

    @component_action
    def set_algorithm(
        self,
        algorithm_value: Union[str, LocalPlannersID],
        keep_alive: bool = True,
    ) -> bool:
        """
        Component action - Set controller algorithm action

        :param algorithm_value: algorithm value
        :type algorithm_value: Union[str, FollowersID, LocalPlannersID]
        :param keep_alive: Keep the controller running while changing, defaults to True
        :type keep_alive: bool, optional

        :raises Exception: Exception while updating algorithm value

        :return: Success
        :rtype: bool
        """
        try:
            if keep_alive:
                self.algorithm = algorithm_value
            else:
                self.stop()
                self.algorithm = algorithm_value
                self.start()
        except Exception:
            raise
        return True

    def _block_types(self) -> None:
        """
        Main service and action types of the controller component
        """
        self.action_type = ControlPath

    def _update_state(self) -> None:
        """
        Updates node inputs from associated callbacks
        """
        self.plan: Optional[Path] = self.callbacks[
            ControllerInputs.PLAN.key
        ].get_output()

        self.robot_state: Optional[RobotState] = self.callbacks[
            ControllerInputs.LOCATION.key
        ].get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

        if self.direct_sensor:
            self.sensor_data: Optional[LaserScanData] = self.callbacks[
                ControllerInputs.SENSOR_DATA.key
            ].get_output(
                transformation=self.__sensor_tf_listener.transform
                if self.__sensor_tf_listener
                else None
            )
        else:
            self.local_map: Optional[np.ndarray] = self.callbacks[
                ControllerInputs.LOCAL_MAP.key
            ].get_output()

    def attach_callbacks(self) -> None:
        """
        Attaches method to set received plan to the controller
        """
        # Adds callback to set the path in the controller when a new plan is received
        self.callbacks[ControllerInputs.PLAN.key].on_callback_execute(
            self._set_path_to_controller
        )

        if self.direct_sensor:
            # Remove callback for the local map and destroy subscriber
            _callback = self.callbacks.pop(ControllerInputs.LOCAL_MAP.key)
            self.destroy_subscription(_callback._subscriber)

            # If direct sensor information is used set maximum range for PointCloud data
            sensor_callback = self.callbacks[ControllerInputs.SENSOR_DATA.key]
            if isinstance(sensor_callback, PointCloudCallback):
                sensor_callback.max_range = (
                    self.config.control_horizon_number_of_steps
                    * self.robot.ctrl_vx_limits.max_vel
                )
                self.get_logger().info(
                    f"Setting PointCloud max range to robot max forward horizon '{sensor_callback.max_range}' to limit computations"
                )
        else:
            # Remove callback for the sensor data and destroy subscriber
            _callback = self.callbacks.pop(key=ControllerInputs.SENSOR_DATA.key)
            self.destroy_subscription(_callback._subscriber)

    def _set_path_to_controller(self, msg, **_) -> None:
        """
        Set a new plan to the controller/follower
        """
        self.__controller.set_path(global_path=msg)

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.robot_state: Optional[RobotState] = (
            None  # robot current state - to be updated from odom
        )
        self.plan: Optional[Path] = None  # robot plan (global path)

        # INIT PATH CONTROLLER
        self.__robot = Robot(
            robot_type=self.config.robot.model_type,
            geometry_type=self.config.robot.geometry_type,
            geometry_params=self.config.robot.geometry_params,
            state=self.robot_state,
        )

        # SET robot control limits
        self.__robot_ctr_limits = RobotCtrlLimits(
            vx_limits=self.config.robot.ctrl_vx_limits,
            vy_limits=self.config.robot.ctrl_vy_limits,
            omega_limits=self.config.robot.ctrl_omega_limits,
        )

        self.__controller: ControllerType = ControlClasses[self.algorithm](
            robot=self.__robot,
            ctrl_limits=self.__robot_ctr_limits,
            config_file=self._config_file,
            config_yaml_root_name=f"{self.node_name}.{self.config.algorithm}",
            control_time_step=self.config.control_time_step,
            prediction_horizon=self.config.prediction_horizon,
        )

        # Initialize the velocity command
        self.__cmd_vel = Twist()
        self.__cmd_vel_array = kompass_msgs.TwistArray()

        self.__reached_end = False
        self.__lat_dist_error: float = 0.0
        self.__ori_error: float = 0.0

        self.__execution_rate = self.create_rate(self.config.loop_rate * 10)

        # Command queue to send controller command list to the robot
        self._cmds_queue: Queue = Queue()

        if self.direct_sensor:
            # Setup transform listener
            self.__sensor_tf_listener: TFListener = (
                self.depth_tf_listener
                if self.in_topics.sensor_data.msg_type._ros_type == PointCloud2
                else self.scan_tf_listener
            )

            self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = None

    def init_flags(self):
        """
        Setup node flags to track operations flow
        """
        # reached end path / end of control action
        self.__reached_end: bool = False

    def _stop_robot(self):
        """
        Publishes a zero velocity command to stop the robot
        """
        # send zero command to stop the robot
        if self.config.use_cmd_list:
            self.__cmd_vel_array = init_twist_array_msg(
                self.config.control_horizon_number_of_steps
            )
            self.publishers_dict[ControllerOutputs.MULTI_CMD.key].publish(
                self.__cmd_vel_array
            )
        else:
            self.__cmd_vel = Twist()  # zero command
            self.publishers_dict[ControllerOutputs.CMD.key].publish(self.__cmd_vel)

    def _publish_control(
        self, linear_ctr_x: float, linear_ctr_y: float, angular_ctr: float
    ):
        """
        Publish the computed command to ROS

        :param linear_ctr: Linear control command (m/s)
        :type linear_ctr: float
        :param angular_ctr: Angular control command (rad/s)
        :type angular_ctr: float
        """
        self.__cmd_vel.linear.x = linear_ctr_x
        self.__cmd_vel.linear.y = linear_ctr_y
        self.__cmd_vel.angular.z = angular_ctr

        self.publishers_dict[ControllerOutputs.CMD.key].publish(self.__cmd_vel)

    def execute_cmd(self, vx: float, vy: float, omega: float) -> bool:
        """Execute a control command in closed loop

        :param vx: _description_
        :type vx: float
        :param vy: _description_
        :type vy: float
        :param omega: _description_
        :type omega: float
        :return: _description_
        :rtype: bool
        """
        executing_closed_loop = True
        count = 0
        while executing_closed_loop and count < 10:
            vx_out = (
                vx
                if abs(self.robot_state.vx - vx) > self.config.cmd_tolerance
                or abs(vx) < self.config.cmd_tolerance
                else 0.0
            )
            vy_out = (
                vy
                if abs(self.robot_state.vy - vy) > self.config.cmd_tolerance
                or abs(vy) < self.config.cmd_tolerance
                else 0.0
            )
            omega_out = (
                omega
                if abs(self.robot_state.omega - omega) > self.config.cmd_tolerance
                or abs(omega) < self.config.cmd_tolerance
                else 0.0
            )
            executing_closed_loop = vx_out or vy_out or omega_out
            self.get_logger().info(
                f"Publishing {count}: ({vx_out}, {vy_out}, {omega_out})"
            )
            count += 1
            self._publish_control(
                linear_ctr_x=vx_out,
                angular_ctr=omega_out,
                linear_ctr_y=vy_out,
            )
            self.__execution_rate.sleep()
            self._update_state()

    def _control(self) -> bool:
        """
        Gets robot control command using reference commands

        :return: If robot reached end of reference plan
        :rtype: bool
        """
        if not self.__controller.path:
            self.get_logger().info("Global plan is not available to controller")
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

        cmd_found: bool = self.__controller.loop_step(
            current_state=self.robot_state,  # type: ignore
            laser_scan=laser_scan,
            point_cloud=point_cloud,
            local_map=local_map,
        )

        # LOG CONTROLLER INFO
        self.get_logger().info(f"{cmd_found}: {self.__controller.logging_info()}")

        # PUBLISH CONTROL TO ROBOT CMD TOPIC
        if not cmd_found:
            self.get_logger().warn("Control command not found")
            self.health_status.set_fail_algorithm(
                algorithm_names=[str(ControlClasses[self.algorithm])]
            )
            self.__reached_end = False
            return False

        self.health_status.set_healthy()

        # Empty the commands queue
        self._cmds_queue.empty()

        # Put new control commands to the queue
        [
            self._cmds_queue.put(i)
            for i in zip(
                self.__controller.linear_x_control,
                self.__controller.linear_y_control,
                self.__controller.angular_control,
            )
        ]

        # Update controller path tracking info (errors/end_reached)
        self.__lat_dist_error = self.__controller.distance_error
        self.__ori_error = self.__controller.orientation_error
        self.__reached_end = self.__controller.reached_end()

        # If end is reached stop the robot
        if self.__reached_end:
            self.get_logger().info("Stopping robot!")
            # Stop robot
            self._stop_robot()
            # Clear path
            self.callbacks[ControllerInputs.PLAN.key].clear_last_msg()
            # Unset reached_end for new incoming paths
            self.__reached_end = False
            return True

        # If commands are to be published in sequence
        if not self.config.publish_ctrl_in_parallel:
            # While queue is not empty
            while self._cmds_queue.qsize() > 0:
                self.get_logger().info(f"Queue size: {self._cmds_queue.qsize()}")
                self._execution_callback()
                self.__cmd_execution_rate.sleep()

        return True

    def main_action_callback(self, goal_handle) -> ControlPath.Result:
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

            self.__controller = ControlClasses[self.algorithm](
                robot=self.__robot,
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
        if not self.callbacks_inputs_check():
            self.get_logger().error(
                "Requested action inputs are not available -> Aborting"
            )
            goal_handle.abort()
            return result

        self.__controller.set_path(self.plan)  # type: ignore

        self.__reached_end: bool = False

        action_rate = self.create_rate(self.config.loop_rate)

        while not self.__reached_end:
            self._update_state()

            cmd_found: bool = self._control()

            if not cmd_found:
                break

            # Send controller feedback
            # TODO: update the computation time using timeit, for controller performance tracking
            compute_time = 0.0
            feedback_msg = send_control_feedback(
                feedback_msg,
                self.__cmd_vel.linear.x,
                self.__cmd_vel.angular.z,
                self.config.control_horizon_number_of_steps,
                compute_time,
                self.__lat_dist_error,
                self.__ori_error,
            )
            self.get_logger().info(
                "Controlling Path: {0}".format(feedback_msg.control_feedback)
            )

            goal_handle.publish_feedback(feedback_msg)

            action_rate.sleep()

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

    def _execution_step(self):
        """
        Controller main execution step
        At each time step the controller:
        1- Updates all inputs
        2- Computes the commands using the specified algorithm
        3- Publish commands
        Robot is stopped when the end of the path is reached
        """
        super()._execution_step()

        # Get inputs from callbacks
        self._update_state()

        got_all: bool = self.got_all_inputs()

        # Check if all inputs are available
        if got_all:
            self._control()
