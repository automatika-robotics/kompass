from typing import Optional, Dict
import numpy as np
import time
from attrs import define, field
from geometry_msgs.msg import Twist
from kompass_core.datatypes import LaserScanData
from kompass_core.models import RobotGeometry, RobotState, RobotType
from scipy import signal
from kompass_interfaces.msg import TwistArray

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..topic import Topic, update_topics
from .component import Component
from .utils import twist_array_to_ros_twist
from ..callbacks import LaserScanCallback
from .defaults import (
    TopicsKeys,
    driver_allowed_inputs,
    driver_allowed_outputs,
    driver_default_inputs,
    driver_default_outputs,
)


@define(kw_only=True)
class DriveManagerConfig(ComponentConfig):
    """
    DriveManager component configuration parameters

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **closed_loop**
      - `bool`, `True`
      - Publish commands in closed loop by checking the robot velocity from the odometry topic

    * - **cmd_rate**
      - `float`, `10.0`
      - Rate for sending the commands to the robot in closed loop (Hz)

    * - **closed_loop_span**
      - `int`, `3`
      - Max number of commands to send in a closed loop execution

    * - **smooth_commands**
      - `bool`, `False`
      - Filter (smooth) incoming velocity commands to limit the acceleration

    * - **cmd_tolerance**
      - `float`, `0.1`
      - Tolerance value when checking for reaching the command in closed loop

    * - **critical_zone_angle**
      - `float`, `0.1`
      - Angle range for the emergency stop critical zone (deg)

    * - **critical_zone_distance**
      - `float`, `0.05`
      - Distance for the emergency stop critical zone (meters)

    ```
    """

    closed_loop: bool = field(default=True)

    closed_loop_span: int = field(
        default=3, validator=BaseValidators.in_range(min_value=1, max_value=10)
    )

    cmd_rate: float = field(
        default=10.0, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Rate for sending the commands to the robot (Hz)

    smooth_commands: bool = field(default=False)

    cmd_tolerance: float = field(
        default=0.1
    )  # tolerance value when checking for reaching the command in closed loop

    critical_zone_angle: float = field(
        default=45.0, validator=BaseValidators.in_range(min_value=1e-9, max_value=360.0)
    )  # Angle range for the critical zone (deg)
    critical_zone_distance: float = field(
        default=0.05, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Distance for the critical zone (meters)
    disable_safety_stop: bool = field(default=False)


class DriveManager(Component):
    """DriveManager."""

    def __init__(
        self,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[DriveManagerConfig] = None,
        inputs: Optional[Dict[str, Topic]] = None,
        outputs: Optional[Dict[str, Topic]] = None,
        **kwargs,
    ) -> None:
        """__init__.

        :param component_name:
        :type component_name: str
        :param config_file:
        :type config_file: Optional[str]
        :param config:
        :type config: Optional[DriveManagerConfig]
        :param inputs:
        :param outputs:
        :param kwargs:
        :rtype: None
        """

        if not config:
            config = DriveManagerConfig()

        # Update defaults from custom topics if provided
        in_topics = (
            update_topics(driver_default_inputs, **inputs)
            if inputs
            else driver_default_inputs
        )
        out_topics = (
            update_topics(driver_default_outputs, **outputs)
            if outputs
            else driver_default_outputs
        )

        super().__init__(
            config=config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=driver_allowed_inputs,
            allowed_outputs=driver_allowed_outputs,
            component_name=component_name,
            allowed_run_types=[ComponentRunType.TIMED, ComponentRunType.EVENT],
            **kwargs,
        )
        self.config: DriveManagerConfig = config

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.emergency_stop: bool = False
        self._unblocking_on: bool = False

        # robot output command
        self.command: Optional[Twist] = None
        self.multi_command: Optional[TwistArray] = None
        self._previous_command: Optional[Twist] = None

        self.robot_radius = RobotGeometry.get_radius(
            self.config.robot.geometry_type, self.config.robot.geometry_params
        )

        if not self.robot_radius:
            raise ValueError(
                "Unknown robot radius. Cannot start drive manager with unknown robot size."
            )

        # define the critical zone for the emergency stop
        self.critical_zone = {
            "left_angle": np.radians(self.config.critical_zone_angle) / 2,
            "right_angle": (2 * np.pi)
            - (np.radians(self.config.critical_zone_angle) / 2),
            "distance": self.config.critical_zone_distance + self.robot_radius,
        }

        self.laser_scan: Optional[LaserScanData] = None

        self.emergency_stop_dict = {}

        self._attach_callbacks_and_processors()

    def _attach_callbacks_and_processors(self):
        """
        Attaches emergency_stop_check to sensor_data callback
        anf filtering commands to commands callbacks
        """
        # Attach emergency check to all sensor data callbacks
        num_sensors = self._inputs_keys.count(TopicsKeys.SPATIAL_SENSOR)
        for idx in range(num_sensors):
            callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx)
            if isinstance(callback, LaserScanCallback):
                callback.on_callback_execute(self._check_emergency_stop_lidar)
            else:
                callback.on_callback_execute(
                    self._check_emergency_stop_proximity_sensor
                )

        if self.config.smooth_commands:
            # Attach filtering to commands callback
            self.attach_custom_callback(
                self.get_in_topic(TopicsKeys.INTERMEDIATE_CMD), self._filter_commands
            )
            self.attach_custom_callback(
                self.get_in_topic(TopicsKeys.INTERMEDIATE_CMD_LIST),
                self._filter_multi_commands,
            )

        # Limit commands before publishing
        self.get_publisher(TopicsKeys.FINAL_COMMAND).add_pre_processors([
            self._limit_command_vel
        ])

    def __update_robot_state(self):
        self.robot_state: RobotState = self.get_callback(
            TopicsKeys.ROBOT_LOCATION
        ).get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

    def _update_state(self):
        """
        Update all inputs
        """
        if hasattr(self, "command"):
            self._previous_command = self.command

        self.command: Optional[Twist] = self.get_callback(
            TopicsKeys.INTERMEDIATE_CMD
        ).get_output(clear_last=True)

        self.multi_command: Optional[TwistArray] = self.get_callback(
            TopicsKeys.INTERMEDIATE_CMD_LIST
        ).get_output(clear_last=True)

        self.__update_robot_state()

        num_sensors = self._inputs_keys.count(TopicsKeys.SPATIAL_SENSOR)
        for idx in range(num_sensors):
            callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx)
            if isinstance(callback, LaserScanCallback):
                callback.transformation = (
                    self.scan_tf_listener.transform if self.scan_tf_listener else None
                )
                self.laser_scan: Optional[LaserScanData] = callback.get_output()

    def execute_cmd_open_loop(self, cmd: Twist, max_time: float):
        """Execute a control command in open loop

        :param cmd: Velocity Twist message
        :type cmd: Twist
        :param max_time: Maximum time for the open loop execution (s)
        :type max_time: float
        """
        _step = 1 / self.config.cmd_rate
        _timer_count = 0.0
        while _timer_count < max_time:
            _timer_count += _step
            self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(cmd)
            time.sleep(_step)

    def execute_cmd_closed_loop(self, cmd: Twist, max_time: float):
        """Execute a control command in closed loop

        :param cmd: Velocity Twist message
        :type cmd: Twist
        :param max_time: Maximum time for the closed loop execution (s)
        :type max_time: float
        """
        executing_closed_loop = True
        _step = 1 / self.config.cmd_rate
        _timer_count = 0.0
        while executing_closed_loop and _timer_count < max_time:
            vx_out = (
                cmd.linear.x
                if abs(self.robot_state.vx - cmd.linear.x) > self.config.cmd_tolerance
                or abs(cmd.linear.x) < self.config.cmd_tolerance
                else 0.0
            )
            vy_out = (
                cmd.linear.y
                if abs(self.robot_state.vy - cmd.linear.y) > self.config.cmd_tolerance
                or abs(cmd.linear.y) < self.config.cmd_tolerance
                else 0.0
            )
            omega_out = (
                cmd.angular.z
                if abs(self.robot_state.omega - cmd.angular.z)
                > self.config.cmd_tolerance
                or abs(cmd.angular.z) < self.config.cmd_tolerance
                else 0.0
            )
            executing_closed_loop = vx_out or vy_out or omega_out

            _timer_count += _step
            _cmd = self.__make_twist(vx_out, vy_out, omega_out)

            self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(_cmd)
            time.sleep(_step)
            self.__update_robot_state()

    def move_forward(self, max_distance: float) -> bool:
        """Moves the robot forward if the forward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """

        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.cmd_rate)
        cmd_rate = self.create_rate(self.config.cmd_rate)
        traveled_distance = 0.0

        ranges_in_front = self.laser_scan.get_ranges(
            right_angle=self.critical_zone["right_angle"],
            left_angle=self.critical_zone["left_angle"],
        )

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance forward is clear
            if np.min(ranges_in_front) < (max_distance + self.robot_radius):
                unblocking = False
            else:
                _cmd = self.__make_twist(
                    vx=self.robot.ctrl_vx_limits.max_vel / 2, vy=0.0, omega=0.0
                )
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(_cmd)
                traveled_distance += step_distance
                cmd_rate.sleep()

        # Return true if unblocking forward is done
        return traveled_distance >= max_distance

    def move_backward(self, max_distance: float) -> bool:
        """Moves the robot backwards if the backward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """
        ranges_in_back = self.laser_scan.get_ranges(
            right_angle=self.critical_zone["right_angle"] + np.pi,
            left_angle=self.critical_zone["left_angle"] + np.pi,
        )

        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.cmd_rate)
        cmd_rate = self.create_rate(self.config.cmd_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance behind the robot is clear
            if np.min(ranges_in_back) < (max_distance + self.robot_radius):
                unblocking = False
            else:
                _cmd = self.__make_twist(
                    vx=-self.robot.ctrl_vx_limits.max_vel / 2, vy=0.0, omega=0.0
                )
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(_cmd)
                traveled_distance += step_distance
                cmd_rate.sleep()

        # Return true if unblocking forward is done
        return traveled_distance >= max_distance

    def rotate_in_place(
        self, max_rotation: float, safety_margin: Optional[float] = None
    ) -> bool:
        """Rotates the robot in place if a safety margin around the robot is clear

        :param safety_margin: Margin clear of obstacles to perform rotation, if None defaults to 5% of the robot_radius
        :type safety_margin: Optional[float], optional

        :return: If the movement action is performed
        :rtype: bool
        """
        if self.robot.model_type == RobotType.ACKERMANN:
            self.get_logger().error(
                "Rotation in place action is called but ACKERMANN type robot cannot rotate in place. Aborting"
            )
            return False

        unblocking = True
        cmd_rate = self.create_rate(self.config.cmd_rate)
        traveled_radius = 0.0

        if not safety_margin:
            # Set by default to 10% of the robot radius
            safety_margin = 0.05 * self.robot_radius

        # FRONT MOVEMENT
        while unblocking and traveled_radius < max_rotation:
            if any(self.laser_scan.ranges < (1 + safety_margin) * self.robot_radius):
                unblocking = False
            else:
                _cmd = self.__make_twist(
                    vx=0.0, vy=0.0, omega=self.robot.ctrl_omega_limits.max_vel / 2
                )
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(_cmd)
                traveled_radius += self.robot.ctrl_omega_limits.max_vel / (
                    2 * self.config.cmd_rate
                )
                cmd_rate.sleep()

        # Return true if unblocking forward is done
        return traveled_radius >= max_rotation

    def move_to_unblock(
        self,
        max_distance_forward: Optional[float] = None,
        max_distance_backwards: Optional[float] = None,
        max_rotation: float = np.pi / 4,
        rotation_safety_margin: Optional[float] = None,
    ) -> bool:
        """Moves the robot forward/backward or rotate in place to get out of blocking spots

        :param max_distance_forward: Maximum distance to move forward (meters), if None defaults to 2 * robot_radius
        :type max_distance_forward: Optional[float], optional
        :param max_distance_backwards: Maximum distance to move backwards (meters), if None defaults to 2 * robot_radius
        :type max_distance_backwards: Optional[float], optional
        :param max_rotation: Maximum rotation angle (radians), defaults to np.pi/4
        :type max_rotation: float, optional
        :param rotation_safety_margin: Safety margin to perform rotation in place (meters), if None defaults to 5% of robot_radius
        :type rotation_safety_margin: Optional[float], optional

        :return: If one of the movement actions is performed
        :rtype: bool
        """
        if not self.laser_scan:
            self.get_logger().error(
                "Scan unavailable - Unblocking functionality requires LaserScan information"
            )
            return False

        if not max_distance_forward:
            max_distance_forward = 2 * self.robot_radius

        if not max_distance_backwards:
            max_distance_backwards = 2 * self.robot_radius

        self._unblocking_on = True
        # Try unblocking forward:
        unblocked = self.move_backward(max_distance_backwards)
        if not unblocked and self.robot.model_type != RobotType.ACKERMANN:
            unblocked = self.rotate_in_place(max_rotation, rotation_safety_margin)
        if not unblocked:
            unblocked = self.move_forward(max_distance_forward)

        if not unblocked:
            self.get_logger().error("Robot unblocking Failed due to nearby obstacles")
        else:
            self.get_logger().info("Robot Unblocking Action Done!")
        self._unblocking_on = False

        return unblocked

    def __make_twist(self, vx: float, vy: float, omega: float) -> Twist:
        """Create a Twist message

        :param vx: Linear X velocity (m/s)
        :type vx: float
        :param vy: Linear Y velocity (m/s)
        :type vy: float
        :param omega: Angular velocity (rad/s)
        :type omega: float

        :return: Twist message
        :rtype: Twist
        """
        _cmd = Twist()
        _cmd.linear.x = vx
        _cmd.linear.y = vy
        _cmd.angular.z = omega
        return _cmd

    def __filter_multi_cmds(self, cmd_list, max_acc: float):
        """__filter_multi_cmds.

        :param cmd_list:
        :param max_acc:
        :type max_acc: float
        """
        # Use a low pass filter based on maximum allowed acceleration if multi commands are available
        cmds = np.array(cmd_list)
        w_lin = max_acc / (self.config.loop_rate / 2)  # Normalize the frequency
        b, a = signal.butter(1, w_lin, "low")
        return signal.filtfilt(b, a, cmds)

    def _filter_multi_commands(self, output: TwistArray, **_) -> TwistArray:
        """
        Filters commands list using a low pass filter based on acceleration limit
        """

        # Use a low pass filter based on maximum allowed acceleration if multi commands are available
        self._filtered_linear_commands_x = self.__filter_multi_cmds(
            output.linear_velocities.x, self.config.robot.ctrl_vx_limits.max_acc
        )

        self._filtered_linear_commands_y = self.__filter_multi_cmds(
            output.linear_velocities.y, self.config.robot.ctrl_vy_limits.max_acc
        )

        self._filtered_angular_commands = self.__filter_multi_cmds(
            output.angular_velocities.z, self.config.robot.ctrl_omega_limits.max_acc
        )

    def _check_bounds(self, target, previous, max_acc, max_decel, freq):
        """
        Checks if acceleration limits are satisfied

        :param target: _description_
        :type target: _type_
        :param previous: _description_
        :type previous: _type_
        :param max_acc: _description_
        :type max_acc: _type_
        :param max_decel: _description_
        :type max_decel: _type_
        :param freq: _description_
        :type freq: _type_
        :return: _description_
        :rtype: _type_
        """
        lower_bound = previous - max_decel * (1 / freq)

        upper_bound = previous + max_acc * (1 / freq)

        if target > upper_bound or target < lower_bound:
            return True
        return False

    def _filter_commands(self, output: Twist, **_) -> Twist:
        """
        Filter incoming commands based on last command and acceleration limits
        """
        # If no previous command is recorded yet
        if not self._previous_command:
            return output

        _cmd = Twist()
        # Check and restrict linear velocity
        if self._check_bounds(
            output.linear.x,
            self._previous_command.linear.x,
            self.config.robot.ctrl_vx_limits.max_acc,
            self.config.robot.ctrl_vx_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.linear.x = self._limit_command_acc(
                output.linear.x,
                self._previous_command.linear.x,
                self.config.robot.ctrl_vx_limits.max_acc,
                self.config.robot.ctrl_vx_limits.max_decel,
                self.config.loop_rate,
            )
        else:
            _cmd.linear.x = output.linear.x

        if self._check_bounds(
            output.linear.y,
            self._previous_command.linear.y,
            self.config.robot.ctrl_vy_limits.max_acc,
            self.config.robot.ctrl_vy_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.linear.y = self._limit_command_acc(
                output.linear.y,
                self._previous_command.linear.y,
                self.config.robot.ctrl_vy_limits.max_acc,
                self.config.robot.ctrl_vy_limits.max_decel,
                self.config.loop_rate,
            )
        else:
            _cmd.linear.x = output.linear.x

        # Check and restrict angular velocity
        if self._check_bounds(
            output.angular.z,
            self._previous_command.angular.z,
            self.config.robot.ctrl_omega_limits.max_acc,
            self.config.robot.ctrl_omega_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.angular.z = self._limit_command_acc(
                output.angular.z,
                self._previous_command.angular.z,
                self.config.robot.ctrl_omega_limits.max_acc,
                self.config.robot.ctrl_omega_limits.max_decel,
                self.config.loop_rate,
            )
        else:
            _cmd.angular.z = output.angular.z

        return _cmd

    def _limit_command_acc(
        self,
        target: float,
        current: float,
        max_acc: float,
        max_decel: float,
        freq: float,
    ) -> float:
        """Restricts command based on acceleration limits

        :param target: Target velocity (m/s)
        :type target: float
        :param current: Current velocity (m/s)
        :type current: float
        :param max_acc: Maximum acceleration (m/s^2)
        :type max_acc: float
        :param max_decel: Maximum deceleration (m/s^2)
        :type max_decel: float
        :param freq: frequency (Hz)
        :type freq: float

        :return: Velocity value (m/s)
        :rtype: float
        """
        # Increment that should ideally be applied
        increment = target - current

        # If previous and target are in different direction -> go to maximum allowed
        if current * target < 0 or increment * target < 0:
            inc_max = max_decel / freq
        else:
            inc_max = max_acc / freq

        return current + inc_max * np.sign(increment)

    def _check_emergency_stop_proximity_sensor(
        self, output: Optional[float], topic: Topic, **_
    ):
        if output:
            self.emergency_stop_dict[topic.name] = (
                output < self.critical_zone["distance"] - self.robot_radius
            )

    def _check_emergency_stop_lidar(
        self, output: Optional[LaserScanData], topic: Topic, **_
    ):
        """
        If emergency stop is required from direct sensor -> sets command to zero
        """
        # Update emergency stop check
        if output:
            forward: bool = True if not self.command else self.command.linear.x >= 0

            if forward:
                # Check in front
                ranges_to_check = output.get_ranges(
                    right_angle=self.critical_zone["right_angle"],
                    left_angle=self.critical_zone["left_angle"],
                )
            else:
                # Moving backwards -> Check behind
                ranges_to_check = output.get_ranges(
                    right_angle=self.critical_zone["right_angle"] + np.pi,
                    left_angle=self.critical_zone["left_angle"] + np.pi,
                )

            emergency_stop = np.any(ranges_to_check <= self.critical_zone["distance"])

            self.emergency_stop_dict[topic.name] = bool(emergency_stop)

    def _limit_command_vel(self, output: Twist) -> Twist:
        """Check and limit the control commands

        :param cmd: Robot control command
        :type cmd: Twist

        :return: False if no control command is available
        :rtype: bool
        """

        if abs(output.linear.x) > self.config.robot.ctrl_vx_limits.max_vel:
            self.get_logger().warn(
                f"Limiting linear velocity by allowed maximum {self.config.robot.ctrl_vx_limits.max_vel}"
            )
            output.linear.x = (
                np.sign(output.linear.x) * self.config.robot.ctrl_vx_limits.max_vel
            )

        if abs(output.linear.y) > self.config.robot.ctrl_vy_limits.max_vel:
            self.get_logger().warn(
                f"Limiting linear velocity by allowed maximum {self.config.robot.ctrl_vy_limits.max_vel}"
            )
            output.linear.y = (
                np.sign(output.linear.y) * self.config.robot.ctrl_vy_limits.max_vel
            )

        if abs(output.angular.z) > self.config.robot.ctrl_omega_limits.max_vel:
            self.get_logger().warn(
                f"Limiting angular velocity by allowed maximum {self.config.robot.ctrl_omega_limits.max_vel}"
            )
            output.angular.z = (
                np.sign(output.angular.z) * self.config.robot.ctrl_omega_limits.max_vel
            )
        return output

    def _execution_step(self):
        """
        Main execution of the component, executed at ech timer tick with rate self.config.loop_rate
        """
        self._update_state()

        self.emergency_stop = any(self.emergency_stop_dict.values())

        self.get_publisher(TopicsKeys.EMERGENCY).publish(bool(self.emergency_stop))

        if self._unblocking_on:
            # If unblocking is ongoing do not publish any other command
            self.command = None
            self.multi_command = None
            return

        if self.emergency_stop:
            # STOP ROBOT
            self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(Twist())
            return

        if self.command or self.multi_command:
            if not self.config.disable_safety_stop and not self.laser_scan:
                self.get_logger().warn(
                    "LaserScan data is not available -> disabling command publish to robot. To use the DriveManager without safety stop set 'disable_safety_stop' to 'True'",
                    once=True,
                )
                return
            if not self.robot_state and self.config.closed_loop:
                self.get_logger().warn(
                    "Robot state is not available and command publish is set to closed loop -> disabling command publish to robot. To use the DriveManager without robot state set 'closed_loop' to 'False'",
                    once=True,
                )
                return

        if self.command:
            if self.config.closed_loop:
                self.execute_cmd_closed_loop(
                    self.command,
                    max_time=self.config.closed_loop_span / self.config.cmd_rate,
                )
            else:
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(self.command)
            # Clear published command
            self.command = None
            return

        if self.multi_command:
            # TODO: Stop publishing old multi command when a new multi command is received
            for idx in range(len(self.multi_command.linear_velocities.x)):
                _cmd = twist_array_to_ros_twist(self.multi_command, idx=idx)
                if self.config.closed_loop:
                    self.execute_cmd_closed_loop(
                        _cmd, max_time=self.multi_command.time_step
                    )
                else:
                    self.execute_cmd_open_loop(
                        _cmd, max_time=self.multi_command.time_step
                    )
            self.multi_command = None
            return

        self.get_logger().debug("Nothing to process, waiting for commands")
