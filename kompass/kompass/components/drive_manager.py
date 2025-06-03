from typing import Optional, Dict
import numpy as np
import time
from queue import Queue, Empty
from attrs import define, field
from functools import partial
from geometry_msgs.msg import Twist
from kompass_core.datatypes import LaserScanData
from kompass_core.models import RobotGeometry, RobotState, RobotType
from scipy import signal
from kompass_interfaces.msg import TwistArray

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from .ros import Topic, update_topics
from .component import Component
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
        default=0.01
    )  # tolerance value when checking for reaching the command in closed loop

    critical_zone_angle: float = field(
        default=45.0, validator=BaseValidators.in_range(min_value=1e-9, max_value=360.0)
    )  # Angle range for the critical zone (deg)
    critical_zone_distance: float = field(
        default=0.05, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Distance for the stop zone (meters)
    slowdown_zone_distance: float = field(
        default=0.2, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Distance for the slowdown zone (meters)
    disable_safety_stop: bool = field(default=False)
    use_gpu: bool = field(default=False)


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
        self._unblocking_on: bool = False

        # robot output command
        self._previous_command: Optional[Twist] = None
        self._multi_command_step = 0.0
        self._last_direction_forward: Optional[bool] = None

        # Command queue to send controller command list to the robot
        self._cmds_queue: Queue = Queue()

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

        self.slow_down_factor: Dict[str, float] = {}

        # Emergency checker gets initialized on activation to get the sensor transformation
        self._emergency_checker = None

        self._attach_callbacks_and_processors()

    def _attach_callbacks_and_processors(self):
        """
        Attaches emergency_stop_check to sensor_data callback
        anf filtering commands to commands callbacks
        """
        if not self.config.disable_safety_stop:
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

        # Add command publishing on intermediate command input
        self.attach_custom_callback(
            self.get_in_topic(TopicsKeys.INTERMEDIATE_CMD),
            partial(
                self._single_cmd_callback,
                closed_loop=self.config.closed_loop,
                smooth_cmds=self.config.smooth_commands,
            ),
        )

        # Add command publishing on multi intermediate command input
        self.attach_custom_callback(
            self.get_in_topic(TopicsKeys.INTERMEDIATE_CMD_LIST),
            partial(
                self._multi_cmds_callback,
                smooth_cmds=self.config.smooth_commands,
            ),
        )

        # Limit commands before publishing
        self.get_publisher(TopicsKeys.FINAL_COMMAND).add_pre_processors([
            self._limit_command_vel
        ])

    def __update_robot_state(self):
        """Update robot state"""
        self.robot_state: RobotState = self.get_callback(
            TopicsKeys.ROBOT_LOCATION
        ).get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

    def _single_cmd_callback(
        self, output: Twist, closed_loop: bool, smooth_cmds: bool, **_
    ):
        """Execute on the callback of incoming single intermediate command

        :param output: Incoming command
        :type output: Twist
        :param closed_loop: Executing the command in closed loop
        :type closed_loop: bool
        :param smooth_cmds: Smooth (filter) the incoming commands
        :type smooth_cmds: bool
        """
        if self._unblocking_on:
            return
        # Check emergency stop
        self._update_state()
        slowdown_val: float = min(self.slow_down_factor.values())
        if slowdown_val == 0.0:
            # STOP ROBOT
            self.get_publisher(TopicsKeys.EMERGENCY).publish(True)
            self.get_logger().warn("EMERGENCY STOP ON")
            return

        # Multiply by slowdown factor
        output.linear.x *= slowdown_val
        output.linear.y *= slowdown_val
        # output.angular.z *= slowdown_val

        filtered_output: Twist = (
            self._filter_commands(output=output) if smooth_cmds else output
        )
        if closed_loop:
            self.execute_cmd_closed_loop(
                filtered_output, self.config.closed_loop_span / self.config.cmd_rate
            )
        else:
            # Publish once in open loop
            self.execute_cmd_open_loop(
                filtered_output,
                max_time=self.config.closed_loop_span / self.config.cmd_rate,
            )
        self._previous_command = filtered_output

    def _multi_cmds_callback(self, output: TwistArray, smooth_cmds: bool, **_):
        """Execute on the callback of incoming single intermediate command

        :param output: Incoming commands
        :type output: TwistArray
        :param smooth_cmds: Smooth (filter) the incoming commands
        :type smooth_cmds: bool
        """
        if self._unblocking_on:
            return

        filtered_output: TwistArray = (
            self._filter_multi_commands(output=output) if smooth_cmds else output
        )

        self._multi_command_step = output.time_step

        # Set filtered commands to queue
        self._cmds_queue.queue.clear()

        # Put new control commands to the queue
        [
            self._cmds_queue.put([vx, vy, omega])
            for (vx, vy, omega) in zip(
                filtered_output.linear_velocities.x,
                filtered_output.linear_velocities.y,
                filtered_output.angular_velocities.z,
            )
        ]

    def _update_state(self):
        """
        Update all inputs
        """
        self.__update_robot_state()

        num_sensors = self._inputs_keys.count(TopicsKeys.SPATIAL_SENSOR)
        for idx in range(num_sensors):
            callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx)
            if isinstance(callback, LaserScanCallback):
                self.laser_scan: Optional[LaserScanData] = callback.get_output()
                break
        # If laserscan is not available and safety_stop is enabled -> raise an emergency stop flog to block publishing
        if not self.config.disable_safety_stop and not self.laser_scan:
            self.get_logger().warn(
                "LaserScan data is not available -> disabling command publish to robot. To use the DriveManager without safety stop set 'disable_safety_stop' to 'True'",
                once=True,
            )
            self.slow_down_factor["unavailable_data"] = 0.0
        else:
            self.slow_down_factor["unavailable_data"] = 1.0

    def _publish_cmd(self, cmd: Twist, slowdown_factor: Optional[float] = None):
        """Publish command to the robot

        :param cmd: Velocity Twist message
        :type cmd: Twist
        """
        # Check emergency stop
        if not slowdown_factor:
            self._update_state()
            slowdown_val: float = min(self.slow_down_factor.values())
        else:
            slowdown_val = slowdown_factor
        cmd.linear.x *= slowdown_val
        cmd.linear.y *= slowdown_val
        # cmd.angular.z *= slowdown_val
        # Publish command
        self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(cmd)

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
            self._publish_cmd(cmd)
            time.sleep(_step)

    def execute_cmd_closed_loop(self, output: Twist, max_time: float):
        """Execute a control command in closed loop

        :param cmd: Velocity Twist message
        :type cmd: Twist
        :param max_time: Maximum time for the closed loop execution (s)
        :type max_time: float
        """
        if not self.robot_state:
            self.get_logger().warn(
                "Robot state is not available and command publish is set to closed loop -> disabling command publish to robot. To use the DriveManager without robot state set 'closed_loop' to 'False'"
            )
            return

        executing_closed_loop = True
        _step = 1 / self.config.cmd_rate
        _timer_count = 0.0
        while executing_closed_loop and _timer_count < max_time:
            vx_out = (
                output.linear.x
                if abs(self.robot_state.vx - output.linear.x)
                > self.config.cmd_tolerance
                or abs(output.linear.x) < self.config.cmd_tolerance
                else 0.0
            )
            vy_out = (
                output.linear.y
                if abs(self.robot_state.vy - output.linear.y)
                > self.config.cmd_tolerance
                or abs(output.linear.y) < self.config.cmd_tolerance
                else 0.0
            )
            omega_out = (
                output.angular.z
                if abs(self.robot_state.omega - output.angular.z)
                > self.config.cmd_tolerance
                or abs(output.angular.z) < self.config.cmd_tolerance
                else 0.0
            )
            executing_closed_loop = vx_out or vy_out or omega_out

            _timer_count += _step
            # Publish command
            _cmd = self.__make_twist(vx_out, vy_out, omega_out)
            self._publish_cmd(_cmd)
            time.sleep(_step)

    def move_forward(self, max_distance: float) -> bool:
        """Moves the robot forward if the forward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """

        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.cmd_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance forward is clear
            slowdown_factor = self._emergency_checker.check(
                angles=self.laser_scan.angles,
                ranges=self.laser_scan.ranges,
                forward=True,
            )
            if slowdown_factor == 0.0:
                unblocking = False
            else:
                _cmd = self.__make_twist(
                    vx=self.robot.ctrl_vx_limits.max_vel / 2, vy=0.0, omega=0.0
                )
                self._publish_cmd(_cmd, slowdown_factor=slowdown_factor)
                traveled_distance += step_distance
                time.sleep(1 / self.config.cmd_rate)

        # Return true if unblocking forward is done
        return traveled_distance >= max_distance

    def move_backward(self, max_distance: float) -> bool:
        """Moves the robot backwards if the backward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """
        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.cmd_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance behind the robot is clear
            slowdown_factor = self._emergency_checker.check(
                angles=self.laser_scan.angles,
                ranges=self.laser_scan.ranges,
                forward=False,
            )
            if slowdown_factor == 0.0:
                unblocking = False
            else:
                _cmd = self.__make_twist(
                    vx=-self.robot.ctrl_vx_limits.max_vel / 4, vy=0.0, omega=0.0
                )
                self._publish_cmd(_cmd, slowdown_factor=slowdown_factor)
                traveled_distance += step_distance
                time.sleep(1 / self.config.cmd_rate)

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
        max_rotation: float = np.pi / 2,
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
        unblocking_actions = [
            (self.move_backward, [max_distance_backwards], "Move Backward"),
            (self.move_forward, [max_distance_forward], "Move Forward"),
        ]
        if self.robot.model_type != RobotType.ACKERMANN:
            unblocking_actions.append((
                self.rotate_in_place,
                [max_rotation, rotation_safety_margin],
                "Rotate In Place",
            ))
        # Shuffle the actions to perform them in random order
        import random

        random.shuffle(unblocking_actions)

        unblocked = False
        for action, args, log_info in unblocking_actions:
            self.get_logger().info(f"Performing unblocking action: {log_info}")
            unblocked = action(*args)
            if unblocked:
                break

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
            self.slow_down_factor[topic.name] = (
                0.0
                if (output < self.critical_zone["distance"] - self.robot_radius)
                else 1.0
            )
        else:
            self.slow_down_factor[topic.name] = 1.0

    def _check_emergency_stop_lidar(
        self, output: Optional[LaserScanData], topic: Topic, **_
    ):
        """
        If emergency stop is required from direct sensor -> sets command to zero
        """
        # Update emergency stop check
        if not output or not self._emergency_checker:
            return
        if not self._previous_command or self._previous_command.linear.x == 0.0:
            forward: bool = (
                self._last_direction_forward
                if self._last_direction_forward is not None
                else True
            )
        else:
            forward = self._previous_command.linear.x >= 0

        self.slow_down_factor[topic.name] = self._emergency_checker.check(
            angles=output.angles, ranges=output.ranges, forward=forward
        )
        self._last_direction_forward = forward

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
        elif abs(output.linear.x) < self.config.robot.ctrl_vx_limits.min_absolute_val:
            output.linear.x = 0.0

        if abs(output.linear.y) > self.config.robot.ctrl_vy_limits.max_vel:
            self.get_logger().warn(
                f"Limiting linear Vy velocity by allowed maximum {self.config.robot.ctrl_vy_limits.max_vel}"
            )
            output.linear.y = (
                np.sign(output.linear.y) * self.config.robot.ctrl_vy_limits.max_vel
            )
        elif abs(output.linear.y) < self.config.robot.ctrl_vy_limits.min_absolute_val:
            output.linear.y = 0.0

        if abs(output.angular.z) > self.config.robot.ctrl_omega_limits.max_vel:
            self.get_logger().warn(
                f"Limiting angular velocity by allowed maximum {self.config.robot.ctrl_omega_limits.max_vel}"
            )
            output.angular.z = (
                np.sign(output.angular.z) * self.config.robot.ctrl_omega_limits.max_vel
            )
        elif (
            abs(output.angular.z) < self.config.robot.ctrl_omega_limits.min_absolute_val
        ):
            output.angular.z = 0.0
        return output

    def _execution_step(self):
        """
        Main execution of the component, executed at ech timer tick with rate self.config.loop_rate
        """
        if self._unblocking_on:
            return
        # Check emergency stop
        self._update_state()
        speed_factor = min(self.slow_down_factor.values())
        if speed_factor < 0.1:
            # STOP ROBOT
            self.get_publisher(TopicsKeys.EMERGENCY).publish(True)
            return
        else:
            self.get_publisher(TopicsKeys.EMERGENCY).publish(False)

        if speed_factor == 0.0:
            self.get_logger().warn(
                "Emergency stop is ON, no commands will be executed"
            )
            return

        # Publish commands in the queue
        try:
            cmd = self._cmds_queue.get_nowait()
        except Empty:
            self.get_logger().debug("No commands to execute")
            return

        # create a publish one twist message
        _cmd_vel = self.__make_twist(cmd[0], cmd[1], cmd[2])

        if self.config.closed_loop:
            self.execute_cmd_closed_loop(_cmd_vel, max_time=self._multi_command_step)
        else:
            self.execute_cmd_open_loop(_cmd_vel, max_time=self._multi_command_step)

    def _execute_once(self):
        super()._execute_once()
        # Get transformation from sensor to robot body
        while not self.scan_tf_listener or not self.scan_tf_listener.transform:
            self.get_logger().info("Waiting to get laserscan TF...", once=True)
            time.sleep(1 / self.config.loop_rate)

        robot_shape = RobotGeometry.Type.to_kompass_cpp_lib(
            self.config.robot.geometry_type
        )
        robot_dimensions = self.config.robot.geometry_params

        if self.config.use_gpu:
            try:
                from kompass_cpp.utils import CriticalZoneCheckerGPU

                # Get laserscan data to initialize the GPU based checker
                while not self.laser_scan:
                    self.get_logger().info(
                        "Waiting to get laserscan data to initialize CriticalZoneCheckerGPU...",
                        once=True,
                    )
                    self._update_state()
                    time.sleep(1 / self.config.loop_rate)

                self._emergency_checker = CriticalZoneCheckerGPU(
                    robot_shape=robot_shape,
                    robot_dimensions=robot_dimensions,
                    sensor_position_body=self.scan_tf_listener.translation,
                    sensor_rotation_body=self.scan_tf_listener.rotation,
                    critical_angle=self.config.critical_zone_angle,
                    critical_distance=self.config.critical_zone_distance,
                    slowdown_distance=self.config.slowdown_zone_distance,
                    scan_angles=self.laser_scan.angles,
                )
                self.get_logger().info("CriticalZoneCheckerGPU is READY!")
                return
            except ImportError:
                self.get_logger().warn(
                    "GPU use is enabled but CriticalZoneCheckerGPU implementation is not found -> Using CPU implementation instead"
                )

        from kompass_cpp.utils import CriticalZoneChecker

        self._emergency_checker = CriticalZoneChecker(
            robot_shape=robot_shape,
            robot_dimensions=robot_dimensions,
            sensor_position_body=self.scan_tf_listener.translation,
            sensor_rotation_body=self.scan_tf_listener.rotation,
            critical_angle=self.config.critical_zone_angle,
            critical_distance=self.config.critical_zone_distance,
            slowdown_distance=self.config.slowdown_zone_distance,
        )
        self.get_logger().info("CriticalZoneChecker is READY!")
