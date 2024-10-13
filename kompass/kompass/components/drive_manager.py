from typing import Optional, Union
import numpy as np
from attrs import define, field
from geometry_msgs.msg import Twist
from kompass_core.datatypes.laserscan import LaserScanData
from kompass_core.models import RobotGeometry, RobotState, RobotType
from kompass_core.utils.geometry import convert_to_0_2pi
from scipy import signal
from kompass_interfaces.msg import TwistArray

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..topic import (
    AllowedTopic,
    RestrictedTopicsConfig,
    Topic,
    create_topics_config,
    update_topics_config,
)
from .component import Component
from ..callbacks import LaserScanCallback


class DriverInputs(RestrictedTopicsConfig):
    # Restricted Topics Config for DriveManager component authorized input topics

    CMD = AllowedTopic(key="command", types=["Twist"])
    MULTI_CMD = AllowedTopic(key="multi_command", types=["TwistArray"])
    SENSOR_DATA = AllowedTopic(
        key="sensor_data",
        types=["LaserScan", "Float64", "Float32"],
        number_required=1,
        number_optional=10,
    )
    LOCATION = AllowedTopic(
        key="location", types=["Odometry", "PoseStamped", "PointStamped"]
    )


class DriverOutputs(RestrictedTopicsConfig):
    # Restricted Topics Config for DriveManager component authorized input topics

    CMD = AllowedTopic(key="robot_command", types=["Twist"])
    STOP = AllowedTopic(key="emergency_stop", types=["Bool"])


_driver_default_inputs = create_topics_config(
    "DriverInputs",
    command=Topic(name="/control", msg_type="Twist"),
    multi_command=Topic(name="/control_list", msg_type="TwistArray"),
    sensor_data=Topic(name="/scan", msg_type="LaserScan"),
    location=Topic(name="/odom", msg_type="Odometry"),
    allowed_config=DriverInputs,
)

_driver_default_outputs = create_topics_config(
    "DriverOutputs",
    robot_command=Topic(name="/cmd_vel", msg_type="Twist"),
    emergency_stop=Topic(name="/emergency_stop", msg_type="Bool"),
    allowed_config=DriverOutputs,
)


@define(kw_only=True)
class DriveManagerConfig(ComponentConfig):
    """
    Drive manager node config
    """

    cmd_rate: float = field(
        default=10.0, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Rate for sending the commands to the robot (Hz)

    smooth_commands: bool = field(default=True)

    closed_loop: bool = field(default=True)  # Checks feedback from odometry topic

    critical_zone_angle: float = field(
        default=45.0, validator=BaseValidators.in_range(min_value=1e-9, max_value=360.0)
    )  # Angle range for the critical zone (deg)
    critical_zone_distance: float = field(
        default=0.05, validator=BaseValidators.in_range(min_value=1e-9, max_value=1e9)
    )  # Distance for the critical zone (meters)


class DriveManager(Component):
    """DriveManager."""

    def __init__(
        self,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[DriveManagerConfig] = None,
        inputs=None,
        outputs=None,
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

        # Get default component inputs/outputs
        in_topics = _driver_default_inputs()
        out_topics = _driver_default_outputs()

        if inputs:
            in_topics = update_topics_config(in_topics, **inputs)

        if outputs:
            out_topics = update_topics_config(out_topics, **outputs)

        super().__init__(
            config=config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=DriverInputs,
            allowed_outputs=DriverOutputs,
            component_name=component_name,
            **kwargs,
        )
        self.config: DriveManagerConfig = config

        # TODO: Raise error here, similar to the property
        self.config.run_type = "Timed"

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
        """Overrides property setter to restrict to implemented motion server run types

        :param value: Run type
        :type value: Union[ComponentRunType, str]
        :raises ValueError: If run_type is unsupported
        """
        if value not in [ComponentRunType.TIMED, ComponentRunType.TIMED.value]:
            raise ValueError("DriveManager works only as a Timed component")

        self.config.run_type = value

    def init_flags(self):
        """
        Setup node flags to track operations flow
        """
        self.emergency_stop: bool = False
        self._unblocking_on: bool = False

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """

        # robot output command
        self.cmd = Twist()
        self._previous_command = None

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

    def attach_callbacks(self):
        """
        Attaches emergency_stop_check to sensor_data callback
        anf filtering commands to commands callbacks
        """
        # Attach emergency check to all sensor data callbacks
        for callback in self.callbacks[DriverInputs.SENSOR_DATA.key].values():
            if isinstance(callback, LaserScanCallback):
                callback.on_callback_execute(self._check_emergency_stop_lidar)
            else:
                callback.on_callback_execute(
                    self._check_emergency_stop_proximity_sensor
                )

        # Attach filtering to commands callback
        self.callbacks[DriverInputs.CMD.key].on_callback_execute(self._filter_commands)

        self.callbacks[DriverInputs.MULTI_CMD.key].on_callback_execute(
            self._filter_multi_commands
        )

    def _update_state(self):
        """
        Update all inputs
        """
        if hasattr(self, "command"):
            self._previous_command = self.command

        self.command: Optional[Twist] = self.callbacks[DriverInputs.CMD.key].get_output()

        self.multi_command: Optional[TwistArray] = self.callbacks[
            DriverInputs.MULTI_CMD.key
        ].get_output()

        self.robot_state: RobotState = self.callbacks[
            DriverInputs.LOCATION.key
        ].get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

        for callback in self.callbacks[DriverInputs.SENSOR_DATA.key].values():
            if isinstance(callback, LaserScanCallback):
                callback.transformation = (
                    self.scan_tf_listener.transform if self.scan_tf_listener else None
                )
                self.laser_scan: Optional[LaserScanData] = callback.get_output()

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

        angles: np.ndarray = convert_to_0_2pi(self.laser_scan.angles)

        # Get indices of angles in front of the robot
        angles_in_critical_indices = (angles <= self.critical_zone["left_angle"]) | (
            angles >= self.critical_zone["right_angle"]
        )

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance forward is clear
            if np.min(self.laser_scan.ranges[angles_in_critical_indices]) < (
                max_distance + self.robot_radius
            ):
                unblocking = False
            else:
                self.cmd = Twist()
                self.cmd.linear.x = self.robot.ctrl_vx_limits.max_vel / 2
                self.publishers_dict[DriverOutputs.CMD.key].publish(self.cmd)
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
        # Behind the robot
        angles: np.ndarray = convert_to_0_2pi(self.laser_scan.angles)

        # Get indices of angles behind the robot
        angles_in_critical_indices = (
            angles <= convert_to_0_2pi(self.critical_zone["left_angle"] + np.pi)
        ) & (angles >= convert_to_0_2pi(self.critical_zone["right_angle"] + np.pi))

        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.cmd_rate)
        cmd_rate = self.create_rate(self.config.cmd_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while unblocking and traveled_distance < max_distance:
            # Check if max_distance behind the robot is clear
            if np.min(self.laser_scan.ranges[angles_in_critical_indices]) < (
                max_distance + self.robot_radius
            ):
                unblocking = False
            else:
                self.cmd = Twist()
                self.cmd.linear.x = -self.robot.ctrl_vx_limits.max_vel / 2
                self.publishers_dict[DriverOutputs.CMD.key].publish(self.cmd)
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
            return

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
                self.cmd = Twist()
                self.cmd.angular.z = self.robot.ctrl_omega_limits.max_vel / 2
                self.publishers_dict[DriverOutputs.CMD.key].publish(self.cmd)
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
            return

        if not max_distance_forward:
            max_distance_forward = 2 * self.robot_radius

        if not max_distance_backwards:
            max_distance_backwards = 2 * self.robot_radius

        self._unblocking_on = True
        # Try unblocking forward:
        unblocked = self.move_backward(max_distance_backwards)
        if not unblocked and self.robot.model_type != RobotType.ACKERMANN:
            unblocked = self.move_rotate_in_place(max_rotation, rotation_safety_margin)
        if not unblocked:
            unblocked = self.move_forward(max_distance_forward)

        if not unblocked:
            self.get_logger().error("Robot unblocking Failed due to nearby obstacles")
        else:
            self.get_logger().info("Robot Unblocking Action Done!")
        self._unblocking_on = False

        return unblocked

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

    def _filter_multi_commands(self, msg, **_):
        """
        Filters commands list using a low pass filter based on acceleration limit
        """
        # Use a low pass filter based on maximum allowed acceleration if multi commands are available
        self._filtered_linear_commands_x = self.__filter_multi_cmds(
            msg.linear.x, self.config.robot.ctrl_vx_limits.max_acc
        )

        self._filtered_linear_commands_y = self.__filter_multi_cmds(
            msg.linear.y, self.config.robot.ctrl_vy_limits.max_acc
        )

        self._filtered_angular_commands = self.__filter_multi_cmds(
            msg.angular.z, self.config.robot.ctrl_omega_limits.max_acc
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

    def _filter_commands(self, msg: Twist, **_):
        """
        Filter incoming commands based on last command and acceleration limits
        """
        if self._previous_command:
            # Check and restrict linear velocity
            if self._check_bounds(
                msg.linear.x,
                self._previous_command.linear.x,
                self.config.robot.ctrl_vx_limits.max_acc,
                self.config.robot.ctrl_vx_limits.max_decel,
                self.config.loop_rate,
            ):
                self.cmd.linear.x = self._restrict_command(
                    msg.linear.x,
                    self._previous_command.linear.x,
                    self.config.robot.ctrl_vx_limits.max_acc,
                    self.config.robot.ctrl_vx_limits.max_decel,
                    self.config.loop_rate,
                )
            else:
                self.cmd.linear.x = msg.linear.x

            if self._check_bounds(
                msg.linear.y,
                self._previous_command.linear.y,
                self.config.robot.ctrl_vy_limits.max_acc,
                self.config.robot.ctrl_vy_limits.max_decel,
                self.config.loop_rate,
            ):
                self.cmd.linear.y = self._restrict_command(
                    msg.linear.y,
                    self._previous_command.linear.y,
                    self.config.robot.ctrl_vy_limits.max_acc,
                    self.config.robot.ctrl_vy_limits.max_decel,
                    self.config.loop_rate,
                )
            else:
                self.cmd.linear.x = msg.linear.x

            # Check and restrict angular velocity
            if self._check_bounds(
                msg.angular.z,
                self._previous_command.angular.z,
                self.config.robot.ctrl_omega_limits.max_acc,
                self.config.robot.ctrl_omega_limits.max_decel,
                self.config.loop_rate,
            ):
                self.cmd.angular.z = self._restrict_command(
                    msg.angular.z,
                    self._previous_command.angular.z,
                    self.config.robot.ctrl_omega_limits.max_acc,
                    self.config.robot.ctrl_omega_limits.max_decel,
                    self.config.loop_rate,
                )
            else:
                self.cmd.angular.z = msg.angular.z
        # If no previous command is recorded yet
        else:
            self.cmd = msg

    def _restrict_command(self, target, current, max_acc, max_decel, freq) -> float:
        """
        Restricts command based on acceleration limits

        :param target: _description_
        :type target: _type_
        :param current: _description_
        :type current: _type_
        :param max_acc: _description_
        :type max_acc: _type_
        :param max_decel: _description_
        :type max_decel: _type_
        :param freq: _description_
        :type freq: _type_
        :return: _description_
        :rtype: float
        """
        # Increment that should ideally be applied
        increment = target - current

        # If previous and tagret are in different direction -> go to maximum allowed
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
            angles: np.ndarray = convert_to_0_2pi(output.angles)

            angles_in_critical_indices = (
                angles <= self.critical_zone["left_angle"]
            ) | (angles >= self.critical_zone["right_angle"])

            emergency_stop = np.any(
                output.ranges[angles_in_critical_indices]
                <= self.critical_zone["distance"]
            )

            self.emergency_stop_dict[topic.name] = bool(emergency_stop)

    def check_limits(self):
        """
        Check and limit the control commands

        :param cmd: Robot control command
        :type cmd: Twist

        :return: Limited control command
        :rtype: Twist
        """
        if abs(self.cmd.linear.x) > self.config.robot.ctrl_vx_limits.max_vel:
            self.get_logger().warn(
                f"Limiting linear velocity by allowed maximum {self.config.robot.ctrl_vx_limits.max_vel}"
            )
            self.cmd.linear.x = (
                np.sign(self.cmd.linear.x) * self.config.robot.ctrl_vx_limits.max_vel
            )

        if abs(self.cmd.linear.y) > self.config.robot.ctrl_vy_limits.max_vel:
            self.get_logger().warn(
                f"Limiting linear velocity by allowed maximum {self.config.robot.ctrl_vy_limits.max_vel}"
            )
            self.cmd.linear.y = (
                np.sign(self.cmd.linear.y) * self.config.robot.ctrl_vy_limits.max_vel
            )

        if abs(self.cmd.angular.z) > self.config.robot.ctrl_omega_limits.max_vel:
            self.get_logger().warn(
                f"Limiting angular velocity by allowed maximum {self.config.robot.ctrl_omega_limits.max_vel}"
            )
            self.cmd.angular.z = (
                np.sign(self.cmd.angular.z)
                * self.config.robot.ctrl_omega_limits.max_vel
            )

    def _execution_step(self):
        """
        Main execution of the component, executed at ech timer tick with rate self.config.loop_rate
        """
        super()._execution_step()
        self._update_state()

        self.emergency_stop = any(self.emergency_stop_dict.values())

        self.publishers_dict[DriverOutputs.STOP.key].publish(bool(self.emergency_stop))

        if self._unblocking_on:
            # If unblocking is ongoing do not publish any other command
            return

        if self.emergency_stop:
            # STOP ROBOT
            self.cmd = Twist()
            self.publishers_dict[DriverOutputs.CMD.key].publish(self.cmd)
            return

        # Apply acceleration limit check
        if self.multi_command:
            self.cmd.linear.x = self.multi_command.linear_velocities.x[0]
            self.cmd.angular.z = self.multi_command.angular_velocities.z[0]

        if self.multi_command or self.command:
            self.check_limits()
            self.publishers_dict[DriverOutputs.CMD.key].publish(self.cmd)
        else:
            self.get_logger().debug("Nothing to process, waiting for commands")

        # Clear published command
        self.command = None
        self.multi_command = None
