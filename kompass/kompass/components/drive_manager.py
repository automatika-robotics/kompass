from typing import Optional, Dict, List, Tuple, Union
import numpy as np
import time
from queue import Queue, Empty
from attrs import define, field
from functools import partial
from geometry_msgs.msg import Twist
from kompass_core.models import RobotGeometry, RobotState
from ..robot import RobotType
from kompass_interfaces.msg import TwistArray
from kompass_cpp.types import SensorInputType

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from .ros import Topic, update_topics, component_action
from .component import Component
from ..callbacks import LaserScanCallback, PointCloudCallback
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

    * - **closed_loop_span**
      - `int`, `3`
      - Max number of commands to send in a closed loop execution

    * - **smooth_commands**
      - `bool`, `False`
      - Filter (smooth) incoming velocity commands to limit the acceleration

    * - **cmd_tolerance**
      - `float`, `0.05`
      - Tolerance value when checking for reaching the command in closed loop

    * - **critical_zone_angle**
      - `float`, `45deg`
      - Angle range for the emergency stop critical zone (deg)

    * - **critical_zone_distance**
      - `float`, `0.05`
      - Distance for the emergency stop critical zone (meters)

    * - **slowdown_zone_distance**
      - `float`, `0.2`
      - Distance for the slowdown zone (meters)

    * - **disable_safety_stop**
      - `bool`, `False`
      - Set to `True` to disable the safety stop functionality

    * - **use_without_scan_sensor**
      - `bool`, `False`
      - Set to `True` to use the drive manager without 360deg scan sensor, e.g. for robots with only front and back ultrasound sensors

    * - **use_gpu**
      - `bool`, `True`
      - Use GPU implementation for the critical zone checking if available, otherwise use CPU implementation

    * - **sensor_data_timeout**
      - `float`, `0.2`
      - Maximum age (seconds) of a safety sensor's last message before it is considered stale

    * - **stale_sensor_policy**
      - `str`, `"stop"`
      - What to do when a safety sensor goes stale: `"stop"` triggers an emergency stop until data returns, `"skip"` runs the check on the remaining sensors only

    ```
    """

    closed_loop: bool = field(default=True)

    closed_loop_span: int = field(
        default=3, validator=BaseValidators.in_range(min_value=1, max_value=10)
    )

    smooth_commands: bool = field(default=False)

    cmd_tolerance: float = field(
        default=0.05
    )  # tolerance value when checking for reaching the desired command in closed loop

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
    use_without_scan_sensor: bool = field(
        default=False
    )  # Use the component without 360deg scan sensor
    use_gpu: bool = field(default=True)
    sensor_data_timeout: float = field(
        default=0.2, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e3)
    )  # Maximum sensor message age before it counts as stale (seconds)
    stale_sensor_policy: str = field(
        default="stop", validator=BaseValidators.in_(["stop", "skip"])
    )  # "stop": stale safety sensor triggers emergency stop; "skip": check runs on the remaining sensors


class DriveManager(Component):
    """
    DriveManager component used for direct communication with the robot.


    ## Inputs:
    ```{list-table}
    :widths: 10 40 10 40
    :header-rows: 1
    * - Key Name
      - Allowed Types
      - Number
      - Default

    * - **intermediate_cmd**
      - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
      - 1
      - `Topic(name="/control", msg_type="Twist")`

    * - **intermediate_cmd_list**
      - [`kompass_interfaces.msg.TwistArray`](https://github.com/automatika-robotics/kompass/tree/main/kompass_interfaces/msg)
      - 1
      - `Topic(name="/control_list", msg_type="TwistArray")`

    * - **spatial_sensor**
      - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html), [`std_msgs.msg.Float32`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html), [`std_msgs.msg.Float64`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html)
      - 1 to 10
      - `Topic(name="/scan", msg_type="LaserScan")`

    * - **robot_location**
      - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
      - 1
      - `Topic(name="/odom", msg_type="Odometry")`
    ```

    ## Outputs:

    ```{list-table}
    :widths: 10 40 10 40
    :header-rows: 1
    * - Key Name
      - Allowed Types
      - Number
      - Default

    * - **final_command**
      - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
      - 1
      - `Topic(name="/cmd_vel", msg_type="Twist")`

    * - **emergency**
      - [`std_msgs.msg.Bool`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)
      - 1
      - `Topic(name="/emergency_stop", msg_type="Bool")`
        ```
    """

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

        # Turn on robot plugin Handling
        config._enable_plugin_actions_handling = True

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
            self.robot_geometry_type, self.robot.geometry_params
        )

        self.robot_height = RobotGeometry.get_height(
            self.robot_geometry_type, self.robot.geometry_params
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

        self.slow_down_factor: Dict[str, float] = {}

        # Safety checkers get initialized on activation, once the per-sensor
        # transformations and first data are available.
        self._pc_checker = None  # All pointcloud sensors share ONE batched checker.
        self._scan_checker = None  # Single laserscan checker
        self._pc_callbacks: Tuple[PointCloudCallback, ...] = ()
        self._scan_callback: Optional[LaserScanCallback] = None
        # Arrival stamps (time.monotonic) for staleness gating
        self._pc_last_msg: List[float] = []  # One per pointcloud sensor
        self._scan_last_msg: float = 0.0  # One for laserscan sensor
        # Set once at activation so the per-tick path reads plain values
        self._stale_stop: bool = True
        self._sensor_timeout: float = self.config.sensor_data_timeout

        # NOTE: proximity sensor data transformation is deliberately NOT set to the callback here.
        # CriticalZoneChecker takes its input in the sensor frame and applies
        # the sensor->body transform itself (it is handed at construction),

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
                if not isinstance(callback, (LaserScanCallback, PointCloudCallback)):
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
            else None,
            clear_last=False,
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

        filtered_output: Twist = (
            self._filter_commands(output=output) if smooth_cmds else output
        )
        if closed_loop:
            self.execute_cmd_closed_loop(
                filtered_output, self.config.closed_loop_span / self.config.loop_rate
            )
        else:
            # Publish once in open loop
            self._publish_cmd(
                filtered_output.linear.x,
                filtered_output.linear.y,
                filtered_output.angular.z,
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

        # If no safety checker could be set up and safety stop is enabled ->
        # raise an emergency stop flag to block publishing
        if (
            not self.config.disable_safety_stop
            and not self.config.use_without_scan_sensor
            and self._pc_checker is None
            and self._scan_checker is None
        ):
            self.get_logger().warning(
                "Proximity sensor data is not available -> blocking command publishing to robot.",
                once=True,
            )
            self.slow_down_factor["unavailable_data"] = 0.0
        else:
            self.slow_down_factor["unavailable_data"] = 1.0

    def _publish_cmd(
        self,
        vx_out: float,
        vy_out: float,
        omega_out: float,
        slowdown_factor: Optional[float] = None,
    ):
        """Publish command to the robot

        :param cmd: Velocity Twist message
        :type cmd: Twist
        """
        # Check emergency stop
        if not slowdown_factor:
            if self._pc_checker or self._scan_checker:
                self._update_state()
                # Check emergency stop from all safety sensors in the direction
                # of the command
                self.slow_down_factor["scan_data"] = self._run_safety_check(
                    forward=(vx_out >= 0.0)
                )
            slowdown_val: float = min(self.slow_down_factor.values(), default=1.0)
        else:
            slowdown_val = slowdown_factor

        if slowdown_val == 0.0:
            # STOP ROBOT
            self.get_publisher(TopicsKeys.EMERGENCY).publish(True)
            self.get_logger().warning("EMERGENCY STOP ON")
            # Publish zero velocity command
            self.get_publisher(TopicsKeys.FINAL_COMMAND).publish([0.0, 0.0, 0.0])
            return
        # Publish command with slowdown
        self.get_publisher(TopicsKeys.FINAL_COMMAND).publish([
            vx_out * slowdown_val,
            vy_out * slowdown_val,
            omega_out * slowdown_val,
        ])

    def _run_safety_check(self, forward: bool) -> float:
        """Runs the critical zone check on every safety sensor and returns the
        minimum safety factor across them.

        All pointcloud sensors go through one batched checker (`clouds[i]`
        pairs with sensor i, `None` = no data this tick); a laserscan sensor
        has its own single-sensor checker. A sensor whose last message is
        older than `sensor_data_timeout` is handled per `stale_sensor_policy`:
        "stop" -> immediate 0.0, "skip" -> excluded from the check. If every
        sensor is stale/skipped nothing constrains the robot, which must read
        as unsafe -> 0.0.

        Any checker error stops the robot and keeps the component alive.

        :param forward: True if the robot is moving forward
        :type forward: bool

        :return: Slowdown factor [0.0, 1.0]; 0.0 = emergency stop
        :rtype: float
        """
        factor = 1.0
        any_checked = False
        now = time.monotonic()
        try:
            if self._scan_checker is not None:
                scan_factor = self._check_scan(forward, now)
                if scan_factor is not None:
                    if scan_factor == 0.0:
                        return 0.0
                    factor = scan_factor
                    any_checked = True
            if self._pc_checker is not None:
                gathered = self._gather_clouds(now)
                if gathered is None:
                    # Stale pointcloud sensor under the "stop" policy
                    return 0.0
                clouds, fresh = gathered
                if fresh:
                    factor = min(
                        factor, self._pc_checker.check(clouds=clouds, forward=forward)
                    )
                    any_checked = True
            if not any_checked:
                # Every safety sensor skipped.
                self.get_logger().warning(
                    "All safety sensors are stale -> Triggering emergency stop"
                )
                return 0.0
        except Exception as e:
            # A checker that cannot evaluate the sensor data must stop the robot
            # and keep the component alive
            self.get_logger().error(
                f"CriticalZoneChecker failed on incoming sensor data: {e} -> Triggering emergency stop"
            )
            return 0.0
        return factor

    def _check_scan(self, forward: bool, now: float) -> Optional[float]:
        """Runs the laserscan checker; returns its factor, 0.0 for a stale
        sensor under the "stop" policy, or None when the sensor is skipped

        :param forward: True if the robot is moving forward
        :type forward: bool
        :param now: Current time.monotonic() reading
        :type now: float
        """
        scan = (
            self._scan_callback.get_output()
            if now - self._scan_last_msg <= self._sensor_timeout
            else None
        )
        if scan is None:
            if self._stale_stop:
                self.get_logger().warning(
                    "LaserScan safety sensor is stale -> Triggering emergency stop"
                )
                return 0.0
            return None
        return self._scan_checker.check(ranges=scan.ranges, forward=forward)

    def _gather_clouds(self, now: float) -> Optional[Tuple[List[Optional[dict]], int]]:
        """Builds the batched checker's cloud list (one metadata-only dict per
        pointcloud sensor, None for a skipped stale sensor) and the count of
        fresh entries. Returns None when a stale sensor demands a stop.

        :param now: Current time.monotonic() reading
        :type now: float
        """
        clouds: List[Optional[dict]] = []
        fresh = 0
        for i, callback in enumerate(self._pc_callbacks):
            pc = (
                callback.get_output()
                if now - self._pc_last_msg[i] <= self._sensor_timeout
                else None
            )
            if pc is None:
                if self._stale_stop:
                    self.get_logger().warning(
                        "PointCloud safety sensor is stale -> Triggering emergency stop"
                    )
                    return None
                clouds.append(None)
                continue
            # Metadata-only view of the raw buffer (zero-copy contract)
            clouds.append(pc.buffer_layout())
            fresh += 1
        return clouds, fresh

    def _stamp_pc_arrival(self, sensor_idx: int, **_):
        """Record a pointcloud sensor message arrival (staleness gating)"""
        self._pc_last_msg[sensor_idx] = time.monotonic()

    def _stamp_scan_arrival(self, **_):
        """Record a laserscan sensor message arrival (staleness gating)"""
        self._scan_last_msg = time.monotonic()

    def execute_cmd_closed_loop(self, output: Twist, max_time: float):
        """Execute a control command in closed loop

        :param cmd: Velocity Twist message
        :type cmd: Twist
        :param max_time: Maximum time for the closed loop execution (s)
        :type max_time: float
        """
        if not self.robot_state:
            self.get_logger().warning(
                "Robot state is not available and command publish is set to closed loop -> disabling command publish to robot. To use the DriveManager without robot state set 'closed_loop' to 'False'"
            )
            return

        executing_closed_loop = True
        _step = 1 / self.config.loop_rate
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
            self._publish_cmd(vx_out, vy_out, omega_out)
            time.sleep(_step)

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "move_forward",
                "description": "Move the robot forward by a given distance while checking for obstacles. "
                "The robot will stop early if an obstacle is detected in the forward direction. "
                "Use when the user asks the robot to move forward, advance, or go straight ahead.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "max_distance": {
                            "type": "number",
                            "description": "Distance to move forward in meters. Map user instructions like 'move forward 1 meter' to max_distance=1.0.",
                        },
                    },
                    "required": ["max_distance"],
                },
            },
        }
    )
    def move_forward(self, max_distance: float, **_) -> bool:
        """Moves the robot forward if the forward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """

        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.loop_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while (
            unblocking
            and traveled_distance < max_distance
            and (self._pc_checker or self._scan_checker)
        ):
            # Check if max_distance forward is clear
            self._update_state()
            # A check that fails or has only stale data treats the direction
            # as blocked (helper returns 0.0)
            slowdown_factor = self._run_safety_check(forward=True)
            if slowdown_factor == 0.0:
                unblocking = False
            else:
                self._publish_cmd(
                    self.robot.ctrl_vx_limits.max_vel / 2,
                    0.0,
                    0.0,
                    slowdown_factor=slowdown_factor,
                )
                traveled_distance += step_distance
                time.sleep(1 / self.config.loop_rate)

        # Return true if unblocking forward is done
        return traveled_distance >= max_distance

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "move_backward",
                "description": "Move the robot backward by a given distance while checking for obstacles behind it. "
                "The robot will stop early if an obstacle is detected in the backward direction. "
                "Use when the user asks the robot to move back, reverse, or go backwards.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "max_distance": {
                            "type": "number",
                            "description": "Distance to move backward in meters. Map user instructions like 'go back 0.5 meters' to max_distance=0.5.",
                        },
                    },
                    "required": ["max_distance"],
                },
            },
        }
    )
    def move_backward(self, max_distance: float, **_) -> bool:
        """Moves the robot backwards if the backward direction is clear of obstacles

        :param max_distance: Maximum distance (m)
        :type max_distance: float

        :return: If the movement action is performed
        :rtype: bool
        """
        unblocking = True
        step_distance = self.robot.ctrl_vx_limits.max_vel / (2 * self.config.loop_rate)
        traveled_distance = 0.0

        # FRONT MOVEMENT
        while (
            unblocking
            and traveled_distance < max_distance
            and (self._pc_checker or self._scan_checker)
        ):
            # Check if max_distance behind the robot is clear
            self._update_state()
            # A check that fails or has only stale data treats the direction
            # as blocked (helper returns 0.0)
            slowdown_factor = self._run_safety_check(forward=False)
            if slowdown_factor == 0.0:
                unblocking = False
            else:
                self._publish_cmd(
                    -self.robot.ctrl_vx_limits.max_vel / 4,
                    0.0,
                    0.0,
                    slowdown_factor=slowdown_factor,
                )
                traveled_distance += step_distance
                time.sleep(1 / self.config.loop_rate)

        # Return true if unblocking forward is done
        return traveled_distance >= max_distance

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "rotate_in_place",
                "description": "Rotate the robot in place by a given angle. Checks that the area around the robot is clear before rotating. "
                "Will not work for Ackermann-type robots (car-like steering). "
                "Use when the user asks the robot to turn, rotate, spin, or face a different direction.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "max_rotation": {
                            "type": "number",
                            "description": "Maximum rotation angle in radians. Convert user requests from degrees to radians "
                            "(e.g. 'turn 90 degrees' -> max_rotation=1.5708). Positive values rotate counter-clockwise.",
                        },
                        "safety_margin": {
                            "type": "number",
                            "description": "Minimum clearance around the robot in meters required to perform the rotation. "
                            "Defaults to 5% of the robot radius if not specified.",
                        },
                    },
                    "required": ["max_rotation"],
                },
            },
        }
    )
    def rotate_in_place(
        self, max_rotation: float, safety_margin: Optional[float] = None, **_
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
        traveled_radius = 0.0

        if not safety_margin:
            # Set by default to 10% of the robot radius
            safety_margin = 0.05 * self.robot_radius

        # FRONT MOVEMENT
        while unblocking and traveled_radius < max_rotation:
            self._update_state()
            # Rotation needs BOTH directions clear. A check that fails or has
            # only stale data treats the rotation as blocked (helper returns 0.0)
            slowdown_factor = self._run_safety_check(forward=True)
            if slowdown_factor > 0.0:
                slowdown_factor = min(
                    slowdown_factor, self._run_safety_check(forward=False)
                )
            if slowdown_factor == 0.0:
                unblocking = False
            else:
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish([
                    0.0,
                    0.0,
                    self.robot.ctrl_omega_limits.max_vel / 2,
                ])
                traveled_radius += self.robot.ctrl_omega_limits.max_vel / (
                    2 * self.config.loop_rate
                )
                time.sleep(1 / self.config.loop_rate)

        # Return true if unblocking forward is done
        return traveled_radius >= max_rotation

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "move_to_unblock",
                "description": "Attempt to free the robot when it is stuck or blocked by obstacles. "
                "Tries moving forward first, then backward, then rotating in place until one succeeds. "
                "Requires sensor data (LaserScan or PointCloud) to check surroundings. "
                "Use when the robot is stuck, blocked, or unable to proceed along its path.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "max_distance_forward": {
                            "type": "number",
                            "description": "Maximum forward distance to try in meters. Defaults to 2x the robot radius.",
                        },
                        "max_distance_backwards": {
                            "type": "number",
                            "description": "Maximum backward distance to try in meters. Defaults to 2x the robot radius.",
                        },
                        "max_rotation": {
                            "type": "number",
                            "description": "Maximum rotation angle to try in radians. Defaults to pi/2 (~90 degrees).",
                        },
                        "rotation_safety_margin": {
                            "type": "number",
                            "description": "Clearance required around the robot to attempt rotation in meters. Defaults to 5% of robot radius.",
                        },
                    },
                    "required": [],
                },
            },
        }
    )
    def move_to_unblock(
        self,
        max_distance_forward: Optional[float] = None,
        max_distance_backwards: Optional[float] = None,
        max_rotation: float = np.pi / 2,
        rotation_safety_margin: Optional[float] = None,
        **_,
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
        if not (self._pc_checker or self._scan_checker):
            self.get_logger().error(
                "Proximity sensor data unavailable - Unblocking functionality requires LaserScan or PointCloud information"
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

    def __filter_multi_cmds(self, cmd_list: list, max_acc: float, max_vel: float):
        """Smooth the multi-cmds

        :param cmd_list: List of commands
        :type cmd_list: list
        :param max_acc: Maximum acceleration
        :type max_acc: float
        :param max_vel: Maximum velocity
        :type max_vel: float
        :param loop_rate: Loop rate (Hz)
        :type loop_rate: float

        :return: Smoothed commands
        :rtype: list
        """
        # TODO: Requires further optimization
        # Use a low pass filter based on maximum allowed acceleration/vel if multi commands are available
        cmds = np.array(cmd_list)
        freq_limit = (max_acc / max_vel) / 2
        bandlimit_index = int(freq_limit * cmds.shape[0] * self.config.loop_rate)
        fsig = np.fft.fft(cmds)
        fsig[bandlimit_index + 1 : -bandlimit_index] = 0
        adata_filtered = np.fft.ifft(fsig)
        return np.real(adata_filtered).tolist()

    def _filter_multi_commands(self, output: TwistArray, **_) -> TwistArray:
        """
        Filters commands list using a low pass filter based on acceleration limit
        """

        # Use a low pass filter based on maximum allowed acceleration if multi commands are available
        self._filtered_linear_commands_x = self.__filter_multi_cmds(
            output.linear_velocities.x,
            self.robot.ctrl_vx_limits.max_acc,
            self.robot.ctrl_vx_limits.max_vel,
        )

        self._filtered_linear_commands_y = self.__filter_multi_cmds(
            output.linear_velocities.y,
            self.robot.ctrl_vy_limits.max_acc,
            self.robot.ctrl_vy_limits.max_vel,
        )

        self._filtered_angular_commands = self.__filter_multi_cmds(
            output.angular_velocities.z,
            self.robot.ctrl_omega_limits.max_acc,
            self.robot.ctrl_omega_limits.max_vel,
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
            self.robot.ctrl_vx_limits.max_acc,
            self.robot.ctrl_vx_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.linear.x = self._limit_command_acc(
                output.linear.x,
                self._previous_command.linear.x,
                self.robot.ctrl_vx_limits.max_acc,
                self.robot.ctrl_vx_limits.max_decel,
                self.config.loop_rate,
            )
        else:
            _cmd.linear.x = output.linear.x

        if self._check_bounds(
            output.linear.y,
            self._previous_command.linear.y,
            self.robot.ctrl_vy_limits.max_acc,
            self.robot.ctrl_vy_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.linear.y = self._limit_command_acc(
                output.linear.y,
                self._previous_command.linear.y,
                self.robot.ctrl_vy_limits.max_acc,
                self.robot.ctrl_vy_limits.max_decel,
                self.config.loop_rate,
            )
        else:
            _cmd.linear.x = output.linear.x

        # Check and restrict angular velocity
        if self._check_bounds(
            output.angular.z,
            self._previous_command.angular.z,
            self.robot.ctrl_omega_limits.max_acc,
            self.robot.ctrl_omega_limits.max_decel,
            self.config.loop_rate,
        ):
            _cmd.angular.z = self._limit_command_acc(
                output.angular.z,
                self._previous_command.angular.z,
                self.robot.ctrl_omega_limits.max_acc,
                self.robot.ctrl_omega_limits.max_decel,
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

    def _limit_command_vel(
        self, output: Union[np.ndarray, list]
    ) -> Union[np.ndarray, list]:
        """Check and limit the control commands

        :param cmd: Robot control command
        :type cmd: Twist

        :return: False if no control command is available
        :rtype: bool
        """

        if abs(output[0]) > self.robot.ctrl_vx_limits.max_vel:
            self.get_logger().debug(
                f"Limiting linear velocity by allowed maximum {self.robot.ctrl_vx_limits.max_vel}"
            )
            output[0] = np.sign(output[0]) * self.robot.ctrl_vx_limits.max_vel
        elif abs(output[0]) < self.robot.ctrl_vx_limits.min_absolute_val:
            output[0] = 0.0

        if abs(output[1]) > self.robot.ctrl_vy_limits.max_vel:
            self.get_logger().debug(
                f"Limiting linear Vy velocity by allowed maximum {self.robot.ctrl_vy_limits.max_vel}"
            )
            output[1] = np.sign(output[1]) * self.robot.ctrl_vy_limits.max_vel
        elif abs(output[1]) < self.robot.ctrl_vy_limits.min_absolute_val:
            output[1] = 0.0

        if abs(output[2]) > self.robot.ctrl_omega_limits.max_vel:
            self.get_logger().debug(
                f"Limiting angular velocity by allowed maximum {self.robot.ctrl_omega_limits.max_vel}"
            )
            output[2] = np.sign(output[2]) * self.robot.ctrl_omega_limits.max_vel
        elif abs(output[2]) < self.robot.ctrl_omega_limits.min_absolute_val:
            output[2] = 0.0
        return output

    def _execution_step(self):
        """
        Main execution of the component, executed at ech timer tick with rate self.config.loop_rate
        """
        if self._unblocking_on:
            return
        # Check emergency stop
        self._update_state()
        speed_factor = min(self.slow_down_factor.values(), default=1.0)
        if speed_factor < 0.1:
            # STOP ROBOT
            self.get_publisher(TopicsKeys.EMERGENCY).publish(True)
            return
        else:
            self.get_publisher(TopicsKeys.EMERGENCY).publish(False)

        if speed_factor == 0.0:
            self.get_logger().warning(
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
        if self.config.closed_loop:
            _cmd_vel = Twist()
            _cmd_vel.linear.x = cmd[0]
            _cmd_vel.linear.y = cmd[1]
            _cmd_vel.angular.z = cmd[2]
            self.execute_cmd_closed_loop(_cmd_vel, max_time=self._multi_command_step)
        else:
            # Execute cmd in open loop -> Publish once
            self._publish_cmd(cmd[0], cmd[1], cmd[2])

    def _make_checker(self, **kwargs):
        """Constructs a critical zone checker: GPU implementation when enabled
        and available, with a CPU fallback

        :return: CriticalZoneChecker or CriticalZoneCheckerGPU
        """
        if self.config.use_gpu:
            try:
                from kompass_cpp.utils import CriticalZoneCheckerGPU

                checker = CriticalZoneCheckerGPU(**kwargs)
                self.get_logger().info("Initialized CriticalZoneCheckerGPU")
                return checker
            except ImportError:
                self.get_logger().warning(
                    "GPU use is enabled but CriticalZoneCheckerGPU implementation is not found -> Using CPU implementation instead"
                )
        from kompass_cpp.utils import CriticalZoneChecker

        checker = CriticalZoneChecker(**kwargs)
        self.get_logger().info("Initialized CriticalZoneChecker")
        return checker

    def _classify_spatial_sensors(
        self,
    ) -> Tuple[List[PointCloudCallback], List[int], Optional[LaserScanCallback], int]:
        """Splits the spatial sensor callbacks ONCE: every PointCloud2 topic
        feeds one batched checker; the first LaserScan topic gets its own checker.
        Scalar proximity sensors are handled separately by their own callback hook.

        :return: (pointcloud callbacks, their topic indices, laserscan
            callback or None, its topic index)
        """
        num_sensors = self._inputs_keys.count(TopicsKeys.SPATIAL_SENSOR)
        pc_callbacks: List[PointCloudCallback] = []
        pc_indices: List[int] = []
        scan_callback: Optional[LaserScanCallback] = None
        scan_idx: int = 0
        for idx in range(num_sensors):
            callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx)
            if isinstance(callback, PointCloudCallback):
                pc_callbacks.append(callback)
                pc_indices.append(idx)
            elif isinstance(callback, LaserScanCallback):
                if scan_callback is None:
                    scan_callback = callback
                    scan_idx = idx
                else:
                    self.get_logger().warning(
                        "Multiple LaserScan sensors are set but the critical zone checker "
                        f"supports exactly one -> '{callback.input_topic.name}' is ignored "
                        "for safety checks"
                    )
        return pc_callbacks, pc_indices, scan_callback, scan_idx

    def _init_safety_checkers(
        self,
        pc_callbacks: List[PointCloudCallback],
        pc_indices: List[int],
        scan_callback: Optional[LaserScanCallback],
        scan_idx: int,
    ):
        """Constructs the batched pointcloud checker and/or the laserscan
        checker, waiting per sensor for its static TF and first data"""

        # Common checker parameters. min/max_height form a BODY-frame band
        # shared by all sensors. Each sensor's mount transform is applied
        # inside.
        common_kwargs = {
            "robot_shape": self.robot_geometry_type,
            "robot_dimensions": self.robot.geometry_params,
            "critical_angle": self.config.critical_zone_angle,
            "critical_distance": self.config.critical_zone_distance,
            "slowdown_distance": self.config.slowdown_zone_distance,
            "min_height": 0.0,
            "max_height": self.robot_height,
            "range_max": 3 * self.config.slowdown_zone_distance,
        }

        if pc_callbacks:
            # One SensorConfig per pointcloud sensor. Get mount pose from TF, point
            # field encoding from the sensor's first message
            sensor_configs = [
                self.wait_sensor_config(TopicsKeys.SPATIAL_SENSOR, idx)
                for idx in pc_indices
            ]
            self._pc_checker = self._make_checker(
                input_type=SensorInputType.POINTCLOUD,
                sensor_configs=sensor_configs,
                **common_kwargs,
            )
        if scan_callback is not None:
            sensor = self.wait_sensor_config(TopicsKeys.SPATIAL_SENSOR, scan_idx)
            # Get first scan. The wait above guarantees a decoded first scan
            scan = scan_callback.get_output()
            self._scan_checker = self._make_checker(
                input_type=SensorInputType.LASERSCAN,
                sensor_configs=[sensor],
                scan_angles=scan.angles,
                **common_kwargs,
            )

    def _execute_once(self):
        """Actions to be executed once at the start of the component execution"""
        super()._execute_once()

        if self.config.use_without_scan_sensor and not self.config.disable_safety_stop:
            self.get_logger().warning(
                "Using DriveManager without 360deg scan sensor and Safety stop functionality is still enabled."
            )
            return

        if self.config.disable_safety_stop:
            self.get_logger().warning("Safety Stop is Disabled!")
            return

        pc_callbacks, pc_indices, scan_callback, scan_idx = (
            self._classify_spatial_sensors()
        )

        if not pc_callbacks and scan_callback is None:
            self.get_logger().error(
                "Cannot initialize CriticalZoneChecker: no LaserScan or PointCloud2 "
                "sensor is configured -> Safety Stop is disabled!"
            )
            # Set failure based on the fact that no spatial sensor provides valid
            # data. The key can be bound to several sensors.
            sensor_names = self.in_topic_name(TopicsKeys.SPATIAL_SENSOR) or []
            self.health_status.set_fail_system(
                topic_names=sensor_names
                if isinstance(sensor_names, list)
                else [sensor_names]
            )
            return

        self._init_safety_checkers(pc_callbacks, pc_indices, scan_callback, scan_idx)
        self.get_logger().info("Got Proximity Sensor TF...")

        # Freeze the per-tick safety state (for the hot path)
        self._pc_callbacks = tuple(pc_callbacks)
        self._scan_callback = scan_callback
        now = time.monotonic()
        self._pc_last_msg = [now] * len(pc_callbacks)
        self._scan_last_msg = now
        self._stale_stop = self.config.stale_sensor_policy == "stop"
        self._sensor_timeout = self.config.sensor_data_timeout
        for i, callback in enumerate(pc_callbacks):
            callback.on_callback_execute(
                partial(self._stamp_pc_arrival, sensor_idx=i), get_processed=False
            )
        if scan_callback is not None:
            scan_callback.on_callback_execute(
                self._stamp_scan_arrival, get_processed=False
            )

        # Warm-up to avoid first-call overhead. A malformed first input will read
        # as an emergency stop until the data is valid
        self._run_safety_check(forward=True)
        self.get_logger().info("CriticalZoneChecker: Warm-up complete - Ready to go!")
