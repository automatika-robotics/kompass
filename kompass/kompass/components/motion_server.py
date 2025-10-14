import os
from typing import Dict, List, Optional

import numpy as np
from attrs import define, field
from geometry_msgs.msg import Twist, Vector3
from kompass_core.models import RobotState
from kompass_core.py_path_tools.executor import PathExecutor
from rclpy.callback_groups import CallbackGroup

from kompass_interfaces.action import MotionRecording

from ..config import BaseValidators, ComponentConfig, ComponentRunType
from .ros import Topic, update_topics
from .component import Component
from .defaults import (
    TopicsKeys,
    motion_server_allowed_inputs,
    motion_server_allowed_outputs,
    motion_server_default_inputs,
    motion_server_default_outputs,
)


@define(kw_only=True)
class MotionServerConfig(ComponentConfig):
    """
    MotionServer config parameters
    """

    test_period: float = field(
        default=10.0, validator=BaseValidators.in_range(min_value=0.1, max_value=1e3)
    )  # sec
    tests_folder: str = field(default="/kompass/tests")

    run_step_test: bool = field(default=False)
    run_circle_test: bool = field(default=True)


class MotionServer(Component):
    """
    MotionServer component used for automatic testing by sending reference commands and recording resulting motion.

    ## Input Topics:
    - *run*: Run tests on start<br />  Default ```Topic(name="/run_tests", msg_type="Bool")```
    - *robot_command*:  Control command.<br />  Default ```Topic(name="/cmd_vel", msg_type="Twist")```

    ## Output Topics:
    - *command*: Control Command<br />  Default ```Topic(name="/control", msg_type="Twist")```

    :::{note} Topic for 'Control Command' is both in MotionServer inputs and outputs:
      - The output is used when running automated testing (i.e. sending the commands directly from the MotionServer).
      - The input is used to purely record motion and control from external sources (example: recording output from Controller).
      - Different command topics can be configured for the input and the output. For example: to test the DriveManager, the control command from MotionServer output can be sent to the DriveManager, then the DriveManager output can be configured as the MotionServer input for recording.
    :::

    ## Available Run Types:
    Set directly from MotionServer 'run_type' property.

    - *TIMED*: Launches an automated test periodically after start.
    - *EVENT*: Launches automated testing when a trigger ir received on RUN input.
    - *ACTIONSERVER*: Offers a MotionRecording ROS action to record motion for location and control commands topics for given recording period.

    ## Usage Example:
    ```
    ```
    """

    def __init__(
        self,
        *,
        component_name: str,
        config: Optional[MotionServerConfig] = None,
        config_file: Optional[str] = None,
        robot_cmd_topic: Optional[Topic] = None,
        robot_odom_topic: Optional[Topic] = None,
        callback_group: Optional[CallbackGroup] = None,
        **kwargs,
    ):
        """__init__.

        :param component_name:
        :type component_name: str
        :param config:
        :type config: Optional[MotionServerConfig]
        :param config_file:
        :type config_file: Optional[str]
        :param robot_cmd_topic:
        :type robot_cmd_topic: Optional[Topic]
        :param robot_odom_topic:
        :type robot_odom_topic: Optional[Topic]
        :param callback_group:
        :type callback_group: Optional[CallbackGroup]
        :param kwargs:
        """
        if not config:
            config = MotionServerConfig()

        # Get default component outputs
        out_topics = motion_server_default_outputs
        in_topics = motion_server_default_inputs

        if robot_cmd_topic:
            out_topics = update_topics(out_topics, robot_command=robot_cmd_topic)
            in_topics = update_topics(in_topics, command=robot_cmd_topic)

        if robot_odom_topic:
            in_topics = update_topics(in_topics, location=robot_odom_topic)

        super().__init__(
            config=config,
            config_file=config_file,
            component_name=component_name,
            callback_group=callback_group,
            outputs=out_topics,
            inputs=in_topics,
            allowed_inputs=motion_server_allowed_inputs,
            allowed_outputs=motion_server_allowed_outputs,
            allowed_run_types=[
                ComponentRunType.EVENT,
                ComponentRunType.ACTION_SERVER,
                ComponentRunType.TIMED,
            ],
            **kwargs,
        )
        self.config: MotionServerConfig
        self.action_type = MotionRecording

    def _update_state(self) -> None:
        """
        Updates all inputs
        """
        self.robot_cmd: Optional[Twist] = self.get_callback(
            TopicsKeys.INTERMEDIATE_CMD
        ).get_output()

        self.robot_state: Optional[RobotState] = self.get_callback(
            TopicsKeys.ROBOT_LOCATION
        ).get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

    def _execute_once(self) -> None:
        """Init recording for timed component"""
        if self.run_type == ComponentRunType.EVENT:
            # Run tests on True
            self.attach_custom_callback(
                self.in_topic_name(TopicsKeys.RUN_TESTS), self.run_motion_response_tests
            )

        # check if topic being published
        while not self.got_all_inputs():
            self.get_logger().warn(
                "Timed MotionServer waiting for inputs to start recording", once=True
            )
            pass
        # Setup the executor to start recording
        self.path_executor.start_motion_recording(
            recording_period=self.config.test_period,
            recording_step=1 / self.config.loop_rate,
        )

        self.motion_recording_max = self.config.test_period + self.get_secs_time()

    def _execution_step(self) -> None:
        """Record values for timed component"""
        time_now: float = self.get_secs_time()
        if time_now <= self.motion_recording_max:
            self._record_motion_from_state()

        else:
            self._end_motion_recording(
                path_to_file=self.config.tests_folder, file_name=f"test_{time_now}"
            )
            self.motion_recording_max = self.config.test_period + self.get_secs_time()
            # Setup the executor to start recording
            self.path_executor.start_motion_recording(
                recording_period=self.config.test_period,
                recording_step=1 / self.config.loop_rate,
            )

    def init_variables(self) -> None:
        """init_variables."""
        self.motion_recording_max: float = 0.0
        self.path_executor = PathExecutor()

    def main_action_callback(
        self, goal_handle: MotionRecording.Goal
    ) -> MotionRecording.Result:
        """Execute motion recording action

        :param goal_handle: Action goal
        :type goal_handle: MotionRecording.Goal
        :return: Action result
        :rtype: MotionRecording.Result
        """
        # Clear existing path to record new
        self.init_variables()

        time_now = self.get_secs_time()

        request = goal_handle.request

        motion_file_location = request.csv_file_location
        motion_file_name = request.csv_file_name
        motion_recording_max = request.recording_period_secs + time_now

        server_rate = self.create_rate(1 / request.recording_time_step_secs)

        # Setup response and feedback of the action
        action_feedback_msg = MotionRecording.Feedback()

        action_result = MotionRecording.Result()

        if request.recording_period_secs <= 0 or request.recording_time_step_secs <= 0:
            action_result.recording_done = False
            action_result.message = f"Recording period {request.recording_period_secs} and step {request.recording_time_step_secs} must be greater than zero"
            return action_result

        if not os.path.exists(motion_file_location):
            action_result.recording_done = False
            action_result.message = "Please provide a valid file save location in the request to start recording"
            return action_result

        if motion_file_name == "":
            motion_file_name = "motion_test"
            self.get_logger().warn(
                f"Motion file save name is not provided in the request -> saving motion in default name {motion_file_name}"
            )

        self.get_logger().info(
            f"Got recording request from topics {request.robot_odom_topic_name} and {request.robot_cmd_topic_name}, Checking topics..."
        )

        # check if topic being published
        if self.got_all_inputs():
            # Setup the executor to start recording
            self.path_executor.start_motion_recording(
                recording_period=request.recording_period_secs,
                recording_step=request.recording_time_step_secs,
            )

            while time_now <= motion_recording_max:
                self._update_state()

                continue_recording: bool = self._record_motion_from_state(
                    action_feedback_msg
                )
                time_now = self.get_secs_time()

                if not continue_recording:
                    goal_handle.abort()
                    break

                goal_handle.publish_feedback(action_feedback_msg)
                server_rate.sleep()

            # Save recorded motion to file
            file_saved: bool = self._end_motion_recording(
                motion_file_location, motion_file_name
            )
            action_result.recording_done = file_saved
            if file_saved:
                action_result.message = f"Done recording motion and data is saved to {motion_file_location}/{motion_file_name}"
            else:
                action_result.message = (
                    "Error saving data to file after motion recording"
                )

        else:
            self.get_logger().warn(
                f"Requested topics not being published: {self.get_missing_inputs()}"
            )
            action_result.recording_done = False
            action_result.message = (
                f"Requested topics not being published {self.get_missing_inputs()}"
            )

        return action_result

    def _record_motion_from_state(
        self, feedback_msg: Optional[MotionRecording.Feedback] = None
    ) -> bool:
        """
        Records a new motion point from the robot state and robot command and updates the feedback message

        :param feedback_msg: MotionRecording feedback message to be updated
        :type feedback_msg: MotionRecording.Feedback

        :return: If a new point is recorded
        :rtype: bool
        """
        if self.robot_state and self.robot_cmd:
            # Get current time as float in seconds
            time_float: float = self.get_secs_time()

            # Record new point
            continue_recording = self.path_executor.record_motion_point(
                x=self.robot_state.x,
                y=self.robot_state.y,
                heading=self.robot_state.yaw,
                linear_ctr_x=self.robot_cmd.linear.x,
                linear_ctr_y=self.robot_cmd.linear.y,
                angular_ctr=self.robot_cmd.angular.z,
                time=time_float,
            )
            continue_recording = True

            if feedback_msg:
                # Update feedback
                feedback_msg.position = Vector3(
                    x=self.robot_state.x, y=self.robot_state.y, z=0.0
                )
                feedback_msg.heading = self.robot_state.yaw
                feedback_msg.cmd = Vector3(
                    x=self.robot_cmd.linear.x,
                    y=self.robot_cmd.linear.y,
                    z=self.robot_cmd.angular.z,
                )

        else:
            if feedback_msg:
                feedback_msg = MotionRecording.Feedback()

            self.get_logger().warn(
                "Could not record a new point: Robot Odometry is not available or no command is sent to the robot"
            )
            continue_recording = False

        return continue_recording

    def _end_motion_recording(self, path_to_file: str, file_name: str) -> bool:
        """
        End motion recording after the requested period have passed and saves the motion points to a csv file
        """
        file_saved: bool = False

        if self.path_executor.motion_recording_idx > 0:
            self.get_logger().info(
                f"Saving {self.path_executor.motion_recording_idx} recorded motion points"
            )
            file_saved: bool = self.path_executor.save_motion_to_csv(
                path_to_file, file_name
            )

        self.init_variables()

        return file_saved

    def run_motion_response_tests(self, msg, **_) -> None:
        """
        Run a set of tests to record the robot motion response
        Used for robot model calibration
        """
        if not msg.data:
            return

        # get number of ctr steps
        N_steps = int(self.config.test_period * self.config.loop_rate)

        # generate basic tests to record the system motion response
        basic_tests = self.generate_basic_ctr_tests(N_steps)

        # LOOP RATE FOR EXECUTING TESTS
        loop_rate = self.create_rate(10 / self.config.loop_rate)

        # run tests
        for test in basic_tests:
            test_processed = self.send_test(test["test"], test["name"], N_steps)

            if not test_processed:
                self.get_logger().error("Test could not be processed -> Shutting down")
                return

            # Wait time before starting new test
            loop_rate.sleep()

        # Publish zero to end testing and stop the robot
        self.get_logger().info("Done running tests")

        # Publish zero to stop the robot
        cmd_vel = Twist()
        self.get_publisher("robot_command").publish(cmd_vel)

    def generate_basic_ctr_tests(self, number_of_steps: int) -> List[Dict]:
        """
        Generates a ser of basic tests

        :param number_of_steps: Number of time steps in each test
        :type number_of_steps: int

        :return: Constructed tests
        :rtype: list[dict]
        """
        BASIC_TESTS = []
        if self.config.run_step_test:
            BASIC_TESTS = [
                {
                    "name": "linear_step_mid",
                    "test": np.full(
                        (number_of_steps, 2),
                        [self.config.robot.ctrl_vx_limits.max_vel / 2, 0.0],
                    ),
                },
                {
                    "name": "linear_step_inv",
                    "test": np.full(
                        (number_of_steps, 2),
                        [-self.config.robot.ctrl_vx_limits.max_vel / 2, 0.0],
                    ),
                },
                {
                    "name": "ang_step_mid",
                    "test": np.full(
                        (number_of_steps, 2),
                        [0.0, self.config.robot.ctrl_omega_limits.max_vel / 2],
                    ),
                },
                {
                    "name": "ang_step_inv",
                    "test": np.full(
                        (number_of_steps, 2),
                        [0.0, -self.config.robot.ctrl_omega_limits.max_vel / 2],
                    ),
                },
            ]
        elif self.config.run_circle_test:
            BASIC_TESTS = [
                {
                    "name": "circle_forward",
                    "test": np.full(
                        (number_of_steps, 2),
                        [
                            self.config.robot.ctrl_vx_limits.max_vel / 2,
                            self.config.robot.ctrl_omega_limits.max_vel / 2,
                        ],
                    ),
                },
                {
                    "name": "circle_forward_inv",
                    "test": np.full(
                        (number_of_steps, 2),
                        [
                            -self.config.robot.ctrl_vx_limits.max_vel / 2,
                            self.config.robot.ctrl_omega_limits.max_vel / 2,
                        ],
                    ),
                },
                {
                    "name": "circle_backward",
                    "test": np.full(
                        (number_of_steps, 2),
                        [
                            self.config.robot.ctrl_vx_limits.max_vel / 2,
                            -self.config.robot.ctrl_omega_limits.max_vel / 2,
                        ],
                    ),
                },
                {
                    "name": "circle_backward_inv",
                    "test": np.full(
                        (number_of_steps, 2),
                        [
                            -self.config.robot.ctrl_vx_limits.max_vel / 2,
                            -self.config.robot.ctrl_omega_limits.max_vel / 2,
                        ],
                    ),
                },
            ]

        return BASIC_TESTS

    def send_test(self, test: np.ndarray, test_name: str, number_steps: int) -> bool:
        """
        Send a new motion recording test to the robot cmd topic

        :param test: Set of control commands
        :type test: np.ndarray
        :param test_name: Name of the test
        :type test_name: str
        :param number_steps: Test control time steps
        :type number_steps: int

        :return: Test processed
        :rtype: bool
        """

        time_now = self.get_secs_time()
        motion_recording_max: float = self.config.test_period + time_now
        # Setup the executor to start recording
        self.path_executor.start_motion_recording(
            recording_period=self.config.test_period,
            recording_step=1 / self.config.loop_rate,
        )

        loop_rate = self.create_rate(self.config.loop_rate)

        self.get_logger().info(f"Starting test: {test_name}")

        # init robot ctr command
        cmd_vel = Twist()

        while time_now <= motion_recording_max:
            time_now = self.get_secs_time()

            for cmd_idx in range(number_steps):
                # get commands from test
                cmd_vel.linear.x = test[cmd_idx, 0]
                cmd_vel.angular.z = test[cmd_idx, 1]
                # send to robot
                self.get_publisher(TopicsKeys.FINAL_COMMAND).publish(cmd_vel)

                self._update_state()

                continue_recording: bool = self._record_motion_from_state()

                if not continue_recording:
                    return False

                loop_rate.sleep()

        self._end_motion_recording(self.config.tests_folder, test_name)

        return True
