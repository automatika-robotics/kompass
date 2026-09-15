"""Functional test for the MissionManager, in a real launched stack.

The translation from a goal to a routine is checked without ROS in
`test_mission_translation.py`. What only exists once there is a stack is
everything else: that the mission registers its routine on a real Monitor,
that each waypoint reaches a real action server in order, that a dwell really
delays, that feedback tracks the cursor, and that cancelling the mission
cancels the goal in flight rather than leaving the robot driving.

The planner here is a stand-in. It runs the same action name and type the real
Planner does, so the mission is wired to it exactly as it would be in a live
stack, without the test depending on OMPL or a map. What is under test is the
mission, not path planning. The controller and drive manager are the real ones,
since stopping the robot before holding position is their actions.
"""

import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

from kompass.components import (
    Controller,
    DriveManager,
    MissionManager,
    MissionManagerConfig,
)
from kompass.components.component import Component
from kompass.config import ComponentRunType
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotConfig,
    RobotGeometryType,
    RobotType,
)
from kompass_interfaces.action import MultiGoalPlanPath
from kompass_interfaces.action import PlanPath as PlanPathAction
from kompass_interfaces.msg import MissionStatus, PathTrackingError

PAUSE_TOPIC = "/mission_go_on"
#: Where the robot stands, away from the origin so a location that was never
#: filled in does not look like one that was
ROBOT_AT = (0.5, -0.25)
ROBOT_FRAME = "map"

#: Goals the stand-in planner was asked to drive to, in the order it got them
planner_goals = []
#: Goals it saw cancelled
planner_cancels = []
#: Waypoints it should refuse, by x position
refuse_at = set()
#: Waypoints that take long to drive, by x position
slow_at = set()


class StandInPlanner(Component):
    """Runs the Planner's action interface without doing any planning.

    Same action name and type as `kompass.components.planner.Planner`, so the
    mission addresses it exactly as it would the real one.
    """

    def __init__(self, component_name: str, **kwargs):
        super().__init__(
            component_name=component_name,
            allowed_run_types=[ComponentRunType.ACTION_SERVER, ComponentRunType.TIMED],
            **kwargs,
        )
        self.action_type = PlanPathAction
        self.main_action_name = "navigate_to_goal"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self, *_, **__):
        pass

    def main_action_callback(self, goal_handle):
        goal = goal_handle.request
        planner_goals.append(round(goal.goal.position.x, 3))
        result = PlanPathAction.Result()

        if round(goal.goal.position.x, 3) in refuse_at:
            result.reached_end = False
            goal_handle.abort()
            return result

        # Long enough to still be driving when a test cancels the mission
        for _ in range(200 if round(goal.goal.position.x, 3) in slow_at else 20):
            if goal_handle.is_cancel_requested:
                planner_cancels.append(round(goal.goal.position.x, 3))
                goal_handle.canceled()
                return result
            time.sleep(0.05)

        result.reached_end = True
        goal_handle.succeed()
        return result


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    from ros_sugar import Launcher

    planner = StandInPlanner(component_name="planner")
    controller = Controller(component_name="controller")
    driver = DriveManager(component_name="drive_manager")
    # No sensors in this test
    driver.config.disable_safety_stop = True
    mission = MissionManager(
        component_name="mission",
        planner=planner,
        controller=controller,
        drive_manager=driver,
        config=MissionManagerConfig(waypoint_timeout=30.0, cursor_poll_rate=10.0),
    )

    launcher = Launcher()
    launcher.add_pkg(components=[planner])
    # In their own processes like in a recipe, so the mission reaches the others
    # only through its config
    launcher.add_pkg(
        components=[controller, driver, mission],
        package_name="kompass",
        multiprocessing=True,
    )
    launcher.robot = RobotConfig(
        model_type=RobotType.DIFFERENTIAL_DRIVE,
        geometry_type=RobotGeometryType.CYLINDER,
        geometry_params=np.array([0.1, 0.3]),
        ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
        ctrl_omega_limits=AngularCtrlLimits(
            max_omega=0.4, max_acc=2.0, max_decel=2.0, max_ang=np.pi / 3
        ),
    )
    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


def pose_at(x: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.orientation.w = 1.0
    return pose


class TestMission(unittest.TestCase):
    """Drives the mission action the way a client would"""

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        cls.context.init()
        cls.node = rclpy.create_node("mission_test_client", context=cls.context)
        cls.executor = rclpy.executors.SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.client = ActionClient(cls.node, MultiGoalPlanPath, "run_mission")
        assert cls.client.wait_for_server(timeout_sec=30.0), (
            "the mission action server never came up"
        )
        cls.pause_publisher = cls.node.create_publisher(Bool, PAUSE_TOPIC, 10)
        # A robot standing still, so the drive manager can confirm it stopped
        odom = Odometry()
        odom.header.frame_id = ROBOT_FRAME
        odom.pose.pose.position.x, odom.pose.pose.position.y = ROBOT_AT
        odom.pose.pose.orientation.w = 1.0
        odom_publisher = cls.node.create_publisher(Odometry, "/odom", 10)
        cls.odom_timer = cls.node.create_timer(
            0.1, lambda: odom_publisher.publish(odom)
        )
        cls.robot_commands = []
        cls.node.create_subscription(Twist, "/cmd_vel", cls.robot_commands.append, 10)
        cls.statuses = []
        cls.node.create_subscription(
            MissionStatus,
            "/mission_status",
            cls.statuses.append,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        cls.context.try_shutdown()

    def setUp(self):
        planner_goals.clear()
        planner_cancels.clear()
        refuse_at.clear()
        slow_at.clear()
        self.robot_commands.clear()
        self.statuses.clear()
        self.feedback = []

    def spin(self, seconds: float) -> None:
        """Let the client node process callbacks for a while"""
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.executor.spin_once(timeout_sec=0.05)

    def send(self, goal: MultiGoalPlanPath.Goal):
        """Send a mission and return its accepted goal handle"""
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback)
        )
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=15.0, executor=self.executor
        )
        handle = future.result()
        assert handle is not None and handle.accepted, "the mission was not accepted"
        return handle

    def result_of(self, handle, timeout: float = 60.0):
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout, executor=self.executor
        )
        assert future.done(), "the mission never returned a result"
        return future.result().result

    @staticmethod
    def mission(count: int = 2, **fields) -> MultiGoalPlanPath.Goal:
        goal = MultiGoalPlanPath.Goal()
        goal.goals = [pose_at(float(i + 1)) for i in range(count)]
        goal.end_tolerance = PathTrackingError(
            orientation_error=0.1, lateral_distance_error=0.2
        )
        for name, value in fields.items():
            setattr(goal, name, value)
        return goal

    # ---- Carrying out a mission ---------------------------------------

    def test_every_waypoint_reaches_the_planner_in_order(self):
        """The whole point: one action, several goals, driven one at a time"""
        result = self.result_of(self.send(self.mission(count=3)))

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        assert planner_goals == [1.0, 2.0, 3.0]
        assert list(result.reached_waypoints) == [True, True, True]
        assert result.last_reached_index == 2

    def test_feedback_tracks_which_waypoint_is_being_driven(self):
        """A mission is opaque without it: one action for minutes of driving"""
        self.result_of(self.send(self.mission(count=2)))

        assert self.feedback, "no feedback was published"
        navigating = [
            msg
            for msg in self.feedback
            if msg.state == MultiGoalPlanPath.Feedback.STATE_NAVIGATING
        ]
        assert navigating, "never reported navigating"
        assert {msg.current_goal_idx for msg in navigating} == {0, 1}
        assert all(msg.total_goals == 2 for msg in self.feedback)

    def test_progress_never_goes_back_to_an_earlier_waypoint(self):
        """Regression: the poll that saw the routine end reported navigating to
        waypoint 0, as an ended routine has no active step"""
        self.result_of(self.send(self.mission(count=2, pause_duration=[0.5])))
        self.spin(0.5)

        indices = [msg.current_goal_idx for msg in self.feedback]
        assert indices == sorted(indices), f"the feedback went back: {indices}"
        assert indices[-1] == 1
        ongoing = [status.current_goal_idx for status in self.statuses if status.mission_id]
        assert ongoing == sorted(ongoing), f"the status went back: {ongoing}"
        assert ongoing[-1] == 1

    def test_feedback_times_the_pause_and_not_the_driving(self):
        self.result_of(self.send(self.mission(count=1, pause_duration=[2.0])))

        dwelling = [
            msg.time_paused
            for msg in self.feedback
            if msg.state == MultiGoalPlanPath.Feedback.STATE_PAUSED_DWELL
        ]
        assert dwelling, "never reported dwelling"
        assert dwelling == sorted(dwelling), f"the pause time went back: {dwelling}"
        # A 2s dwell, polled at 10Hz
        assert 1.0 < dwelling[-1] < 2.5, f"timed the dwell as {dwelling[-1]:.2f}s"
        assert all(
            msg.time_paused == 0.0
            for msg in self.feedback
            if msg.state == MultiGoalPlanPath.Feedback.STATE_NAVIGATING
        ), "reported a pause time while driving"

    def test_feedback_carries_where_the_robot_is(self):
        self.result_of(self.send(self.mission(count=1)))

        assert self.feedback, "no feedback was published"
        pose = self.feedback[-1].current_pose
        assert (pose.pose.position.x, pose.pose.position.y) == pytest.approx(ROBOT_AT)
        assert pose.header.frame_id == ROBOT_FRAME

    def test_a_dwell_holds_the_mission_between_waypoints(self):
        started = time.time()
        result = self.result_of(self.send(self.mission(count=2, pause_duration=[2.0])))
        elapsed = time.time() - started

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        # Two waypoints at ~1s each, plus two dwells of 2s
        assert elapsed >= 4.0, f"the mission did not dwell, took {elapsed:.1f}s"
        assert any(
            msg.state == MultiGoalPlanPath.Feedback.STATE_PAUSED_DWELL
            for msg in self.feedback
        ), "never reported dwelling"

    def test_the_robot_is_stopped_before_a_dwell(self):
        """The planner is done within tolerance while the controller may still drive"""
        result = self.result_of(self.send(self.mission(count=1, pause_duration=[1.0])))

        # Stopping is a step of the mission, one that fails would end it
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        assert any(
            msg.linear.x == 0.0 and msg.angular.z == 0.0 for msg in self.robot_commands
        ), "the drive manager never sent the robot a zero command"

    def test_a_mission_with_no_hold_does_not_stop_the_robot(self):
        result = self.result_of(self.send(self.mission(count=2)))

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        assert not self.robot_commands, "the robot was stopped between waypoints"

    def test_a_pause_condition_holds_the_mission_until_the_topic_says_go(self):
        """The reason the topic is named per mission rather than configured"""
        handle = self.send(
            self.mission(count=1, pause_condition_topic=PAUSE_TOPIC)
        )
        result_future = handle.get_result_async()

        def paused() -> bool:
            return any(
                msg.state == MultiGoalPlanPath.Feedback.STATE_PAUSED_CONDITION
                for msg in self.feedback
            )

        deadline = time.time() + 15.0
        while not paused() and time.time() < deadline:
            self.spin(0.1)
        assert paused(), "the mission did not wait for the condition"
        # The waypoint is done, the mission is not
        self.spin(1.0)
        assert not result_future.done(), "the mission did not hold for the condition"

        # Only published once paused, which is when the condition is watched
        for _ in range(5):
            self.pause_publisher.publish(Bool(data=True))
            self.spin(0.3)

        result = self.result_of(handle)
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED

    # ---- Ending early -------------------------------------------------

    def test_cancelling_the_mission_cancels_the_goal_on_the_planner(self):
        """Preemption has to reach the planner, or the robot keeps driving"""
        # Still driving however long the cancel takes to arrive
        slow_at.update({1.0, 2.0, 3.0})
        handle = self.send(self.mission(count=3))
        self.spin(0.5)

        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self.node, cancel, timeout_sec=15.0, executor=self.executor
        )

        deadline = time.time() + 15.0
        while not planner_cancels and time.time() < deadline:
            self.spin(0.2)
        assert planner_cancels, "the planner never saw a cancel request"
        assert len(planner_goals) < 3, "the mission kept starting waypoints"
        # Settled before the next test sends a mission
        result = self.result_of(handle)
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_CANCELED

    def test_a_waypoint_the_planner_refuses_ends_the_mission(self):
        """Driving on to the next waypoint after a failure is not safe"""
        refuse_at.add(2.0)
        result = self.result_of(self.send(self.mission(count=3)))

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_FAILED
        assert list(result.reached_waypoints) == [True, False, False]
        assert result.last_reached_index == 0
        assert 3.0 not in planner_goals, "it carried on after a failed waypoint"

    # ---- Mission status -----------------------------------------------

    def test_the_status_follows_the_mission_under_one_id(self):
        sent_at = time.time()
        result = self.result_of(self.send(self.mission(count=2)))
        self.spin(0.5)

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        ongoing = [status for status in self.statuses if status.mission_id]
        assert ongoing, "no status was published during the mission"
        ids = {status.mission_id for status in ongoing}
        assert len(ids) == 1, f"the mission id changed: {ids}"
        # The id is the time the mission was received, in nanoseconds
        assert sent_at - 1.0 <= int(ids.pop()) / 1e9 <= time.time()
        assert all(status.total_goals == 2 for status in ongoing)
        assert {status.current_goal_idx for status in ongoing} == {0, 1}

        last = self.statuses[-1]
        assert last.mission_id == ""
        assert last.state == MissionStatus.STATE_IDLE
        assert last.message_level == MissionStatus.LEVEL_INFO
        assert "completed" in last.message

    def test_a_refused_mission_is_reported_as_an_error(self):
        self.result_of(self.send(self.mission(count=0)))
        self.spin(0.5)

        last = self.statuses[-1]
        assert last.state == MissionStatus.STATE_IDLE
        assert last.message_level == MissionStatus.LEVEL_ERROR
        assert "at least one" in last.message

    def test_a_mission_sent_while_one_is_ongoing_is_rejected(self):
        """The ongoing mission has to finish or be canceled first"""
        ongoing = self.send(self.mission(count=2))
        self.spin(0.5)

        future = self.client.send_goal_async(self.mission(count=1))
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=15.0, executor=self.executor
        )
        assert not future.result().accepted, "the ongoing mission was replaced"

        result = self.result_of(ongoing)
        self.spin(0.5)
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        assert len({status.mission_id for status in self.statuses if status.mission_id}) == 1
        assert planner_goals == [1.0, 2.0]

    # ---- Construction -------------------------------------------------

    def test_a_planner_that_is_not_an_action_server_is_refused(self):
        """Better than accepting the mission and failing on the first waypoint"""
        planner = StandInPlanner(component_name="timed_planner")
        planner.run_type = ComponentRunType.TIMED

        with pytest.raises(ValueError, match="action server"):
            MissionManager(component_name="refused_mission", planner=planner)

    def test_the_components_are_written_into_the_config(self):
        """The config is all a mission launched in its own process gets"""
        planner = StandInPlanner(component_name="some_planner")
        planner.run_type = "ActionServer"
        mission = MissionManager(
            component_name="some_mission",
            planner=planner,
            controller=Controller(component_name="some_controller"),
            drive_manager=DriveManager(component_name="some_driver"),
        )
        assert mission.config.planner_action == "some_planner/navigate_to_goal"
        assert mission.stop_refs == [
            "some_controller/stop_path_tracking",
            "some_driver/stop_robot",
        ]

    def test_a_mission_with_no_components_needs_them_in_its_config(self):
        with pytest.raises(ValueError, match="controller_name"):
            MissionManager(
                component_name="uncontrolled_mission",
                config=MissionManagerConfig(planner_action="planner/navigate_to_goal"),
            )

        mission = MissionManager(
            component_name="configured_mission",
            config=MissionManagerConfig(
                planner_action="planner/navigate_to_goal",
                controller_name="controller",
                drive_manager_name="drive_manager",
            ),
        )
        assert mission.config.controller_name == "controller"

    def test_a_component_of_the_wrong_kind_is_refused(self):
        with pytest.raises(TypeError, match="Controller"):
            MissionManager(
                component_name="miswired_mission",
                controller=StandInPlanner(component_name="not_a_controller"),
            )

    def test_a_mission_with_no_waypoints_is_refused(self):
        result = self.result_of(self.send(self.mission(count=0)))
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_FAILED

    def test_each_mission_cleans_up_its_routine(self):
        """A routine per goal would otherwise pile up on the Monitor"""
        self.result_of(self.send(self.mission(count=1)))
        self.result_of(self.send(self.mission(count=1)))

        from ros_sugar.core import Monitor

        names = [
            name
            for name in rclpy.create_node(
                "topic_lister", context=self.context
            ).get_topic_names_and_types()
        ]
        mission_cursors = [
            name for name, _ in names if name.startswith("/routine/mission_")
        ]
        assert not mission_cursors, (
            f"missions left their routines behind: {mission_cursors}"
        )
        assert Monitor.RUNTIME_API_SERVICE
