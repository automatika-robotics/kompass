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

import math
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from automatika_ros_sugar.srv import ExecuteMethod
from geometry_msgs.msg import Pose, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

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
#: Where the origin of the odom frame sits in the world frame, so waypoints
#: given in odom land somewhere else once driven
ODOM_OFFSET_X = 10.0

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
        odom_in_world = TransformStamped()
        odom_in_world.header.frame_id = ROBOT_FRAME
        odom_in_world.child_frame_id = "odom"
        odom_in_world.transform.translation.x = ODOM_OFFSET_X
        odom_in_world.transform.rotation.w = 1.0
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)
        cls.tf_broadcaster.sendTransform(odom_in_world)
        cls.lifecycle_client = cls.node.create_client(
            ChangeState, "/mission/change_state"
        )
        cls.mission_methods = cls.node.create_client(
            ExecuteMethod, "/mission/execute_method"
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

    def wait_for(self, predicate, seconds: float) -> bool:
        """Spin until the predicate holds, or the time runs out"""
        deadline = time.time() + seconds
        while not predicate() and time.time() < deadline:
            self.spin(0.1)
        return predicate()

    def call_mission(self, method: str) -> ExecuteMethod.Response:
        """Run one of the mission manager's actions, as a caller would"""
        request = ExecuteMethod.Request()
        request.name = method
        future = self.mission_methods.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=30.0, executor=self.executor
        )
        assert future.done(), f"'{method}' got no answer"
        return future.result()

    def change_state(self, transition_id: int) -> bool:
        """Take the mission manager through one lifecycle transition"""
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=30.0, executor=self.executor
        )
        return future.done() and future.result().success

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
            self.mission(count=2, pause_condition_topic=PAUSE_TOPIC)
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

    def test_the_last_waypoint_does_not_wait_for_the_condition(self):
        """There is nothing to go on to from the last waypoint, so reaching it
        is the end of the mission, with no go-ahead to wait for"""
        result = self.result_of(
            self.send(self.mission(count=1, pause_condition_topic=PAUSE_TOPIC)),
            timeout=15.0,
        )

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        assert not any(
            msg.state == MultiGoalPlanPath.Feedback.STATE_PAUSED_CONDITION
            for msg in self.feedback
        ), "the mission waited at its last waypoint"

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
        self.spin(0.5)
        assert self.statuses[-1].state == MissionStatus.STATE_CANCELED

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

        # The final status, once, and it stays the last one: a late
        # subscriber still learns how the mission ended
        final = [s for s in self.statuses if s.state == MissionStatus.STATE_COMPLETED]
        assert len(final) == 1, f"published {len(final)} final statuses"
        last = self.statuses[-1]
        assert last is final[0]
        assert last.mission_id == ongoing[0].mission_id
        assert last.message_level == MissionStatus.LEVEL_INFO
        assert "completed" in last.message

    def test_a_refused_mission_is_reported_as_an_error(self):
        self.result_of(self.send(self.mission(count=0)))
        self.spin(0.5)

        last = self.statuses[-1]
        assert last.state == MissionStatus.STATE_ABORTED
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

    def test_the_result_says_how_far_from_the_last_waypoint_the_robot_ended(self):
        result = self.result_of(self.send(self.mission(count=1)))

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        # The robot stands at ROBOT_AT facing along x, the waypoint is at (1, 0)
        expected = math.hypot(ROBOT_AT[0] - 1.0, ROBOT_AT[1])
        assert result.end_displacement.lateral_distance_error == pytest.approx(expected)
        assert result.end_displacement.orientation_error == pytest.approx(0.0)

    # ---- Frames -------------------------------------------------------

    def test_waypoints_given_in_another_frame_are_driven_in_the_world_frame(self):
        result = self.result_of(self.send(self.mission(count=1, frame_id="odom")))

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        # Waypoint 1.0 in odom, whose origin is at ODOM_OFFSET_X in the world
        assert planner_goals == [1.0 + ODOM_OFFSET_X]

    def test_waypoints_in_a_frame_with_no_transform_are_refused(self):
        """Driving to the raw coordinates would take the robot somewhere else"""
        result = self.result_of(self.send(self.mission(count=1, frame_id="nowhere")))
        self.spin(0.5)

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_FAILED
        assert not planner_goals, "it drove to a waypoint in an unknown frame"
        assert "nowhere" in self.statuses[-1].message

    # ---- Pausing ------------------------------------------------------

    def test_pausing_stops_the_robot_until_the_mission_is_resumed(self):
        slow_at.update({1.0})
        handle = self.send(self.mission(count=1))
        result_future = handle.get_result_async()
        assert self.wait_for(lambda: 1.0 in planner_goals, 10.0)
        self.robot_commands.clear()

        paused = self.call_mission("pause_mission")
        assert paused.success, paused.error_msg

        # The waypoint's goal is canceled, and the robot told to stop
        assert self.wait_for(lambda: planner_cancels == [1.0], 5.0), (
            "the planner kept driving to the waypoint"
        )
        assert self.wait_for(
            lambda: any(
                cmd.linear.x == 0.0 and cmd.angular.z == 0.0
                for cmd in self.robot_commands
            ),
            10.0,
        ), "the robot was not stopped"
        assert self.wait_for(
            lambda: self.statuses and self.statuses[-1].state == MissionStatus.STATE_PAUSED,
            5.0,
        ), "the status does not say paused"
        assert any(
            msg.state == MultiGoalPlanPath.Feedback.STATE_PAUSED for msg in self.feedback
        )
        # Held: nothing is driven while paused
        self.spin(1.0)
        assert planner_goals == [1.0]
        assert not result_future.done()
        # Under the name a UI is told to follow before any mission starts
        topics = [name for name, _ in self.node.get_topic_names_and_types()]
        assert "/routine/navigation_mission/state" in topics, (
            f"routine topics: {[t for t in topics if t.startswith('/routine')]}"
        )

        slow_at.clear()
        resumed = self.call_mission("resume_mission")
        assert resumed.success, resumed.error_msg
        result = self.result_of(handle)

        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED
        # Resuming drives to the waypoint it was paused on the way to
        assert planner_goals == [1.0, 1.0]

    def test_pausing_with_no_mission_is_refused(self):
        response = self.call_mission("pause_mission")
        assert not response.success
        assert "No ongoing mission" in response.error_msg

    # ---- Lifecycle ----------------------------------------------------

    def test_deactivating_ends_the_mission_and_its_client_gets_the_result(self):
        """Deactivating takes the action server down. Unless the mission ends
        first, its client never gets a result and the routine keeps driving"""
        slow_at.update({1.0})
        handle = self.send(self.mission(count=2))
        # Asked for straight away, like any client that waits on the mission
        result_future = handle.get_result_async()
        assert self.wait_for(lambda: 1.0 in planner_goals, 10.0)

        try:
            assert self.change_state(Transition.TRANSITION_DEACTIVATE)
            rclpy.spin_until_future_complete(
                self.node, result_future, timeout_sec=15.0, executor=self.executor
            )
            assert result_future.done(), "the client never got a result"
            assert result_future.result().status == GoalStatus.STATUS_ABORTED
            # Reaches the planner as the mission ends, not necessarily before
            # its client hears the result
            assert self.wait_for(lambda: planner_cancels == [1.0], 5.0), (
                "the planner goal was left running"
            )
            # Published while the publisher still existed
            assert any(
                status.state == MissionStatus.STATE_ABORTED
                and "deactivated" in status.message
                for status in self.statuses
            ), "the mission's final status was never published"
        finally:
            assert self.change_state(Transition.TRANSITION_ACTIVATE)
            assert self.client.wait_for_server(timeout_sec=15.0)

        # And missions run again once it is back
        result = self.result_of(self.send(self.mission(count=1)))
        assert result.outcome == MultiGoalPlanPath.Result.OUTCOME_COMPLETED

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
        # The same for every mission, so a UI can be told to follow it up front
        assert mission.routine_name == "navigation_mission"
        assert mission.stop_refs == [
            "some_controller/stop_path_tracking",
            "some_driver/stop_robot",
        ]

    def test_the_routine_name_can_be_configured(self):
        planner = StandInPlanner(component_name="named_planner")
        planner.run_type = "ActionServer"
        mission = MissionManager(
            component_name="named_mission",
            planner=planner,
            controller=Controller(component_name="named_controller"),
            drive_manager=DriveManager(component_name="named_driver"),
            config=MissionManagerConfig(routine_name="patrol"),
        )
        assert mission.routine_name == "patrol"

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
            name
            for name, _ in names
            if name.startswith("/routine/navigation_mission")
        ]
        assert not mission_cursors, (
            f"missions left their routines behind: {mission_cursors}"
        )
        assert Monitor.RUNTIME_API_SERVICE
