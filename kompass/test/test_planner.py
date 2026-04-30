"""Correctness unit tests for the Planner component.

Tests the helpers of ``kompass.components.planner.Planner`` without
instantiating a ROS node, using the same ``object.__new__`` + MagicMock
pattern as ``test_vision_tracking.py``.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
from kompass_core.models import RobotState

from builtin_interfaces.msg import Time

from kompass.components.planner import Planner
from kompass.components.ros import Topic
from kompass_interfaces.msg import PathTrackingError
from ros_sugar.io.publisher import Publisher


def make_real_publisher(topic_name: str, msg_type: str) -> Publisher:
    """Construct a real ros_sugar Publisher with the underlying rclpy
    publisher replaced by a MagicMock.

    This exercises the real ``msg_type.convert`` + header/frame_id attachment
    that happens inside ``Publisher.publish``, without needing rclpy.init()
    or a live node. Tests can inspect the sent message via
    ``pub._publisher.publish.call_args``.
    """
    topic = Topic(name=topic_name, msg_type=msg_type)
    # When multiple packages register converters for the same type, msg_type.convert
    # is a list; the real pipeline flattens it in _select_output_converters.
    if isinstance(topic.msg_type.convert, list):
        topic.msg_type.convert = topic.msg_type.convert[0]
    pub = Publisher(output_topic=topic, node_name="test_planner")
    pub._publisher = MagicMock()
    return pub


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mangle(name: str) -> str:
    return f"_Planner__{name}"


def make_planner_stub(**overrides) -> Planner:
    p = object.__new__(Planner)

    # Config
    p.config = MagicMock()
    p.config.frames = MagicMock()
    p.config.frames.world = "map"
    p.config.frames.odom = "odom"
    p.config.distance_tolerance = 0.1
    p.config.loop_rate = 1000.0
    p.config.topic_subscription_timeout = 0.01

    # State
    p.robot_state = None
    p.goal = {}
    p.map = None
    p.map_data = None
    p.path = None
    p.ros_path = None
    p.reached_end = False
    p._recording_on = False
    p._recorded_motion = None
    p._last_path_cost = float("inf")
    p._main_goal_lock = threading.Lock()
    p._config_file = None

    # ROS infra fakes
    p.get_callback = MagicMock(return_value=None)
    p.get_publisher = MagicMock()
    p.get_logger = MagicMock()
    p.get_ros_time = MagicMock(return_value=Time(sec=0, nanosec=0))
    p.health_status = MagicMock()
    p.in_topic_name = MagicMock(side_effect=lambda key: f"/{str(key).lower()}")
    p.got_all_inputs = MagicMock(return_value=True)
    p.get_missing_inputs = MagicMock(return_value=[])

    # OMPL planner mock
    ompl = MagicMock()
    ompl.planner_id = "TRRT"
    ompl.path_cost = 1.0
    ompl.setup_problem = MagicMock()
    ompl.solve = MagicMock(return_value=None)  # default: no path
    p.ompl_planner = ompl

    # Mangled name for the private __robot_state_to_pose_stamped helper
    # (defined with __name so __class__.__name prefixing applies)

    for k, v in overrides.items():
        setattr(p, k, v)
    return p


# ---------------------------------------------------------------------------
# reached_point  (A11: no per-tick publish; purity)
# ---------------------------------------------------------------------------

class TestReachedPoint:
    def _call(self, p, goal_xy, tol=0.1):
        goal = RobotState(x=goal_xy[0], y=goal_xy[1])
        return p.reached_point(goal, PathTrackingError(lateral_distance_error=tol))

    def test_true_when_within_tolerance(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.05, yaw=0.0)
        # RobotState.distance returns numpy scalar -> bool(<=) not Python True
        assert bool(self._call(p, (0.0, 0.0), tol=0.1)) is True

    def test_false_when_outside_tolerance(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=1.0, y=0.0, yaw=0.0)
        assert bool(self._call(p, (0.0, 0.0), tol=0.1)) is False

    def test_false_when_robot_state_missing(self):
        p = make_planner_stub()
        p.robot_state = None
        assert self._call(p, (0.0, 0.0)) is False

    def test_does_not_publish_on_any_outcome(self):
        """A11 regression: reached_point is pure; no side-effect publish."""
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.05, yaw=0.0)
        self._call(p, (0.0, 0.0), tol=0.1)
        self._call(p, (5.0, 0.0), tol=0.1)
        p.get_publisher.assert_not_called()


# ---------------------------------------------------------------------------
# __robot_state_to_pose_stamped  (quaternion correctness + header)
# ---------------------------------------------------------------------------

class TestRobotStateToPoseStamped:
    def _call(self, p):
        return getattr(p, _mangle("robot_state_to_pose_stamped"))()

    def test_identity_quaternion_for_zero_yaw(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=1.0, y=2.0, yaw=0.0)
        pose = self._call(p)
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 2.0
        assert pose.pose.orientation.z == 0.0
        assert pose.pose.orientation.w == 1.0

    def test_half_pi_yaw(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.0, yaw=np.pi / 2)
        pose = self._call(p)
        assert pose.pose.orientation.z == float(np.sin(np.pi / 4))
        assert pose.pose.orientation.w == float(np.cos(np.pi / 4))

    def test_header_frame_and_stamp_populated(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.0, yaw=0.0)
        pose = self._call(p)
        assert pose.header.frame_id == p.config.frames.world
        # stamp must be whatever get_ros_time returned, not the default epoch
        p.get_ros_time.assert_called()


# ---------------------------------------------------------------------------
# _plan_on_goal  (B1 fix: callback-based conversion)
# ---------------------------------------------------------------------------

class TestPlanOnGoal:
    def test_pointstamped_fallback_without_callback(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.0)
        p._clear_path = MagicMock()
        p._update_state = MagicMock()
        p._plan_on_goal_core = MagicMock(return_value=True)

        msg = SimpleNamespace(point=SimpleNamespace(x=3.0, y=4.0))
        p._plan_on_goal(msg)

        p._plan_on_goal_core.assert_called_once()
        goal_arg, kwargs = p._plan_on_goal_core.call_args
        assert goal_arg[0].x == 3.0 and goal_arg[0].y == 4.0
        assert kwargs.get("goal_index") == 0

    def test_uses_callback_get_output_when_provided(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.0)
        p._clear_path = MagicMock()
        p._update_state = MagicMock()
        p._plan_on_goal_core = MagicMock(return_value=True)

        cb = MagicMock()
        cb.get_output.return_value = RobotState(x=7.0, y=8.0)

        p._plan_on_goal(MagicMock(), callback=cb, goal_index=2)

        cb.get_output.assert_called_once_with(
            to_robot_state=True, robot_state=p.robot_state
        )
        p._plan_on_goal_core.assert_called_once()
        _args, kwargs = p._plan_on_goal_core.call_args
        assert kwargs.get("goal_index") == 2

    def test_bails_gracefully_when_callback_returns_no_state(self):
        p = make_planner_stub()
        p.robot_state = RobotState(x=0.0, y=0.0)
        p._clear_path = MagicMock()
        p._update_state = MagicMock()
        p._plan_on_goal_core = MagicMock()

        cb = MagicMock()
        cb.get_output.return_value = None

        p._plan_on_goal(MagicMock(), callback=cb)

        p._plan_on_goal_core.assert_not_called()
        p.get_logger.return_value.error.assert_called()


# ---------------------------------------------------------------------------
# _plan_on_goal_core  (reached_end edge + publish)
# ---------------------------------------------------------------------------

class TestPlanOnGoalCore:
    def test_returns_false_when_robot_location_missing(self):
        p = make_planner_stub()
        p.got_all_inputs = MagicMock(return_value=False)

        result = p._plan_on_goal_core(RobotState(x=1.0, y=0.0))

        assert result is False

    def test_publishes_reached_end_and_empty_path_when_reached(self):
        p = make_planner_stub()
        p.reached_end = True
        pub = MagicMock()
        p.get_publisher = MagicMock(return_value=pub)

        assert p._plan_on_goal_core(RobotState(x=1.0, y=0.0), goal_index=3) is True
        # After reaching: goal slot cleared, path cleared
        assert p.goal.get(3) is None
        # Publisher was called at least twice: reached_end True + empty path
        assert pub.publish.call_count >= 2


# ---------------------------------------------------------------------------
# _plan  (A4 regression: ros_path header populated)
# ---------------------------------------------------------------------------

class TestPlanHeader:
    def test_published_path_has_frame_and_stamp(self):
        p = make_planner_stub()
        # Provide a "solved" OMPL path with a single state
        state = MagicMock()
        state.getX.return_value = 0.0
        state.getY.return_value = 0.0
        state.getYaw.return_value = 0.0
        solution = MagicMock()
        solution.getStates.return_value = [state]
        p.ompl_planner.solve.return_value = solution
        p.ompl_planner.path_cost = 0.5
        p.map_data = {"origin_x": 0.0, "origin_y": 0.0, "width": 10, "height": 10, "resolution": 0.05}
        p.map = np.zeros((10, 10), dtype=np.int8)

        # Use a real Publisher so the frame_id is attached by the actual
        # convert + header-injection path inside Publisher.publish().
        pub = make_real_publisher("/plan", "Path")
        p.get_publisher = MagicMock(return_value=pub)

        ok = p._plan(
            start=RobotState(x=0.0, y=0.0),
            goal=RobotState(x=1.0, y=1.0),
            publish_path=True,
        )
        assert ok is True

        # Inspect the message handed to the underlying rclpy publisher
        pub._publisher.publish.assert_called_once()
        sent_msg = pub._publisher.publish.call_args[0][0]
        assert sent_msg.header.frame_id == p.config.frames.world
        # Stamp is set by Publisher.publish (non-zero time)
        assert (sent_msg.header.stamp.sec, sent_msg.header.stamp.nanosec) != (0, 0)
        # ros_path reference matches the sent msg (identity convert for Path)
        assert p.ros_path is sent_msg


# ---------------------------------------------------------------------------
# _save_plan_to_file_srv_callback  (recording flag cleanup)
# ---------------------------------------------------------------------------

class TestSavePlanCallback:
    def test_recording_flag_cleared_after_save(self, tmp_path):
        p = make_planner_stub()
        p._recording_on = True

        from nav_msgs.msg import Path as RosPath
        from geometry_msgs.msg import PoseStamped
        recorded = RosPath()
        recorded.header.frame_id = "map"
        pose = PoseStamped()
        pose.pose.position.x = 1.0
        recorded.poses.append(pose)
        p._recorded_motion = recorded

        from kompass_interfaces.srv import PathFromToFile
        req = PathFromToFile.Request()
        req.file_location = str(tmp_path)
        req.file_name = "path.json"
        resp = PathFromToFile.Response()

        p._save_plan_to_file_srv_callback(req, resp)

        assert p._recording_on is False
        assert p._recorded_motion is None
        assert resp.path_num_points == 1
