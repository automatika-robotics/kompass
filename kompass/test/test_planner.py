"""Correctness unit tests for the Planner component.

Tests the helpers of ``kompass.components.planner.Planner`` without
instantiating a ROS node, using the same ``object.__new__`` + MagicMock
pattern as ``test_vision_tracking.py``.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock, PropertyMock, patch

import numpy as np
from kompass_core.models import RobotState
from rclpy import qos

from builtin_interfaces.msg import Time

from kompass.components.defaults import TopicsKeys
from kompass.components.planner import Planner
from kompass.components.ros import Topic
from kompass.config import ComponentRunType
from kompass_interfaces.action import PlanPath as PlanPathAction
from kompass_interfaces.msg import PathTrackingError
from ros_sugar.config import QoSConfig
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
    p._map_lock = threading.Lock()
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
# Planning map  (set on the OMPL planner once when received, not on every plan)
# ---------------------------------------------------------------------------

MAP_DATA = {
    "resolution": 0.05,
    "width": 10,
    "height": 10,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "origin_yaw": 0.0,
}


def make_map_callback() -> MagicMock:
    """A map topic callback, read by the planner for the map metadata only"""
    callback = MagicMock()
    callback.get_output.return_value = MAP_DATA
    return callback


class TestPlanningMap:
    def test_received_map_is_set_on_the_ompl_planner(self):
        p = make_planner_stub(get_callback=MagicMock(return_value=make_map_callback()))
        map_3d = np.zeros((4, 3), dtype=np.float32)

        p._set_planning_map(output=map_3d, msg=MagicMock(), topic=MagicMock())

        p.ompl_planner.set_map.assert_called_once_with(map_3d)
        assert p.map is map_3d
        assert p.map_data == MAP_DATA

    def test_map_received_before_the_ompl_planner_is_kept(self):
        p = make_planner_stub(get_callback=MagicMock(return_value=make_map_callback()))
        del p.ompl_planner
        map_3d = np.zeros((4, 3), dtype=np.float32)

        p._set_planning_map(output=map_3d)

        assert p.map is map_3d
        assert p.map_data == MAP_DATA

    def test_kept_map_is_set_when_the_ompl_planner_is_created(self):
        p = make_planner_stub(get_callback=MagicMock(return_value=make_map_callback()))
        del p.ompl_planner
        map_3d = np.zeros((4, 3), dtype=np.float32)
        p._set_planning_map(output=map_3d)

        p._attach_callbacks = MagicMock()
        p._attach_map_callback = MagicMock()
        with patch("kompass.components.planner.OMPLGeometric") as ompl_class:
            with patch("kompass.components.planner.Robot"):
                with patch.object(Planner, "robot", new_callable=PropertyMock):
                    with patch.object(
                        Planner, "robot_geometry_type", new_callable=PropertyMock
                    ):
                        p.init_variables()

        ompl_class.return_value.set_map.assert_called_once_with(map_3d)
        assert p.map_data == MAP_DATA

    def test_missing_map_output_is_ignored(self):
        p = make_planner_stub(get_callback=MagicMock(return_value=make_map_callback()))

        p._set_planning_map(output=None)

        p.ompl_planner.set_map.assert_not_called()
        assert p.map_data is None

    def test_map_callback_sets_the_planning_map(self):
        callback = make_map_callback()
        p = make_planner_stub(get_callback=MagicMock(return_value=callback))

        p._attach_map_callback()

        callback.on_callback_execute.assert_called_once_with(p._set_planning_map)

    def test_planning_does_not_set_the_map_again(self):
        p = make_planner_stub()
        p.map_data = MAP_DATA

        p._plan(start=RobotState(x=0.0, y=0.0), goal=RobotState(x=1.0, y=1.0))

        p.ompl_planner.setup_problem.assert_called_once()
        args, kwargs = p.ompl_planner.setup_problem.call_args
        # Map metadata, start and goal only: no map to rebuild the collision map from
        assert len(args) == 7 and "map_3d" not in kwargs
        p.ompl_planner.set_map.assert_not_called()
        assert not p._map_lock.locked()

    def test_no_planning_before_a_map_is_received(self):
        p = make_planner_stub()

        result = p._plan(start=RobotState(x=0.0, y=0.0), goal=RobotState(x=1.0, y=1.0))

        assert result is False
        p.ompl_planner.setup_problem.assert_not_called()
        p.health_status.set_fail_system.assert_called_once()

    def test_updating_the_state_does_not_read_the_map(self):
        p = make_planner_stub(get_callback=MagicMock(return_value=MagicMock()))
        p._inputs_keys = [TopicsKeys.ROBOT_LOCATION]

        with patch.object(
            Planner, "odom_tf_listener", new_callable=PropertyMock, return_value=None
        ):
            p._update_state()

        read_keys = [call.args[0] for call in p.get_callback.call_args_list]
        assert TopicsKeys.GLOBAL_MAP not in read_keys


class TestMapInputQoS:
    """The map is published once: the planner has to get it even when joining later"""

    @staticmethod
    def _assert_latched(topic: Topic):
        assert topic.qos_profile.durability == qos.DurabilityPolicy.TRANSIENT_LOCAL
        assert topic.qos_profile.reliability == qos.ReliabilityPolicy.RELIABLE

    def test_default_map_input_is_latched(self):
        planner = Planner(component_name="planner_default_map_qos_test")

        self._assert_latched(planner.get_in_topic(TopicsKeys.GLOBAL_MAP))

    def test_given_map_input_is_latched(self):
        shared_qos = QoSConfig(reliability=qos.ReliabilityPolicy.BEST_EFFORT)
        map_topic = Topic(
            name="/my_map", msg_type="OccupancyGrid", qos_profile=shared_qos
        )

        planner = Planner(
            component_name="planner_map_qos_test", inputs={"map": map_topic}
        )

        self._assert_latched(planner.get_in_topic(TopicsKeys.GLOBAL_MAP))
        # A QoS profile shared with other topics is left unchanged
        assert shared_qos.durability == qos.DurabilityPolicy.VOLATILE
        assert shared_qos.reliability == qos.ReliabilityPolicy.BEST_EFFORT

    def test_map_input_set_after_init_is_latched(self):
        planner = Planner(component_name="planner_map_qos_after_init_test")

        map_topic = Topic(name="/my_map", msg_type="OccupancyGrid")
        planner.inputs(map=map_topic)

        assert planner.get_in_topic(TopicsKeys.GLOBAL_MAP).name == map_topic.name
        self._assert_latched(planner.get_in_topic(TopicsKeys.GLOBAL_MAP))


# ---------------------------------------------------------------------------
# trigger_main_action_server  (component action (bool, str) contract)
# ---------------------------------------------------------------------------

class TestTriggerMainActionServer:
    @staticmethod
    def _call(p, **kwargs):
        # The undecorated method: the decorator only runs it with rclpy initialized
        return Planner.trigger_main_action_server.__wrapped__(
            p, goal_x=1.0, goal_y=2.0, **kwargs
        )

    @staticmethod
    def _make_planner(run_type=ComponentRunType.ACTION_SERVER) -> Planner:
        p = make_planner_stub()
        p.config._run_type = run_type
        p.node_name = "planner"
        p.main_action_name = "navigate_to_goal"
        p.action_type = PlanPathAction
        return p

    @staticmethod
    def _assert_contract(result):
        assert isinstance(result, tuple) and len(result) == 2
        assert isinstance(result[0], bool) and isinstance(result[1], str)

    def test_accepted_goal_succeeds(self):
        p = self._make_planner()
        with patch("kompass.components.planner.ActionClientHandler") as client_class:
            client_class.return_value.send_request.return_value = True
            result = self._call(p, goal_orientation=np.pi / 2, tolerance_dist=0.3)

        self._assert_contract(result)
        assert result[0] is True
        goal = client_class.return_value.send_request.call_args.args[0]
        assert (goal.goal.position.x, goal.goal.position.y) == (1.0, 2.0)
        assert goal.end_tolerance.lateral_distance_error == 0.3

    def test_goal_not_accepted_fails(self):
        p = self._make_planner()
        with patch("kompass.components.planner.ActionClientHandler") as client_class:
            client_class.return_value.send_request.return_value = False
            result = self._call(p)

        self._assert_contract(result)
        assert result[0] is False

    def test_planner_not_running_as_action_server_fails(self):
        p = self._make_planner(run_type=ComponentRunType.EVENT)
        with patch("kompass.components.planner.ActionClientHandler") as client_class:
            result = self._call(p)

        self._assert_contract(result)
        assert result[0] is False
        client_class.assert_not_called()

    def test_client_error_fails(self):
        p = self._make_planner()
        with patch(
            "kompass.components.planner.ActionClientHandler",
            side_effect=RuntimeError("no client"),
        ):
            result = self._call(p)

        self._assert_contract(result)
        assert result[0] is False
        assert "no client" in result[1]
        p.health_status.set_fail_component.assert_called_once()


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
