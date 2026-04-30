"""Correctness unit tests for the path-following controller.

These tests exercise the path-control helpers of
``kompass.components.controller.Controller`` without instantiating a ROS node.
The controller is built via ``object.__new__`` and only the attributes each
helper needs are attached — no rclpy runtime, no TF, no publishers, no
simulation required.

They pin down the ``PathControlStatus`` contract introduced to split the
old ``_path_control``'s tangled side-effects from its status reporting.
"""

from __future__ import annotations

import threading
from queue import Queue
from types import SimpleNamespace
from unittest.mock import MagicMock

from kompass_core.datatypes import LaserScanData
from kompass_core.models import RobotState

from kompass.components.controller import (
    CmdPublishType,
    Controller,
    ControllerMode,
    PathControlStatus,
)
from kompass.components.defaults import TopicsKeys


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mangle(name: str) -> str:
    """Return the attribute name for a Controller private method.
    """
    return f"_{name}"


def _set_mangled(c: Controller, name: str, value) -> None:
    setattr(c, _mangle(name), value)


def _get_mangled(c: Controller, name: str):
    return getattr(c, _mangle(name))


def make_path_controller_stub(**overrides) -> Controller:
    """Bare Controller wired for path-follower unit tests."""
    c = object.__new__(Controller)

    # Threading / queues
    c._main_goal_lock = threading.Lock()
    c._cmds_queue = Queue()

    # Tracking state
    _set_mangled(c, "reached_end", False)
    _set_mangled(c, "lat_dist_error", 0.0)
    _set_mangled(c, "ori_error", 0.0)
    _set_mangled(c, "goal_point", RobotState(x=10.0, y=0.0))

    c.robot_state = RobotState(x=0.0, y=0.0, yaw=0.0)
    c.plan = None
    c.sensor_data = None
    c.local_map = None
    c.local_map_resolution = None

    # Config
    c.config = MagicMock()
    c.config._mode = ControllerMode.PATH_FOLLOWER
    c.config.use_direct_sensor = False
    c.config.ctrl_publish_type = CmdPublishType.TWIST_ARRAY
    c.config.topic_subscription_timeout = 0.01
    c.config.loop_rate = 1000.0
    c.config.debug = False
    c.config.prediction_horizon = 1.0
    # The algorithm setter on the real config is a converter; stub the enum directly
    from kompass_core.control import ControllersID
    c.config.algorithm = ControllersID.DWA

    # Fake path controller; real one lives in kompass-core
    path_controller = MagicMock()
    path_controller.path = True
    path_controller._config = MagicMock()
    path_controller._config.goal_dist_tolerance = 0.1
    path_controller.linear_x_control = [0.1, 0.1]
    path_controller.linear_y_control = [0.0, 0.0]
    path_controller.angular_control = [0.0, 0.0]
    path_controller.distance_error = 0.05
    path_controller.orientation_error = 0.01
    path_controller.logging_info = MagicMock(return_value="ok")
    # loop_step default: cmd found
    path_controller.loop_step = MagicMock(return_value=True)
    _set_mangled(c, "path_controller", path_controller)

    # Health + IO fakes
    c.health_status = MagicMock()
    c.get_callback = MagicMock(return_value=None)
    c.get_publisher = MagicMock()
    c.get_logger = MagicMock()

    # _update_state / _update_sensor_data / _publish / _stop_robot are side-effect surfaces; record calls
    c._update_state = MagicMock()
    c._update_sensor_data = MagicMock()
    c._publish = MagicMock()
    c._stop_robot_calls = []
    c._stop_robot = lambda: c._stop_robot_calls.append(True)

    # reached_point proxy — tests override easily via c.reached_point = ...
    c.reached_point = MagicMock(return_value=False)

    for k, v in overrides.items():
        setattr(c, k, v)
    return c


def make_goal_handle(is_cancel_requested: bool = False, is_active: bool = True):
    gh = SimpleNamespace()
    gh.is_cancel_requested = is_cancel_requested
    gh.is_active = is_active
    gh.canceled = MagicMock()
    gh.abort = MagicMock()
    gh.succeed = MagicMock()
    gh.publish_feedback = MagicMock()
    gh.request = SimpleNamespace(algorithm_name="")
    return gh


# ---------------------------------------------------------------------------
# _path_control
# ---------------------------------------------------------------------------

class TestPathControlStatus:
    def test_returns_idle_when_no_path_controller(self):
        c = make_path_controller_stub()
        _set_mangled(c, "path_controller", None)

        assert c._path_control() == PathControlStatus.IDLE

    def test_returns_idle_when_controller_has_no_path(self):
        c = make_path_controller_stub()
        _get_mangled(c, "path_controller").path = False

        assert c._path_control() == PathControlStatus.IDLE

    def test_returns_waiting_when_robot_state_never_arrives(self):
        c = make_path_controller_stub()
        c.robot_state = None
        # _update_state is a no-op mock, so state stays None through the bounded wait

        assert c._path_control() == PathControlStatus.WAITING_INPUTS

    def test_returns_goal_reached_and_stops_robot(self):
        c = make_path_controller_stub()
        c.reached_point = MagicMock(return_value=True)

        assert c._path_control() == PathControlStatus.GOAL_REACHED
        assert c._stop_robot_calls == [True]
        # loop_step must not run when the goal is already reached
        _get_mangled(c, "path_controller").loop_step.assert_not_called()

    def test_returns_failed_when_loop_step_reports_no_cmd(self):
        c = make_path_controller_stub()
        _get_mangled(c, "path_controller").loop_step = MagicMock(return_value=False)

        assert c._path_control() == PathControlStatus.FAILED
        c._publish.assert_not_called()
        # FAILED status is reported; _path_control itself doesn't flip health
        c.health_status.set_fail_algorithm.assert_not_called()

    def test_returns_running_on_happy_path(self):
        c = make_path_controller_stub()

        assert c._path_control() == PathControlStatus.RUNNING
        c._publish.assert_called_once_with([0.1, 0.1], [0.0, 0.0], [0.0, 0.0])
        c.health_status.set_healthy.assert_called_once()
        assert _get_mangled(c, "lat_dist_error") == 0.05
        assert _get_mangled(c, "ori_error") == 0.01

    def test_does_not_mutate_reached_end(self):
        """Key invariant of the refactor: _path_control is pure w.r.t. __reached_end."""
        for outcome in (
            ("idle_no_controller", lambda c: _set_mangled(c, "path_controller", None)),
            ("idle_no_path", lambda c: setattr(_get_mangled(c, "path_controller"), "path", False)),
            ("goal_reached", lambda c: setattr(c, "reached_point", MagicMock(return_value=True))),
            ("failed", lambda c: setattr(_get_mangled(c, "path_controller"), "loop_step", MagicMock(return_value=False))),
            ("running", lambda c: None),
        ):
            label, setup = outcome
            c = make_path_controller_stub()
            _set_mangled(c, "reached_end", False)
            setup(c)

            c._path_control()

            assert _get_mangled(c, "reached_end") is False, (
                f"_path_control mutated __reached_end on outcome '{label}'"
            )

    def test_direct_sensor_routes_laser_scan(self):
        c = make_path_controller_stub()
        c.config.use_direct_sensor = True
        scan = MagicMock(spec=LaserScanData)
        c.sensor_data = scan

        c._path_control()

        kwargs = _get_mangled(c, "path_controller").loop_step.call_args.kwargs
        assert kwargs["laser_scan"] is scan
        assert kwargs["point_cloud"] is None
        assert kwargs["local_map"] is None

    def test_non_direct_sensor_routes_local_map(self):
        c = make_path_controller_stub()
        c.config.use_direct_sensor = False
        c.local_map = object()
        c.local_map_resolution = 0.05

        c._path_control()

        kwargs = _get_mangled(c, "path_controller").loop_step.call_args.kwargs
        assert kwargs["local_map"] is c.local_map
        assert kwargs["local_map_resolution"] == 0.05
        assert kwargs["laser_scan"] is None
        assert kwargs["point_cloud"] is None


# ---------------------------------------------------------------------------
# _execution_step  (TIMED run type dispatcher)
# ---------------------------------------------------------------------------

class TestExecutionStepDispatch:
    def test_noop_in_vision_mode(self):
        c = make_path_controller_stub()
        c.config._mode = ControllerMode.VISION_FOLLOWER
        c._path_control = MagicMock(return_value=PathControlStatus.RUNNING)

        c._execution_step()

        c._path_control.assert_not_called()

    def test_skips_when_already_reached_end(self):
        c = make_path_controller_stub()
        _set_mangled(c, "reached_end", True)
        c._path_control = MagicMock(return_value=PathControlStatus.RUNNING)

        c._execution_step()

        c._path_control.assert_not_called()

    def test_goal_reached_flips_reached_end_and_clears_callback(self):
        c = make_path_controller_stub()
        plan_cb = MagicMock()
        c.get_callback = MagicMock(side_effect=lambda key: plan_cb if key == TopicsKeys.GLOBAL_PLAN else None)
        c._path_control = MagicMock(return_value=PathControlStatus.GOAL_REACHED)

        c._execution_step()

        assert _get_mangled(c, "reached_end") is True
        plan_cb.clear_last_msg.assert_called_once()
        c.health_status.set_fail_algorithm.assert_not_called()

    def test_failed_flips_fail_algorithm(self):
        c = make_path_controller_stub()
        c._path_control = MagicMock(return_value=PathControlStatus.FAILED)

        c._execution_step()

        assert _get_mangled(c, "reached_end") is False
        c.health_status.set_fail_algorithm.assert_called_once()

    def test_running_does_not_touch_reached_end_or_health(self):
        c = make_path_controller_stub()
        c._path_control = MagicMock(return_value=PathControlStatus.RUNNING)

        c._execution_step()

        assert _get_mangled(c, "reached_end") is False
        c.health_status.set_fail_algorithm.assert_not_called()

    def test_waiting_inputs_is_a_noop(self):
        c = make_path_controller_stub()
        c._path_control = MagicMock(return_value=PathControlStatus.WAITING_INPUTS)

        c._execution_step()

        assert _get_mangled(c, "reached_end") is False
        c.health_status.set_fail_algorithm.assert_not_called()

    def test_exception_inside_path_control_is_logged_and_fails_health(self):
        c = make_path_controller_stub()
        c._path_control = MagicMock(side_effect=RuntimeError("boom"))

        c._execution_step()

        c.get_logger.return_value.error.assert_called_once()
        c.health_status.set_fail_algorithm.assert_called_once()


# ---------------------------------------------------------------------------
# _path_tracking_callback  (ACTION_SERVER dispatcher)
# ---------------------------------------------------------------------------

class TestPathTrackingCallbackDispatch:
    """
    The dispatcher loops on _path_control statuses. Tests mock _path_control
    with a `side_effect` list so we can shape the status sequence and assert
    terminal behavior.
    """

    def _prep(self, c):
        # Inputs check passes; mode assignment is a no-op on MagicMock config;
        # set_path is on the path controller mock.
        c.callbacks_inputs_check = MagicMock(return_value=True)
        # __vision_mode_inputs reaches into self._inputs_keys — stub it out.
        _set_mangled(c, "vision_mode_inputs", lambda: [])
        return make_goal_handle()

    def test_succeeds_when_path_control_reaches_goal(self):
        c = make_path_controller_stub()
        gh = self._prep(c)
        c._path_control = MagicMock(
            side_effect=[
                PathControlStatus.RUNNING,
                PathControlStatus.RUNNING,
                PathControlStatus.GOAL_REACHED,
            ]
        )

        c._path_tracking_callback(gh)

        gh.succeed.assert_called_once()
        gh.abort.assert_not_called()
        # Two RUNNING iterations → two feedback publishes
        assert gh.publish_feedback.call_count == 2
        assert _get_mangled(c, "reached_end") is True

    def test_aborts_on_failed(self):
        c = make_path_controller_stub()
        gh = self._prep(c)
        c._path_control = MagicMock(
            side_effect=[PathControlStatus.RUNNING, PathControlStatus.FAILED]
        )

        c._path_tracking_callback(gh)

        gh.abort.assert_called_once()
        gh.succeed.assert_not_called()
        assert _get_mangled(c, "reached_end") is False

    def test_aborts_on_idle(self):
        c = make_path_controller_stub()
        gh = self._prep(c)
        c._path_control = MagicMock(return_value=PathControlStatus.IDLE)

        c._path_tracking_callback(gh)

        gh.abort.assert_called_once()
        gh.succeed.assert_not_called()

    def test_waiting_inputs_keeps_looping_until_success(self):
        c = make_path_controller_stub()
        gh = self._prep(c)
        c._path_control = MagicMock(
            side_effect=[
                PathControlStatus.WAITING_INPUTS,
                PathControlStatus.WAITING_INPUTS,
                PathControlStatus.GOAL_REACHED,
            ]
        )

        c._path_tracking_callback(gh)

        gh.succeed.assert_called_once()
        assert c._path_control.call_count == 3

    def test_feedback_control_list_has_correct_field_mapping(self):
        """A1 regression: linear_y / angular must come from their own getters,
        not copies of linear_x_control."""
        c = make_path_controller_stub()
        gh = self._prep(c)
        path_ctrl = _get_mangled(c, "path_controller")
        path_ctrl.linear_x_control = [1.1, 1.2]
        path_ctrl.linear_y_control = [0.5, 0.6]
        path_ctrl.angular_control = [0.01, 0.02]

        c._path_control = MagicMock(
            side_effect=[PathControlStatus.RUNNING, PathControlStatus.GOAL_REACHED]
        )

        c._path_tracking_callback(gh)

        feedback = gh.publish_feedback.call_args.args[0]
        assert list(feedback.control_list.linear_velocities.x) == [1.1, 1.2]
        assert list(feedback.control_list.linear_velocities.y) == [0.5, 0.6]
        assert list(feedback.control_list.angular_velocities.z) == [0.01, 0.02]

    def test_feedback_deviation_has_correct_field_mapping(self):
        """A2 regression: lateral_distance_error and orientation_error not swapped."""
        c = make_path_controller_stub()
        gh = self._prep(c)
        _set_mangled(c, "lat_dist_error", 0.42)
        _set_mangled(c, "ori_error", 0.07)
        c._path_control = MagicMock(
            side_effect=[PathControlStatus.RUNNING, PathControlStatus.GOAL_REACHED]
        )

        c._path_tracking_callback(gh)

        fb = gh.publish_feedback.call_args.args[0]
        assert fb.global_path_deviation.lateral_distance_error == 0.42
        assert fb.global_path_deviation.orientation_error == 0.07


# ---------------------------------------------------------------------------
# _set_path_to_controller
# ---------------------------------------------------------------------------

class TestSetPathToController:
    def _make_path_msg(self, xy_pairs):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        p = Path()
        for x, y in xy_pairs:
            ps = PoseStamped()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            p.poses.append(ps)
        return p

    def test_multi_pose_path_populates_goal_and_resets_reached_end(self):
        c = make_path_controller_stub()
        _set_mangled(c, "reached_end", True)  # pretend previous goal completed
        path_ctrl = _get_mangled(c, "path_controller")

        msg = self._make_path_msg([(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)])
        c._set_path_to_controller(msg)

        assert _get_mangled(c, "reached_end") is False
        path_ctrl.set_path.assert_called_once_with(global_path=msg)
        goal = _get_mangled(c, "goal_point")
        assert goal.x == 2.0 and goal.y == 1.0

    def test_single_pose_path_clears_goal_point(self):
        """A10-adjacent: a one-pose path is treated as 'no goal'."""
        c = make_path_controller_stub()
        msg = self._make_path_msg([(5.0, 5.0)])

        c._set_path_to_controller(msg)

        assert _get_mangled(c, "goal_point") is None
        # And kompass-core set_path is still called — it's responsible for
        # clearing its own internal path for < 2 poses.
        _get_mangled(c, "path_controller").set_path.assert_called_once_with(
            global_path=msg
        )


# ---------------------------------------------------------------------------
# reached_point
# ---------------------------------------------------------------------------

class TestReachedPoint:
    # The stub mocks c.reached_point for the _path_control tests; call the real
    # method via the class descriptor to bypass the instance-level mock.
    @staticmethod
    def _real(c, goal):
        return Controller.reached_point(c, goal)

    def test_returns_true_when_goal_is_none(self):
        c = make_path_controller_stub()
        assert self._real(c, None) is True

    def test_returns_false_when_robot_state_missing(self):
        c = make_path_controller_stub()
        c.robot_state = None
        assert self._real(c, RobotState(x=0.0, y=0.0)) is False

    def test_returns_false_when_no_path_controller(self):
        c = make_path_controller_stub()
        _set_mangled(c, "path_controller", None)
        assert self._real(c, RobotState(x=0.0, y=0.0)) is False

    def test_true_within_tolerance(self):
        c = make_path_controller_stub()
        c.robot_state = RobotState(x=0.05, y=0.0, yaw=0.0)
        # goal_dist_tolerance = 0.1 from stub; numpy bool -> cast to Python bool
        assert bool(self._real(c, RobotState(x=0.0, y=0.0))) is True

    def test_false_outside_tolerance(self):
        c = make_path_controller_stub()
        c.robot_state = RobotState(x=1.0, y=0.0, yaw=0.0)
        assert bool(self._real(c, RobotState(x=0.0, y=0.0))) is False


# ---------------------------------------------------------------------------
# _stop_robot
# ---------------------------------------------------------------------------

class TestStopRobot:
    def test_array_mode_publishes_one_cmd_array(self):
        c = make_path_controller_stub()
        c.config.ctrl_publish_type = CmdPublishType.TWIST_ARRAY
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)
        c._cmds_queue.put((1, 2, 3))

        Controller._stop_robot(c)

        assert c._cmds_queue.empty()
        pub.publish.assert_called_once()
        published_msg = pub.publish.call_args.args[0]
        assert list(published_msg.linear_velocities.x) == [0.0]
        assert list(published_msg.angular_velocities.z) == [0.0]

    def test_sequence_mode_publishes_single_zero_cmd(self):
        c = make_path_controller_stub()
        c.config.ctrl_publish_type = CmdPublishType.TWIST_SEQUENCE
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        Controller._stop_robot(c)

        pub.publish.assert_called_once_with([0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# _publish dispatch
# ---------------------------------------------------------------------------

class TestPublishDispatch:
    def test_parallel_mode_enqueues_commands(self):
        c = make_path_controller_stub()
        c.config.ctrl_publish_type = CmdPublishType.TWIST_PARALLEL
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        Controller._publish(c, [0.1, 0.2, 0.3], [0.0, 0.0, 0.0], [0.5, 0.4, 0.3])

        pub.publish.assert_not_called()
        queued = list(c._cmds_queue.queue)
        assert queued == [(0.1, 0.0, 0.5), (0.2, 0.0, 0.4), (0.3, 0.0, 0.3)]

    def test_array_mode_publishes_twist_array(self):
        c = make_path_controller_stub()
        c.config.ctrl_publish_type = CmdPublishType.TWIST_ARRAY
        c.config.control_time_step = 0.25
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        Controller._publish(c, [0.1, 0.2], [0.0, 0.0], [0.5, 0.4])

        pub.publish.assert_called_once()
        msg = pub.publish.call_args.args[0]
        assert list(msg.linear_velocities.x) == [0.1, 0.2]
        assert list(msg.linear_velocities.y) == [0.0, 0.0]
        assert list(msg.angular_velocities.z) == [0.5, 0.4]
        assert msg.time_step == 0.25

    def test_sequence_mode_publishes_each(self, monkeypatch):
        c = make_path_controller_stub()
        c.config.ctrl_publish_type = CmdPublishType.TWIST_SEQUENCE
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)
        # avoid real sleeps
        from kompass.components import controller as ctrl_mod
        monkeypatch.setattr(ctrl_mod.time, "sleep", lambda *_: None)

        Controller._publish(c, [0.1, 0.2], [0.0, 0.0], [0.5, 0.4])

        assert pub.publish.call_count == 2
        first = pub.publish.call_args_list[0].args[0]
        assert first == [0.1, 0.0, 0.5]


# ---------------------------------------------------------------------------
# _cmds_publishing_callback
# ---------------------------------------------------------------------------

class TestCmdsPublishingCallback:
    def test_skips_and_clears_queue_when_reached_end(self):
        c = make_path_controller_stub()
        _set_mangled(c, "reached_end", True)
        c._cmds_queue.put((0.1, 0.0, 0.0))
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        c._cmds_publishing_callback()

        pub.publish.assert_not_called()
        assert c._cmds_queue.empty()

    def test_publishes_one_cmd_from_queue(self):
        c = make_path_controller_stub()
        _set_mangled(c, "reached_end", False)
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)
        c._cmds_queue.put((0.1, 0.0, 0.2))

        c._cmds_publishing_callback()

        pub.publish.assert_called_once_with((0.1, 0.0, 0.2))

    def test_noop_on_empty_queue(self):
        c = make_path_controller_stub()
        _set_mangled(c, "reached_end", False)
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        c._cmds_publishing_callback()

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# _info_publishing_callback
# ---------------------------------------------------------------------------

class TestInfoPublishingCallback:
    def test_noop_in_vision_mode(self):
        c = make_path_controller_stub()
        c.config._mode = ControllerMode.VISION_FOLLOWER
        pub = MagicMock()
        c.get_publisher = MagicMock(return_value=pub)

        c._info_publishing_callback()

        pub.publish.assert_not_called()


# ---------------------------------------------------------------------------
# set_algorithm   (B10 regression: log on failure)
# ---------------------------------------------------------------------------

class TestSetAlgorithm:
    # set_algorithm is wrapped with @component_action which requires rclpy.
    # functools.wraps exposes the original via __wrapped__.
    _raw = staticmethod(Controller.set_algorithm.__wrapped__)

    def test_returns_true_when_value_matches_current(self):
        c = make_path_controller_stub()
        from kompass_core.control import ControllersID
        c.config.algorithm = ControllersID.DWA

        assert self._raw(c, "DWA") is True

    def test_returns_true_on_successful_change(self, monkeypatch):
        """Happy path: setter succeeds -> method must return True (not None)."""
        c = make_path_controller_stub()
        from kompass_core.control import ControllersID
        c.config.algorithm = ControllersID.DWA

        # Silent successful setter (avoids hitting _activate_follower_mode internals)
        monkeypatch.setattr(
            Controller,
            "algorithm",
            property(
                lambda self: self.config.algorithm,
                lambda self, value: setattr(self.config, "algorithm", value),
            ),
        )

        assert self._raw(c, "Stanley") is True

    def test_logs_and_returns_false_when_setter_raises(self, monkeypatch):
        """B10 regression: failure path logs via get_logger and returns False."""
        c = make_path_controller_stub()
        from kompass_core.control import ControllersID
        c.config.algorithm = ControllersID.DWA

        def _raise(self, value):
            raise RuntimeError("simulated setter failure")

        monkeypatch.setattr(
            Controller,
            "algorithm",
            property(lambda self: self.config.algorithm, _raise),
        )

        result = self._raw(c, "Stanley")

        assert result is False
        c.get_logger.return_value.error.assert_called()
