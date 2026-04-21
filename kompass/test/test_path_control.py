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
    """Return the Controller-mangled attribute name for a private method."""
    return f"_Controller__{name}"


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

    # _update_state / _publish / _stop_robot are side-effect surfaces; record calls
    c._update_state = MagicMock()
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
