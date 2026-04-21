"""Correctness unit tests for the vision tracking controller.

These tests exercise the helper methods of
``kompass.components.controller.Controller`` without instantiating a ROS node.
The controller is built via ``object.__new__`` and only the attributes each
helper needs are attached — no rclpy runtime, no TF, no publishers, no
simulation required.
"""

from __future__ import annotations

import threading
import time
from queue import Queue
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

from kompass_interfaces.action import TrackVisionTarget

from kompass.components.controller import (
    CmdPublishType,
    Controller,
    ControllerMode,
)
from kompass.components.defaults import TopicsKeys
from kompass_core.datatypes import LaserScanData


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mangle(name: str) -> str:
    """Return the Controller-mangled attribute name for a private method."""
    return f"_Controller__{name}"


def make_controller_stub(**overrides) -> Controller:
    """Build a bare Controller with mock attributes attached.

    Each test tops up whatever extra state it needs; defaults here are
    the ones common to most of the helpers under test.
    """
    c = object.__new__(Controller)

    # Threading / queues
    c._main_goal_lock = threading.Lock()
    c._cmds_queue = Queue()

    # Tracking state
    setattr(c, _mangle("reached_end"), False)
    c._tracked_center = None
    c.vision_detections = None
    c.depth_image = None
    c.sensor_data = None
    c.local_map = None
    c.local_map_resolution = None
    c.robot_state = None
    c.plan = None

    # Config (MagicMock so tests can freely tweak nested attrs)
    c.config = MagicMock()
    c.config._mode = ControllerMode.VISION_FOLLOWER
    c.config.use_direct_sensor = False
    c.config.ctrl_publish_type = CmdPublishType.TWIST_ARRAY
    c.config.topic_subscription_timeout = 0.1
    c.config.loop_rate = 100.0
    c.config.frames = MagicMock()
    c.config.frames.odom = "odom"
    c.config.frames.world = "odom"  # same-frame -> no TF needed

    # TF listener (property reads self._odom_tf_listener lazily; bypass by setting it directly)
    tf_listener = MagicMock()
    tf_listener.transform = MagicMock()
    c._odom_tf_listener = tf_listener

    # IO fakes
    c.get_callback = MagicMock(return_value=None)
    c.get_publisher = MagicMock()
    c.get_logger = MagicMock()

    # Controller references
    setattr(c, _mangle("vision_controller"), None)

    # _stop_robot recorder (instead of calling the real one which needs publishers)
    c._stop_robot_calls = []
    c._stop_robot = lambda: c._stop_robot_calls.append(True)

    for k, v in overrides.items():
        setattr(c, k, v)
    return c


def make_goal_handle(is_cancel_requested: bool = False, is_active: bool = True):
    """Plain fake for rclpy ServerGoalHandle with the attrs/methods we use."""
    gh = SimpleNamespace()
    gh.is_cancel_requested = is_cancel_requested
    gh.is_active = is_active
    gh.canceled = MagicMock()
    gh.abort = MagicMock()
    gh.succeed = MagicMock()
    gh.publish_feedback = MagicMock()
    gh.request = SimpleNamespace(label="", pose_x=0, pose_y=0)
    return gh


# ---------------------------------------------------------------------------
# __gather_local_obstacles
# ---------------------------------------------------------------------------

class TestGatherLocalObstacles:
    def test_routes_laser_scan_when_direct_sensor(self):
        c = make_controller_stub()
        c.config.use_direct_sensor = True
        scan = MagicMock(spec=LaserScanData)
        c.sensor_data = scan

        laser, pc, local = getattr(c, _mangle("gather_local_obstacles"))()
        assert laser is scan
        assert pc is None
        assert local is None

    def test_routes_point_cloud_when_direct_sensor_and_not_laser(self):
        c = make_controller_stub()
        c.config.use_direct_sensor = True
        pc_data = object()  # not a LaserScanData instance
        c.sensor_data = pc_data

        laser, pc, local = getattr(c, _mangle("gather_local_obstacles"))()
        assert laser is None
        assert pc is pc_data
        assert local is None

    def test_routes_local_map_when_not_direct_sensor(self):
        c = make_controller_stub()
        c.config.use_direct_sensor = False
        grid = np.zeros((4, 4), dtype=np.int8)
        c.local_map = grid

        laser, pc, local = getattr(c, _mangle("gather_local_obstacles"))()
        assert laser is None
        assert pc is None
        assert local is grid


# ---------------------------------------------------------------------------
# __terminate_vision_action
# ---------------------------------------------------------------------------

class TestTerminateVisionAction:
    def _call(self, c, gh, status, started_ago: float = 0.05):
        result = TrackVisionTarget.Result()
        return getattr(c, _mangle("terminate_vision_action"))(
            gh, result, time.time() - started_ago, status
        )

    def test_cancel_when_requested_calls_canceled(self):
        c = make_controller_stub()
        gh = make_goal_handle(is_cancel_requested=True)
        result = self._call(c, gh, "cancel")
        gh.canceled.assert_called_once()
        gh.abort.assert_not_called()
        gh.succeed.assert_not_called()
        assert result.success is False
        assert result.tracked_duration > 0.0

    def test_cancel_without_request_falls_through_to_abort_when_active(self):
        """Edge case: callback saw !is_active, not a genuine cancel. Should abort."""
        c = make_controller_stub()
        gh = make_goal_handle(is_cancel_requested=False, is_active=True)
        self._call(c, gh, "cancel")
        gh.canceled.assert_not_called()
        gh.abort.assert_called_once()

    def test_cancel_noop_when_inactive_and_not_cancel_requested(self):
        c = make_controller_stub()
        gh = make_goal_handle(is_cancel_requested=False, is_active=False)
        self._call(c, gh, "cancel")
        gh.canceled.assert_not_called()
        gh.abort.assert_not_called()
        gh.succeed.assert_not_called()

    def test_abort_calls_abort_when_active(self):
        c = make_controller_stub()
        gh = make_goal_handle(is_active=True)
        result = self._call(c, gh, "abort", started_ago=0.25)
        gh.abort.assert_called_once()
        gh.succeed.assert_not_called()
        gh.canceled.assert_not_called()
        assert result.success is False
        assert result.tracked_duration >= 0.25

    def test_abort_is_noop_when_already_inactive(self):
        c = make_controller_stub()
        gh = make_goal_handle(is_active=False)
        self._call(c, gh, "abort")
        gh.abort.assert_not_called()

    def test_succeed_sets_success_and_transitions_goal(self):
        c = make_controller_stub()
        gh = make_goal_handle(is_active=True)
        result = self._call(c, gh, "succeed")
        gh.succeed.assert_called_once()
        gh.abort.assert_not_called()
        gh.canceled.assert_not_called()
        assert result.success is True

    def test_terminate_resets_state_and_stops_robot(self):
        c = make_controller_stub()
        c._tracked_center = np.array([10, 20])
        c.vision_detections = [object()]
        setattr(c, _mangle("reached_end"), False)
        gh = make_goal_handle(is_active=True)
        self._call(c, gh, "abort")
        assert getattr(c, _mangle("reached_end")) is True
        assert c._tracked_center is None
        assert c.vision_detections is None
        assert c._stop_robot_calls == [True]


# ---------------------------------------------------------------------------
# _update_state(block=False)
# ---------------------------------------------------------------------------

class TestUpdateStateNonBlocking:
    def test_returns_none_without_waiting_when_tf_missing(self):
        """Confirms non-blocking path skips the TF wait loop."""
        c = make_controller_stub()
        c.config.frames.odom = "odom"
        c.config.frames.world = "map"
        c.odom_tf_listener.transform = None
        state_cb = MagicMock()
        c.get_callback = MagicMock(
            side_effect=lambda key: state_cb
            if key == TopicsKeys.ROBOT_LOCATION
            else None
        )
        c.robot_state = "stale"

        t0 = time.time()
        c._update_state(block=False)
        elapsed = time.time() - t0

        assert c.robot_state is None
        state_cb.get_output.assert_not_called()
        # must not have slept anywhere near topic_subscription_timeout
        assert elapsed < c.config.topic_subscription_timeout

    def test_uses_cached_transform_when_available(self):
        c = make_controller_stub()
        c.config.frames.odom = "odom"
        c.config.frames.world = "map"
        cached_tf = object()
        c.odom_tf_listener.transform = cached_tf
        state_cb = MagicMock()
        state_cb.get_output.return_value = "STATE"
        c.get_callback = MagicMock(
            side_effect=lambda key: state_cb
            if key == TopicsKeys.ROBOT_LOCATION
            else None
        )

        c._update_state(block=False)
        state_cb.get_output.assert_called_once_with(
            transformation=cached_tf, get_front=True, clear_last=True
        )
        assert c.robot_state == "STATE"

    def test_same_frame_bypasses_tf_entirely(self):
        c = make_controller_stub()
        c.config.frames.odom = "odom"
        c.config.frames.world = "odom"
        c.odom_tf_listener.transform = None  # irrelevant
        state_cb = MagicMock()
        state_cb.get_output.return_value = "STATE"
        c.get_callback = MagicMock(
            side_effect=lambda key: state_cb
            if key == TopicsKeys.ROBOT_LOCATION
            else None
        )

        c._update_state(block=False)
        state_cb.get_output.assert_called_once_with(get_front=True, clear_last=True)
        assert c.robot_state == "STATE"


# ---------------------------------------------------------------------------
# _vision_tracking_callback — high-level branches
# ---------------------------------------------------------------------------

class TestVisionTrackingCallback:
    def _stub_loop_deps(self, c):
        """No-op the input refreshers and time.sleep so the callback loop is fast."""
        c._update_vision = MagicMock()
        c._update_state = MagicMock()
        c._publish = MagicMock()
        setattr(c, _mangle("setup_vision_controller"), MagicMock())
        setattr(
            c, _mangle("setup_initial_tracking_target"), MagicMock(return_value=True)
        )

    def _stub_vision_controller(self, c, loop_step_return=True):
        vc = MagicMock()
        vc.loop_step.return_value = loop_step_return
        vc.linear_x_control = [0.0]
        vc.linear_y_control = [0.0]
        vc.angular_control = [0.0]
        vc.dist_error = 0.0
        vc.orientation_error = 0.0
        setattr(c, _mangle("vision_controller"), vc)
        return vc

    def test_aborts_when_vision_controller_init_fails(self):
        c = make_controller_stub()
        self._stub_loop_deps(c)
        # setup returns None -> init failure
        getattr(c, _mangle("setup_vision_controller")).return_value = None

        gh = make_goal_handle()
        result = c._vision_tracking_callback(gh)

        gh.abort.assert_called_once()
        assert result.success is False
        assert c._stop_robot_calls == [True]

    def test_aborts_when_initial_target_not_found(self):
        c = make_controller_stub()
        self._stub_loop_deps(c)
        self._stub_vision_controller(c)
        getattr(c, _mangle("setup_initial_tracking_target")).return_value = False

        gh = make_goal_handle()
        result = c._vision_tracking_callback(gh)

        gh.abort.assert_called_once()
        assert result.success is False

    def test_cancel_mid_loop_calls_canceled(self):
        c = make_controller_stub()
        self._stub_loop_deps(c)
        self._stub_vision_controller(c, loop_step_return=True)

        class FlippingGoal(SimpleNamespace):
            """is_cancel_requested flips True after N reads."""

            def __init__(self, flip_after):
                super().__init__()
                self.is_active = True
                self._flip_after = flip_after
                self._reads = 0
                self.canceled = MagicMock()
                self.abort = MagicMock()
                self.succeed = MagicMock()
                self.publish_feedback = MagicMock()
                self.request = SimpleNamespace(label="", pose_x=0, pose_y=0)

            @property
            def is_cancel_requested(self):
                self._reads += 1
                return self._reads >= self._flip_after

        gh = FlippingGoal(flip_after=1)  # first check triggers cancel
        with patch("kompass.components.controller.time.sleep", lambda *_: None):
            result = c._vision_tracking_callback(gh)

        gh.canceled.assert_called_once()
        gh.abort.assert_not_called()
        gh.succeed.assert_not_called()
        assert result.success is False
        assert c._stop_robot_calls == [True]

    def test_abort_when_loop_step_returns_false(self):
        c = make_controller_stub()
        self._stub_loop_deps(c)
        self._stub_vision_controller(c, loop_step_return=False)

        gh = make_goal_handle()
        with patch("kompass.components.controller.time.sleep", lambda *_: None):
            result = c._vision_tracking_callback(gh)

        gh.abort.assert_called_once()
        gh.succeed.assert_not_called()
        gh.canceled.assert_not_called()
        assert result.success is False
        assert c._stop_robot_calls == [True]

    def test_succeed_when_mode_changes_to_path_follower(self):
        c = make_controller_stub()
        self._stub_loop_deps(c)
        vc = self._stub_vision_controller(c, loop_step_return=True)

        # After one successful iteration, flip mode to exit the while loop.
        call_count = {"n": 0}

        def loop_step_side_effect(**_):
            call_count["n"] += 1
            if call_count["n"] >= 1:
                c.config._mode = ControllerMode.PATH_FOLLOWER
            return True

        vc.loop_step.side_effect = loop_step_side_effect

        gh = make_goal_handle()
        with patch("kompass.components.controller.time.sleep", lambda *_: None):
            result = c._vision_tracking_callback(gh)

        gh.succeed.assert_called_once()
        gh.abort.assert_not_called()
        gh.canceled.assert_not_called()
        assert result.success is True
