"""Unit tests for the vision follower's tracking frame.

The follower tracks in the LOCAL (robot-relative) frame unless the vision
algorithm's configuration asks for the world frame
(``use_local_coordinates=False``). Only then is localization looked for: a
request without it falls back to LOCAL with a warning, and the per-mission
refresh moves to GLOBAL once localization comes up. Frame mode is baked into
the core controller at construction, so that move must rebuild via setup();
everything else must be a cheap no-op.

Exercised on a duck-typed stub component - no ROS node or executor is
needed, but importing the component module requires rclpy/ros_sugar on
the path.
"""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")

from kompass.components._modes import FrameMode  # noqa: E402
from kompass.components._vision_follower import VisionFollower  # noqa: E402


class _Logger:
    def __init__(self):
        self.warnings = []

    def info(self, *args, **kwargs):
        pass

    def warning(self, message, *args, **kwargs):
        self.warnings.append(message)

    def error(self, *args, **kwargs):
        pass


class _TfListener:
    def __init__(self, got_transform: bool):
        self.got_transform = got_transform


class _Config:
    def __init__(self, frame_mode):
        self._frame_mode = frame_mode


class _Component:
    def __init__(self, frame_mode, tf_listener):
        self.config = _Config(frame_mode)
        self.odom_tf_listener = tf_listener
        self.logger = _Logger()
        self.state_probes = 0

    def get_logger(self):
        return self.logger

    def _update_state(self, block=False):
        self.state_probes += 1


def _follower(frame_mode, tf_listener, global_requested=False):
    follower = VisionFollower(_Component(frame_mode, tf_listener))
    follower._global_requested = global_requested
    # Count rebuilds instead of running the real (blocking) setup
    follower.setup_calls = 0

    def _fake_setup():
        follower.setup_calls += 1
        return True

    follower.setup = _fake_setup
    return follower


# ---------------------------------------------------------------------------
# Frame chosen at setup from the algorithm configuration
# ---------------------------------------------------------------------------


def test_local_by_default_without_looking_for_localization():
    """Localization being available does not switch the frame on its own."""
    follower = _follower(FrameMode.GLOBAL, _TfListener(True))
    config = SimpleNamespace(use_local_coordinates=True)
    follower._resolve_frame_mode(config)

    assert follower._component.config._frame_mode == FrameMode.LOCAL
    assert follower._global_requested is False
    assert follower._component.state_probes == 0
    assert config.use_local_coordinates is True


def test_image_follower_without_the_setting_is_local():
    follower = _follower(FrameMode.GLOBAL, _TfListener(True))
    follower._resolve_frame_mode(SimpleNamespace())

    assert follower._component.config._frame_mode == FrameMode.LOCAL
    assert follower._global_requested is False


def test_requested_world_frame_with_localization_is_global():
    follower = _follower(FrameMode.LOCAL, _TfListener(True))
    config = SimpleNamespace(use_local_coordinates=False)
    follower._resolve_frame_mode(config)

    assert follower._component.config._frame_mode == FrameMode.GLOBAL
    assert follower._global_requested is True
    assert follower._component.state_probes == 1
    assert config.use_local_coordinates is False
    assert follower._component.logger.warnings == []


def test_requested_world_frame_without_localization_falls_back_to_local():
    follower = _follower(FrameMode.LOCAL, _TfListener(False))
    config = SimpleNamespace(use_local_coordinates=False)
    follower._resolve_frame_mode(config)

    assert follower._component.config._frame_mode == FrameMode.LOCAL
    assert follower._global_requested is True
    # The core controller is built to match the frame the component uses
    assert config.use_local_coordinates is True
    assert len(follower._component.logger.warnings) == 1


# ---------------------------------------------------------------------------
# Per-mission refresh
# ---------------------------------------------------------------------------


def test_global_mode_is_a_no_op():
    follower = _follower(FrameMode.GLOBAL, _TfListener(True), global_requested=True)
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 0


def test_local_without_localization_stays_local():
    follower = _follower(FrameMode.LOCAL, _TfListener(False), global_requested=True)
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 0

    follower_no_listener = _follower(FrameMode.LOCAL, None, global_requested=True)
    assert follower_no_listener._refresh_frame_mode() is True
    assert follower_no_listener.setup_calls == 0


def test_local_by_default_is_not_upgraded_when_localization_appears():
    follower = _follower(FrameMode.LOCAL, _TfListener(True), global_requested=False)
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 0


def test_requested_world_frame_rebuilds_once_localization_appears():
    follower = _follower(FrameMode.LOCAL, _TfListener(True), global_requested=True)
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 1


def test_failed_rebuild_is_reported():
    follower = _follower(FrameMode.LOCAL, _TfListener(True), global_requested=True)
    follower.setup = lambda: False
    assert follower._refresh_frame_mode() is False
