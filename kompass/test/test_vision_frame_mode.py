"""Unit tests for VisionFollower._refresh_frame_mode: the per-mission
LOCAL -> GLOBAL upgrade once localization becomes available.

Frame mode is baked into the core controller at construction, so the
upgrade must rebuild via setup(); everything else must be a cheap no-op.
Exercised on a duck-typed stub component - no ROS node or executor is
needed, but importing the component module requires rclpy/ros_sugar on
the path.
"""

import pytest

pytest.importorskip("rclpy")

from kompass.components._modes import FrameMode  # noqa: E402
from kompass.components._vision_follower import VisionFollower  # noqa: E402


class _Logger:
    def info(self, *args, **kwargs):
        pass

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

    def get_logger(self):
        return _Logger()


def _follower(frame_mode, tf_listener):
    follower = VisionFollower(_Component(frame_mode, tf_listener))
    # Count rebuilds instead of running the real (blocking) setup
    follower.setup_calls = 0

    def _fake_setup():
        follower.setup_calls += 1
        return True

    follower.setup = _fake_setup
    return follower


def test_global_mode_is_a_no_op():
    follower = _follower(FrameMode.GLOBAL, _TfListener(True))
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 0


def test_local_without_localization_stays_local():
    follower = _follower(FrameMode.LOCAL, _TfListener(False))
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 0

    follower_no_listener = _follower(FrameMode.LOCAL, None)
    assert follower_no_listener._refresh_frame_mode() is True
    assert follower_no_listener.setup_calls == 0


def test_local_with_localization_rebuilds_via_setup():
    follower = _follower(FrameMode.LOCAL, _TfListener(True))
    assert follower._refresh_frame_mode() is True
    assert follower.setup_calls == 1


def test_failed_rebuild_is_reported():
    follower = _follower(FrameMode.LOCAL, _TfListener(True))
    follower.setup = lambda: False
    assert follower._refresh_frame_mode() is False
