"""Unit tests for LocalMapper's tick-driven multi-cloud fusion helper:
staleness gating, zero-copy metadata dicts and the all-stale skip (the last
grid must be kept, not wiped).

The helper is exercised unbound on a duck-typed stub - no ROS node or
executor is needed, but importing the kompass component module requires
rclpy/ros_sugar on the path.
"""

import time

import numpy as np
import pytest

pytest.importorskip("rclpy")

from kompass.components.mapper import LocalMapper  # noqa: E402
from ros_sugar.io import PointCloudData  # noqa: E402


class _Logger:
    def warning(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass


class _Callback:
    def __init__(self, output):
        self._output = output

    def get_output(self):
        return self._output


def _CloudData() -> PointCloudData:
    """A real PointCloudData, as PointCloudCallback hands it out (the gather relies on its buffer_layout())"""
    buffer = np.zeros((4, 4), dtype=np.float32)
    buffer[:, 0] = 2.0
    return PointCloudData(
        data=buffer.reshape(-1).view(np.uint8),
        point_step=16,
        row_step=64,
        height=1,
        width=4,
        x_offset=0,
        y_offset=4,
        z_offset=8,
    )


class _Builder:
    """Captures update_from_pointclouds calls"""

    def __init__(self):
        self.calls = []

    def update_from_pointclouds(self, robot_pose, *, clouds):
        self.calls.append((robot_pose, clouds))


class _State:
    x = 1.0
    y = 2.0
    yaw = 0.5


class _HealthStatus:
    """Records the health failures the mapper reports"""

    def __init__(self):
        self.failed_topics = []

    def set_fail_system(self, topic_names):
        self.failed_topics.extend(topic_names)


class _Topic:
    def __init__(self, name):
        self.name = name


class _Stub:
    _update_map_from_clouds = LocalMapper._update_map_from_clouds
    _robot_pose_in_world = LocalMapper._robot_pose_in_world

    def __init__(self, outputs, ages, timeout=0.2):
        now = time.monotonic()
        self._pc_callbacks = tuple(_Callback(output) for output in outputs)
        self._pc_last_msg = [now - age for age in ages]
        self._sensor_timeout = timeout
        self._local_map_builder = _Builder()
        self.robot_state = _State()
        self._logger = _Logger()
        self.health_status = _HealthStatus()

    def get_logger(self):
        return self._logger

    def get_in_topic(self, key):
        return _Topic(str(key))


def test_fresh_clouds_are_fused_with_metadata_dicts():
    clouds_in = [_CloudData(), _CloudData()]
    stub = _Stub(outputs=clouds_in, ages=[0.0, 0.0])
    stub._update_map_from_clouds()
    assert len(stub._local_map_builder.calls) == 1
    pose, clouds = stub._local_map_builder.calls[0]
    assert pose.x == 1.0 and pose.y == 2.0
    assert len(clouds) == 2
    for element, source in zip(clouds, clouds_in):
        # Zero-copy contract: the dict must carry the SAME buffer object
        assert element["data"] is source.data


def test_stale_sensor_contributes_none_slot():
    stub = _Stub(outputs=[_CloudData(), _CloudData()], ages=[0.0, 5.0])
    stub._update_map_from_clouds()
    _, clouds = stub._local_map_builder.calls[0]
    assert clouds[0] is not None
    assert clouds[1] is None


def test_all_stale_skips_the_update_entirely():
    stub = _Stub(outputs=[_CloudData(), _CloudData()], ages=[5.0, 5.0])
    stub._update_map_from_clouds()
    # No update -> the previous grid is kept instead of being wiped
    assert stub._local_map_builder.calls == []


def test_missing_robot_state_skips_the_update():
    stub = _Stub(outputs=[_CloudData()], ages=[0.0])
    stub.robot_state = None
    stub._update_map_from_clouds()
    assert stub._local_map_builder.calls == []
    # The skip must be reported as a system failure, not swallowed silently
    assert stub.health_status.failed_topics
