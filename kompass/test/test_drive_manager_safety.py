"""Unit tests for DriveManager's per-tick safety gating helpers: staleness
handling, batched cloud-list assembly (zero-copy metadata dicts) and the
min-combination across checkers.

The helpers are exercised unbound on a duck-typed stub carrying only the
attributes they read - no ROS node or executor is needed, but importing the
kompass component module requires rclpy/ros_sugar on the path.
"""

import time

import numpy as np
import pytest

pytest.importorskip("rclpy")

from kompass.components.drive_manager import DriveManager  # noqa: E402


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


class _CloudData:
    """Minimal PointCloudData stand-in with the 8 metadata attributes"""

    def __init__(self):
        buffer = np.zeros((4, 4), dtype=np.float32)
        buffer[:, 0] = 5.0
        self.data = buffer.reshape(-1).view(np.uint8)
        self.point_step = 16
        self.row_step = 64
        self.height = 1
        self.width = 4
        self.x_offset = 0
        self.y_offset = 4
        self.z_offset = 8


class _Checker:
    def __init__(self, factor=1.0, raises=False):
        self.factor = factor
        self.raises = raises

    def check(self, **_):
        if self.raises:
            raise ValueError("malformed cloud metadata")
        return self.factor


class _Stub:
    """Duck-typed DriveManager stand-in for the safety helpers"""

    _gather_clouds = DriveManager._gather_clouds
    _check_scan = DriveManager._check_scan
    _run_safety_check = DriveManager._run_safety_check

    def __init__(
        self,
        pc_outputs=(),
        pc_ages=(),
        scan_checker=None,
        pc_checker=None,
        scan_age=0.0,
        timeout=0.2,
        stale_stop=True,
    ):
        now = time.monotonic()
        self._pc_callbacks = tuple(_Callback(output) for output in pc_outputs)
        self._pc_last_msg = [now - age for age in pc_ages]
        self._scan_callback = _Callback(
            type("Scan", (), {"ranges": np.zeros(4, dtype=np.float32)})()
        )
        self._scan_last_msg = now - scan_age
        self._sensor_timeout = timeout
        self._stale_stop = stale_stop
        self._scan_checker = scan_checker
        self._pc_checker = pc_checker
        self._logger = _Logger()

    def get_logger(self):
        return self._logger


def test_gather_clouds_fresh_builds_metadata_dicts():
    clouds_in = [_CloudData(), _CloudData()]
    stub = _Stub(pc_outputs=clouds_in, pc_ages=[0.0, 0.0])
    clouds, fresh = stub._gather_clouds(time.monotonic())
    assert fresh == 2
    assert len(clouds) == 2
    for element, source in zip(clouds, clouds_in):
        assert set(element) == {
            "data",
            "point_step",
            "row_step",
            "height",
            "width",
            "x_offset",
            "y_offset",
            "z_offset",
        }
        # Zero-copy contract: the dict must carry the SAME buffer object,
        # never a copy or conversion
        assert element["data"] is source.data


def test_gather_clouds_stale_skip_gives_none_slot():
    stub = _Stub(
        pc_outputs=[_CloudData(), _CloudData()],
        pc_ages=[0.0, 5.0],  # second sensor stale
        stale_stop=False,
    )
    clouds, fresh = stub._gather_clouds(time.monotonic())
    assert fresh == 1
    assert clouds[0] is not None
    assert clouds[1] is None


def test_gather_clouds_stale_stop_returns_none():
    stub = _Stub(
        pc_outputs=[_CloudData(), _CloudData()],
        pc_ages=[0.0, 5.0],
        stale_stop=True,
    )
    assert stub._gather_clouds(time.monotonic()) is None


def test_run_safety_check_min_combines_scan_and_clouds():
    stub = _Stub(
        pc_outputs=[_CloudData()],
        pc_ages=[0.0],
        scan_checker=_Checker(factor=0.5),
        pc_checker=_Checker(factor=0.7),
    )
    assert stub._run_safety_check(forward=True) == 0.5
    stub._scan_checker.factor = 0.9
    assert stub._run_safety_check(forward=True) == 0.7


def test_run_safety_check_scan_critical_short_circuits():
    stub = _Stub(
        pc_outputs=[_CloudData()],
        pc_ages=[0.0],
        scan_checker=_Checker(factor=0.0),
        pc_checker=_Checker(factor=1.0),
    )
    assert stub._run_safety_check(forward=True) == 0.0


def test_run_safety_check_all_stale_returns_zero_even_when_skipping():
    stub = _Stub(
        pc_outputs=[_CloudData()],
        pc_ages=[5.0],
        pc_checker=_Checker(factor=1.0),
        scan_checker=None,
        stale_stop=False,
    )
    # Every sensor skipped -> a fully-stale fleet must not read as "safe"
    assert stub._run_safety_check(forward=True) == 0.0


def test_run_safety_check_stale_scan_under_stop_policy():
    stub = _Stub(
        scan_checker=_Checker(factor=1.0),
        scan_age=5.0,
        stale_stop=True,
    )
    assert stub._run_safety_check(forward=True) == 0.0


def test_run_safety_check_checker_error_returns_zero():
    stub = _Stub(
        pc_outputs=[_CloudData()],
        pc_ages=[0.0],
        pc_checker=_Checker(raises=True),
    )
    assert stub._run_safety_check(forward=True) == 0.0
