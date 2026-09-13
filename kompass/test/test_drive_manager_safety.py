"""Unit tests for DriveManager's per-tick safety gating helpers: staleness
handling, batched cloud-list assembly (zero-copy metadata dicts) and the
min-combination across checkers.

The helpers are exercised unbound on a duck-typed stub carrying only the
attributes they read - no ROS node or executor is needed, but importing the
kompass component module requires rclpy/ros_sugar on the path.
"""

import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rclpy")

from kompass.components.drive_manager import DriveManager  # noqa: E402
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
    buffer[:, 0] = 5.0
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
    _check_ranges = DriveManager._check_ranges
    _resolve_range_facing = DriveManager._resolve_range_facing
    _on_range_reading = DriveManager._on_range_reading
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
        range_readings=(),
        range_ages=(),
        critical_distance=0.3,
        range_facing=(),
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
        # Range sensors: the last valid reading (None = not a measurement)
        self._range_callbacks = tuple(
            SimpleNamespace(input_topic=SimpleNamespace(name=f"/range_{i}"))
            for i in range(len(range_readings))
        )
        self._range_readings = list(range_readings)
        self._range_facing = list(range_facing) or [2] * len(range_readings)
        self._range_last_msg = [now - age for age in range_ages] or [now] * len(
            range_readings
        )
        self.config = SimpleNamespace(critical_zone_distance=critical_distance)
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


# ---------------------------------------------------------------------------
# Range (single-beam) safety sensors, e.g. the Lite3 ultrasounds
# ---------------------------------------------------------------------------


def _range_msg(reading, min_range=0.28, max_range=4.5, frame_id="ultrasound_front"):
    return SimpleNamespace(
        range=reading,
        min_range=min_range,
        max_range=max_range,
        header=SimpleNamespace(frame_id=frame_id),
    )


def test_range_inside_critical_distance_stops():
    stub = _Stub(range_readings=[0.2], critical_distance=0.3)
    assert stub._run_safety_check(forward=True) == 0.0
    assert stub._run_safety_check(forward=False) == 0.0


def test_range_beyond_critical_distance_is_clear_and_counts_as_checked():
    """A Range-only setup with nothing close must not read as 'all sensors
    stale'."""
    stub = _Stub(range_readings=[1.2], critical_distance=0.3)
    assert stub._run_safety_check(forward=True) == 1.0


def test_range_reading_outside_the_sensor_limits_is_not_a_measurement():
    """Below min_range or above max_range the value is discarded (the
    sensor saw nothing it can report), so it neither stops the robot nor
    counts as a stale sensor."""
    stub = _Stub(range_readings=[None], critical_distance=0.3)
    stub._on_range_reading(0, msg=_range_msg(0.0))  # below the 0.28 m minimum
    assert stub._range_readings[0] is None
    assert stub._run_safety_check(forward=True) == 1.0
    stub._on_range_reading(0, msg=_range_msg(9.0))  # beyond the 4.5 m maximum
    assert stub._range_readings[0] is None
    stub._on_range_reading(0, msg=_range_msg(float("nan")))
    assert stub._range_readings[0] is None
    stub._on_range_reading(0, msg=_range_msg(0.25, min_range=0.0))
    assert stub._range_readings[0] == 0.25
    assert stub._run_safety_check(forward=True) == 0.0


def test_range_reading_refreshes_the_arrival_stamp():
    stub = _Stub(range_readings=[1.0], range_ages=[5.0], timeout=0.2)
    assert stub._run_safety_check(forward=True) == 0.0  # stale under "stop"
    stub._on_range_reading(0, msg=_range_msg(1.0))
    assert stub._run_safety_check(forward=True) == 1.0


def test_stale_range_follows_the_stale_sensor_policy():
    stopping = _Stub(range_readings=[1.0], range_ages=[1.0], timeout=0.2, stale_stop=True)
    assert stopping._run_safety_check(forward=True) == 0.0
    skipping = _Stub(range_readings=[1.0], range_ages=[1.0], timeout=0.2, stale_stop=False)
    # Skipped and nothing else configured -> every sensor skipped -> unsafe
    assert skipping._run_safety_check(forward=True) == 0.0
    # Skipped next to a fresh scan -> the scan decides
    skipping = _Stub(
        range_readings=[1.0],
        range_ages=[1.0],
        timeout=0.2,
        stale_stop=False,
        scan_checker=_Checker(factor=0.5),
    )
    assert skipping._run_safety_check(forward=True) == 0.5


def test_range_combines_with_the_other_checkers_by_minimum():
    stub = _Stub(range_readings=[1.0], scan_checker=_Checker(factor=0.4), critical_distance=0.3)
    assert stub._run_safety_check(forward=True) == 0.4
    stub = _Stub(range_readings=[0.1], scan_checker=_Checker(factor=0.4), critical_distance=0.3)
    assert stub._run_safety_check(forward=True) == 0.0


def test_range_facing_uses_the_critical_zone_cone():
    """The beam is in the forward cone within critical_zone_angle/2 of +x,
    in the backward cone within that of -x, outside both otherwise, and
    constrains either way when its TF cannot be resolved."""

    def facing(yaw_deg, cone_deg, resolved=True):
        stub = _Stub()
        stub.config = SimpleNamespace(
            critical_zone_distance=0.3,
            critical_zone_angle=cone_deg,
            topic_subscription_timeout=0.0,
        )
        rotation = np.array([0.0, 0.0, np.sin(np.radians(yaw_deg) / 2), np.cos(np.radians(yaw_deg) / 2)])
        listener = SimpleNamespace(rotation=rotation) if resolved else None
        stub.wait_input_tf = lambda *_, **__: listener
        callback = SimpleNamespace(input_topic=SimpleNamespace(name="/range"))
        return stub._resolve_range_facing(callback, 0)

    assert facing(0, 100.0) == 1
    assert facing(180, 100.0) == -1
    assert facing(30, 100.0) == 1  # inside the 50 deg half cone
    assert facing(60, 100.0) == 0  # outside it
    assert facing(60, 180.0) == 1  # a wider cone takes it
    assert facing(90, 100.0) == 0
    assert facing(150, 100.0) == -1
    assert facing(0, 100.0, resolved=False) == 2


def test_range_beam_only_constrains_motion_in_its_direction():
    """A front beam reading 0.1 m stops forward motion but not reversing,
    the rear beam the other way round; a scan is present so the skipped
    beam leaves the scan's verdict."""
    front = _Stub(range_readings=[0.1], range_facing=[1], scan_checker=_Checker(1.0))
    assert front._run_safety_check(forward=True) == 0.0
    assert front._run_safety_check(forward=False) == 1.0
    rear = _Stub(range_readings=[0.1], range_facing=[-1], scan_checker=_Checker(1.0))
    assert rear._run_safety_check(forward=False) == 0.0
    assert rear._run_safety_check(forward=True) == 1.0


def test_unresolved_beam_constrains_both_directions_and_outside_cone_none():
    unresolved = _Stub(range_readings=[0.1], range_facing=[2])
    assert unresolved._run_safety_check(forward=True) == 0.0
    assert unresolved._run_safety_check(forward=False) == 0.0
    outside = _Stub(range_readings=[0.1], range_facing=[0], scan_checker=_Checker(1.0))
    assert outside._run_safety_check(forward=True) == 1.0
    assert outside._run_safety_check(forward=False) == 1.0
