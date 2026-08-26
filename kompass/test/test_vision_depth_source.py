"""Unit tests for the separate depth input of the vision callbacks
(``VISION_DEPTH``): how a depth Image or a PointCloud2 subscribed next to
the detections is paired with them and handed to the core, and how the
Controller's vision follower and the Planner wire it up.

Exercised on duck-typed stubs - no ROS node or executor is needed, but
importing the modules requires rclpy/ros_sugar on the path.
"""

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

pytest.importorskip("rclpy")

import kompass.components  # noqa: E402, F401  (resolves the package import order)
from kompass._external_types import _ExternalDepthMixin  # noqa: E402
from ros_sugar.io import PointCloudData  # noqa: E402


def _stamp(seconds: float):
    return SimpleNamespace(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


def _message(stamp_seconds: float):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(stamp_seconds)))


def _source(output, stamp_seconds: float, frame_id: str = "camera_optical"):
    """A stub sugarcoat callback: the last message, its frame and its output"""
    return SimpleNamespace(
        msg=_message(stamp_seconds),
        frame_id=frame_id,
        input_topic=SimpleNamespace(name="/vision_depth"),
        get_output=lambda: output,
    )


class _Detections(_ExternalDepthMixin):
    """The state the mixin relies on, as the vision callbacks keep it"""

    def __init__(self, stamp_seconds: float = 10.0, embedded=None):
        self.msg = _message(stamp_seconds)
        self._depth_image = embedded
        self._img_size = None
        self.node_name = "test"


def _cloud(n: int = 4) -> PointCloudData:
    """A real sugarcoat cloud container, as PointCloudCallback hands it out"""
    records = np.zeros((n, 4), dtype=np.float32)
    return PointCloudData(
        data=records.reshape(-1).view(np.uint8),
        point_step=16,
        row_step=16 * n,
        height=1,
        width=n,
        x_offset=0,
        y_offset=4,
        z_offset=8,
    )


def test_without_a_source_the_embedded_depth_is_used():
    embedded = np.zeros((4, 4), dtype=np.uint16)
    assert _Detections(embedded=embedded).depth == {"depth_image": embedded}
    assert _Detections().depth == {}


def test_depth_image_within_max_age_is_handed_over_as_is():
    image = np.zeros((4, 4), dtype=np.uint16)
    detections = _Detections(stamp_seconds=10.0, embedded=np.ones((4, 4), np.uint16))
    detections.set_depth_source(_source(image, 10.05), max_age=0.2)
    # The separate input wins over the embedded depth, without a copy
    assert detections.depth["depth_image"] is image


def test_stale_depth_gives_no_depth_this_tick():
    image = np.zeros((4, 4), dtype=np.uint16)
    detections = _Detections(stamp_seconds=10.0)
    detections.set_depth_source(_source(image, 9.5), max_age=0.2)
    assert detections.depth == {}
    detections.set_depth_source(_source(image, 9.5), max_age=1.0)
    assert detections.depth["depth_image"] is image


def test_depth_image_in_another_frame_is_not_used():
    """A depth image that is not registered to the detection camera has no
    pixel correspondence with the boxes, so it counts as no depth rather than
    producing wrong targets. Without a camera frame to compare against the
    check is skipped."""
    image = np.zeros((4, 4), dtype=np.uint16)
    detections = _Detections(stamp_seconds=10.0)
    detections.set_depth_source(
        _source(image, 10.0, frame_id="lidar_link"),
        max_age=0.2,
        camera_frame="camera_optical",
    )
    assert detections.depth == {}
    detections.set_depth_source(
        _source(image, 10.0, frame_id="camera_optical"),
        max_age=0.2,
        camera_frame="camera_optical",
    )
    assert detections.depth["depth_image"] is image
    detections.set_depth_source(
        _source(image, 10.0, frame_id="lidar_link"), max_age=0.2
    )
    assert detections.depth["depth_image"] is image


def test_point_cloud_is_handed_over_as_its_layout():
    cloud = _cloud()
    detections = _Detections(stamp_seconds=10.0)
    detections.set_depth_source(_source(cloud, 10.1), max_age=0.2)
    layout = detections.depth
    assert layout == cloud.buffer_layout()
    assert set(layout) == {
        "data",
        "point_step",
        "row_step",
        "height",
        "width",
        "x_offset",
        "y_offset",
        "z_offset",
    }
    # The raw buffer is passed through, not copied
    assert layout["data"] is cloud.data


def test_image_size_is_seeded_from_the_intrinsics_only_when_unknown():
    detections = _Detections()
    detections.set_depth_source(
        _source(None, 10.0), max_age=0.2, img_size=np.array([640, 480])
    )
    assert detections._img_size.tolist() == [640, 480]
    detections.set_depth_source(
        _source(None, 10.0), max_age=0.2, img_size=np.array([1280, 720])
    )
    assert detections._img_size.tolist() == [640, 480]


def test_planner_attaches_the_depth_source_only_when_configured(monkeypatch):
    from test_planner import _mangle, make_planner_stub

    from kompass.callbacks import DetectionsCallback
    from kompass.components.component import Component
    from kompass.components.defaults import TopicsKeys

    monkeypatch.setattr(Component, "create_all_subscribers", lambda self: None)
    detector = MagicMock()

    def planner_with(depth_source):
        p = make_planner_stub()
        p._inputs_keys = [TopicsKeys.GOAL_POINT]
        goal_callback = MagicMock(spec=DetectionsCallback)
        goal_callback.input_topic = SimpleNamespace(name="/detections")
        callbacks = {
            TopicsKeys.GOAL_POINT: goal_callback,
            TopicsKeys.VISION_DEPTH: depth_source,
        }
        p.get_callback = MagicMock(side_effect=lambda key, idx=0: callbacks.get(key))
        p._depth_image_info = SimpleNamespace(width=640, height=480, frame_id="cam")
        p.config.vision_depth_max_age = 0.2
        setattr(p, _mangle("setup_depth_detector"), lambda: detector)
        return p, goal_callback

    p, goal_callback = planner_with(None)
    p.create_all_subscribers()
    goal_callback.set_depth_detector.assert_called_once_with(detector)
    goal_callback.set_depth_source.assert_not_called()

    depth_source = SimpleNamespace(input_topic=SimpleNamespace(name="/lidar/points"))
    p, goal_callback = planner_with(depth_source)
    p.create_all_subscribers()
    args, kwargs = goal_callback.set_depth_source.call_args
    assert args[0] is depth_source
    assert args[1] == 0.2
    assert kwargs["img_size"].tolist() == [640, 480]
    assert kwargs["camera_frame"] == "cam"


def test_detections_are_lifted_from_a_lidar_cloud_end_to_end():
    """Real objects, no stubs: an embodied-agents Detections2D message with no
    embedded image, a PointCloud2 from a yawed LiDAR mounted away from the
    camera, the kompass callbacks and the core depth detector. The goal must
    come back at the cluster's world position; a stale cloud gives no goal."""
    ea_msgs = pytest.importorskip("automatika_embodied_agents.msg")
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import PointCloud2, PointField

    from kompass.callbacks import DetectionsCallback
    from kompass.components.ros import Topic
    from kompass_core.models import RobotState
    from kompass_core.vision import DepthDetector
    from kompass_cpp.types import SensorConfig
    from ros_sugar.io import PointCloudCallback

    # Camera at the body origin looking ahead (optical pose), fx = fy = 500, VGA
    detector = DepthDetector(
        depth_range=np.array([0.1, 10.0], np.float32),
        camera_in_body_translation=np.zeros(3, np.float32),
        camera_in_body_rotation=np.array([-0.5, 0.5, -0.5, 0.5], np.float32),
        focal_length=np.array([500.0, 500.0], np.float32),
        principal_point=np.array([320.0, 240.0], np.float32),
    )
    # LiDAR at body (-0.1, 0, 0.8), yawed 0.2 rad
    yaw = 0.2
    lidar_position = np.array([-0.1, 0.0, 0.8], np.float32)
    detector.set_point_cloud_sensor(
        SensorConfig(
            position=lidar_position,
            rotation=np.array([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)], np.float32),
        )
    )
    rot = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
        np.float32,
    )
    cluster_center = np.array([3.0, 0.5, 0.4], np.float32)
    offsets = np.array(
        [
            [0.05 * k, 0.04 * i, 0.04 * j]
            for i in range(-5, 6)
            for j in range(-5, 6)
            for k in (-1, 0, 1)
        ],
        np.float32,
    )
    points_lidar = (cluster_center + offsets - lidar_position) @ rot

    # The cloud as a LiDAR driver publishes it: float32 xyz plus 4 padding bytes
    records = np.zeros((len(points_lidar), 4), np.float32)
    records[:, :3] = points_lidar
    cloud_msg = PointCloud2()
    cloud_msg.header.frame_id = "lidar_link"
    cloud_msg.header.stamp = Time(sec=10, nanosec=50_000_000)
    cloud_msg.height, cloud_msg.width = 1, len(records)
    cloud_msg.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (("x", 0), ("y", 4), ("z", 8))
    ]
    cloud_msg.point_step, cloud_msg.row_step = 16, 16 * len(records)
    cloud_msg.data = records.tobytes()
    cloud_callback = PointCloudCallback(
        Topic(name="/lidar/points", msg_type="PointCloud2")
    )
    cloud_callback.callback(cloud_msg)

    # Detections with no embedded image or depth, one box around the cluster's
    # projection (u in [203, 270], v in [140, 207])
    det_msg = ea_msgs.Detections2D()
    det_msg.header.stamp = Time(sec=10, nanosec=0)
    det_msg.labels = ["person"]
    det_msg.boxes = [
        ea_msgs.Bbox2D(
            top_left_x=200.0,
            top_left_y=137.0,
            bottom_right_x=274.0,
            bottom_right_y=210.0,
        )
    ]
    detections = DetectionsCallback(Topic(name="/detections", msg_type="Detections"))
    detections.set_depth_detector(detector)
    detections.set_depth_source(
        cloud_callback,
        max_age=0.2,
        img_size=np.array([640, 480]),
        camera_frame="camera_optical",
    )
    detections.callback(det_msg)

    robot = RobotState(x=1.0, y=-2.0, yaw=0.0, speed=0.0)
    goal = detections.get_output(to_robot_state=True, robot_state=robot)
    assert goal is not None
    assert goal.x == pytest.approx(1.0 + cluster_center[0], abs=0.05)
    assert goal.y == pytest.approx(-2.0 + cluster_center[1], abs=0.05)
    # The labelled path lifts too (it used to drop the robot state)
    labelled = detections.get_output(
        label="person", to_robot_state=True, robot_state=robot
    )
    assert labelled is not None
    assert labelled.x == pytest.approx(goal.x, abs=1e-6)

    # A cloud older than max_age is not paired with the detections
    cloud_msg.header.stamp = Time(sec=9, nanosec=0)
    cloud_callback.callback(cloud_msg)
    assert detections.depth == {}
    assert detections.get_output(to_robot_state=True, robot_state=robot) is None
