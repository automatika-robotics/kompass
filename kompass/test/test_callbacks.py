"""Correctness unit tests for the message conversions in ``kompass.callbacks``.

Exercised on real ROS messages through the real callbacks; no node or
executor is needed, but importing the modules requires rclpy/ros_sugar on
the path. The detection tests additionally need the embodied-agents
messages and are skipped without them.
"""

import pytest

pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry  # noqa: E402

try:
    from automatika_embodied_agents import msg as ea_msgs
except ImportError:  # pragma: no cover - depends on the installed workspace
    ea_msgs = None

# The components package must be imported before the callbacks module, as the
# two import each other and only that order resolves
import kompass.components  # noqa: E402, F401
from kompass.callbacks import DetectionsCallback, OdomCallback  # noqa: E402
from kompass.ros import Topic  # noqa: E402

needs_ea_msgs = pytest.mark.skipif(
    ea_msgs is None, reason="embodied-agents messages are not on the path"
)


# --- OdomCallback: Odometry -> RobotState -----------------------------------


def test_odom_speed_is_the_planar_speed():
    """The state's speed is the norm of both linear components.

    Regression: the speed used to be computed from the lateral component
    twice, so a differential-drive robot moving straight ahead reported a
    speed of zero to every controller that reads it.
    """
    callback = OdomCallback(
        input_topic=Topic(name="/odom", msg_type="Odometry"), node_name="odom_test"
    )
    msg = Odometry()
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = 0.3
    msg.twist.twist.linear.y = 0.4

    state = callback._process(msg)

    assert state.vx == pytest.approx(0.3)
    assert state.vy == pytest.approx(0.4)
    assert state.speed == pytest.approx(0.5)


# --- DetectionsCallback: selecting detections by label ----------------------
# The box handed to the vision follower for a label must be that label's own
# box, whatever else the message contains and in whatever order.


def _box(x0, y0, x1, y1):
    return ea_msgs.Bbox2D(
        top_left_x=float(x0),
        top_left_y=float(y0),
        bottom_right_x=float(x1),
        bottom_right_y=float(y1),
    )


def _message(*labelled_boxes):
    msg = ea_msgs.Detections2D()
    msg.labels = [label for label, _ in labelled_boxes]
    msg.boxes = [box for _, box in labelled_boxes]
    return msg


def _corners(box):
    """(x0, y0, x1, y1) of a core Bbox2D"""
    x0, y0 = (int(v) for v in box.top_left_corner)
    w, h = (int(v) for v in box.size)
    return (x0, y0, x0 + w, y0 + h)


def _detections_callback():
    return DetectionsCallback(
        Topic(name="/detections", msg_type="Detections"), node_name="test"
    )


@needs_ea_msgs
def test_label_query_returns_that_labels_own_box():
    """Regression: the M20 bottle test (2026-09-03). With the bottle third of
    six detections the follower was handed the geometry of the last box in
    the message (a tv monitor at the far wall) and lifted it to 4 m."""
    callback = _detections_callback()
    callback.callback(
        _message(
            ("person", _box(131, 70, 269, 505)),
            ("chair", _box(468, 267, 534, 390)),
            ("bottle", _box(386, 438, 409, 538)),
            ("chair", _box(594, 299, 670, 404)),
            ("tvmonitor", _box(633, 199, 742, 277)),
        )
    )
    boxes = callback.get_output(label="bottle")
    assert [_corners(b) for b in boxes] == [(386, 438, 409, 538)]
    assert boxes[0].label == "bottle"


@needs_ea_msgs
def test_label_query_returns_every_instance_in_message_order():
    callback = _detections_callback()
    callback.callback(
        _message(
            ("bottle", _box(386, 438, 409, 538)),
            ("chair", _box(468, 267, 534, 390)),
            ("bottle", _box(567, 246, 580, 270)),
        )
    )
    boxes = callback.get_output(label="bottle")
    assert [_corners(b) for b in boxes] == [
        (386, 438, 409, 538),
        (567, 246, 580, 270),
    ]


@needs_ea_msgs
def test_label_absent_or_no_recent_detections_gives_none():
    callback = _detections_callback()
    callback.callback(_message(("chair", _box(468, 267, 534, 390))))
    assert callback.get_output(label="bottle") is None
    # An empty frame after the detections means there is no current target
    callback.callback(_message())
    assert callback.get_output(label="chair") is None


@needs_ea_msgs
def test_unlabelled_query_still_returns_all_boxes():
    callback = _detections_callback()
    callback.callback(
        _message(
            ("person", _box(131, 70, 269, 505)),
            ("bottle", _box(386, 438, 409, 538)),
        )
    )
    assert [b.label for b in callback.get_output()] == ["person", "bottle"]
