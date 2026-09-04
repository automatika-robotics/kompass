"""Unit tests for selecting detections by label on ``DetectionsCallback``:
the box handed to the vision follower for a label must be that label's own
box, whatever else the message contains and in whatever order.

Exercised with real embodied-agents messages through the real callback;
importing the modules requires rclpy/ros_sugar on the path.
"""

import pytest

pytest.importorskip("rclpy")
ea_msgs = pytest.importorskip("automatika_embodied_agents.msg")

import kompass.components  # noqa: E402, F401  (resolves the package import order)
from kompass.callbacks import DetectionsCallback  # noqa: E402
from kompass.components.ros import Topic  # noqa: E402


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


def _callback():
    return DetectionsCallback(
        Topic(name="/detections", msg_type="Detections"), node_name="test"
    )


def test_label_query_returns_that_labels_own_box():
    """Regression: the M20 bottle test (2026-09-03). With the bottle third of
    six detections the follower was handed the geometry of the last box in
    the message (a tv monitor at the far wall) and lifted it to 4 m."""
    callback = _callback()
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


def test_label_query_returns_every_instance_in_message_order():
    callback = _callback()
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


def test_label_absent_or_no_recent_detections_gives_none():
    callback = _callback()
    callback.callback(_message(("chair", _box(468, 267, 534, 390))))
    assert callback.get_output(label="bottle") is None
    # An empty frame after the detections means there is no current target
    callback.callback(_message())
    assert callback.get_output(label="chair") is None


def test_unlabelled_query_still_returns_all_boxes():
    callback = _callback()
    callback.callback(
        _message(
            ("person", _box(131, 70, 269, 505)),
            ("bottle", _box(386, 438, 409, 538)),
        )
    )
    assert [b.label for b in callback.get_output()] == ["person", "bottle"]
