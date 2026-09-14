"""Correctness unit tests for the MapServer component.

The map is published once when loaded, instead of on every timer tick, so its
output needs a QoS that delivers it to subscribers joining later. The map
loading tests use the ``object.__new__`` + MagicMock pattern of
``test_planner.py``, the QoS tests build a real component.
"""

from unittest.mock import MagicMock, patch

import pytest

pytest.importorskip("rclpy")

import numpy as np  # noqa: E402
from rclpy import qos  # noqa: E402

from kompass.components import MapServer, MapServerConfig  # noqa: E402
from kompass.components.defaults import TopicsKeys  # noqa: E402
from kompass.ros import Topic  # noqa: E402
from ros_sugar.config import QoSConfig  # noqa: E402


def make_map_server_stub(**config) -> MapServer:
    m = object.__new__(MapServer)
    m.config = MapServerConfig(**config)
    m._pc_msg = None
    m._grid_data = None
    m._grid_res = None
    m._grid_origin = None
    m.get_publisher = MagicMock()
    m.get_logger = MagicMock()
    return m


def load_grid(m: MapServer):
    """Stand-in for reading a YAML map from file"""

    def _read(_path):
        m._grid_data = np.zeros((10, 10), dtype=np.int8)
        m._grid_res = 0.05

    return _read


def map_publish_calls(m: MapServer) -> int:
    return sum(
        1
        for call in m.get_publisher.call_args_list
        if call.args[0] == TopicsKeys.GLOBAL_MAP
    )


class TestMapPublishing:
    def test_map_is_published_once_when_loaded(self):
        m = make_map_server_stub(map_file_path="/maps/office.yaml")
        m._read_map_from_yaml = MagicMock(side_effect=load_grid(m))

        with patch("os.path.isfile", return_value=True):
            assert m.convert_map_from_file() is True

        assert map_publish_calls(m) == 1
        m.get_publisher.return_value.publish.assert_called_once()
        assert m.get_publisher.return_value.publish.call_args.args[0] is m._grid_data

    def test_map_is_not_published_on_timer_ticks(self):
        m = make_map_server_stub(map_file_path="/maps/office.yaml")
        m._read_map_from_yaml = MagicMock(side_effect=load_grid(m))
        with patch("os.path.isfile", return_value=True):
            m.convert_map_from_file()

        for _ in range(5):
            m._execution_step()

        assert map_publish_calls(m) == 1

    def test_map_that_fails_to_load_is_not_published(self):
        m = make_map_server_stub(map_file_path="/maps/missing.yaml")

        with patch("os.path.isfile", return_value=False):
            assert m.convert_map_from_file() is False

        assert map_publish_calls(m) == 0


class TestMapOutputQoS:
    """The map is published once: subscribers joining later still have to get it"""

    @staticmethod
    def _assert_latched(topic: Topic):
        assert topic.qos_profile.durability == qos.DurabilityPolicy.TRANSIENT_LOCAL
        assert topic.qos_profile.reliability == qos.ReliabilityPolicy.RELIABLE

    def test_default_map_output_is_latched(self):
        map_server = MapServer(component_name="map_server_default_qos_test")

        self._assert_latched(map_server.get_out_topic(TopicsKeys.GLOBAL_MAP))

    def test_given_map_output_is_latched(self):
        shared_qos = QoSConfig(reliability=qos.ReliabilityPolicy.BEST_EFFORT)
        map_topic = Topic(
            name="/my_map", msg_type="OccupancyGrid", qos_profile=shared_qos
        )

        map_server = MapServer(
            component_name="map_server_qos_test", outputs={"map": map_topic}
        )

        self._assert_latched(map_server.get_out_topic(TopicsKeys.GLOBAL_MAP))
        # A QoS profile shared with other topics is left unchanged
        assert shared_qos.durability == qos.DurabilityPolicy.VOLATILE
        assert shared_qos.reliability == qos.ReliabilityPolicy.BEST_EFFORT

    def test_map_output_set_after_init_is_latched(self):
        map_server = MapServer(component_name="map_server_qos_after_init_test")

        map_topic = Topic(name="/my_map", msg_type="OccupancyGrid")
        map_server.outputs(map=map_topic)

        assert map_server.get_out_topic(TopicsKeys.GLOBAL_MAP).name == map_topic.name
        self._assert_latched(map_server.get_out_topic(TopicsKeys.GLOBAL_MAP))
