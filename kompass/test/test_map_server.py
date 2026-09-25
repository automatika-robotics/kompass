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
from kompass.robot import (  # noqa: E402
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotConfig,
    RobotGeometryType,
    RobotType,
)
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

    def test_the_map_is_loaded_again_after_a_restart(self):
        """`init_variables` drops the loaded map on every activation, so the
        load `_execute_once` does has to be repeatable: a restarted MapServer
        that never loads again publishes no map at all"""
        m = make_map_server_stub(
            map_file_path="/maps/office.yaml",
            # init_variables() sizes the robot, so the stub needs one
            robot=RobotConfig(
                model_type=RobotType.DIFFERENTIAL_DRIVE,
                geometry_type=RobotGeometryType.CYLINDER,
                geometry_params=[0.2, 0.4],
                ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
                ctrl_omega_limits=AngularCtrlLimits(
                    max_omega=0.4, max_acc=2.0, max_decel=2.0, max_ang=1.0
                ),
            ),
        )
        m._read_map_from_yaml = MagicMock(side_effect=load_grid(m))

        with patch("os.path.isfile", return_value=True):
            # An activation: state reset, then the load on the first tick
            m.init_variables()
            m._execute_once()
            # A deactivation and a new activation
            m.init_variables()
            assert m._grid_data is None
            m._execute_once()

        assert map_publish_calls(m) == 2

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

    def test_a_map_output_given_without_a_qos_is_latched(self):
        shared_qos = QoSConfig()
        map_topic = Topic(
            name="/my_map", msg_type="OccupancyGrid", qos_profile=shared_qos
        )

        map_server = MapServer(
            component_name="map_server_qos_test", outputs={"map": map_topic}
        )

        self._assert_latched(map_server.get_out_topic(TopicsKeys.GLOBAL_MAP))
        # A QoS profile shared with other topics is left unchanged
        assert shared_qos.durability == qos.DurabilityPolicy.SYSTEM_DEFAULT

    def test_a_qos_set_in_the_recipe_is_kept(self):
        """Latching is a default: only what the recipe left alone is filled in"""
        map_topic = Topic(
            name="/my_map",
            msg_type="OccupancyGrid",
            qos_profile=QoSConfig(reliability=qos.ReliabilityPolicy.BEST_EFFORT),
        )

        map_server = MapServer(
            component_name="map_server_qos_kept_test", outputs={"map": map_topic}
        )

        profile = map_server.get_out_topic(TopicsKeys.GLOBAL_MAP).qos_profile
        assert profile.reliability == qos.ReliabilityPolicy.BEST_EFFORT
        # The recipe said nothing about durability, so it is still latched
        assert profile.durability == qos.DurabilityPolicy.TRANSIENT_LOCAL

    def test_map_output_set_after_init_is_latched(self):
        map_server = MapServer(component_name="map_server_qos_after_init_test")

        map_topic = Topic(name="/my_map", msg_type="OccupancyGrid")
        map_server.outputs(map=map_topic)

        assert map_server.get_out_topic(TopicsKeys.GLOBAL_MAP).name == map_topic.name
        self._assert_latched(map_server.get_out_topic(TopicsKeys.GLOBAL_MAP))
