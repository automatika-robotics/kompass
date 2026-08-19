"""Correctness unit tests for the base Component's topic handling.

These cover reconfiguring inputs/outputs after construction. Unlike the
other component test modules, they build a real component: the defect they
guard lives in the relationship between the constructor and the
``callbacks`` / ``publishers_dict`` it builds, so bypassing construction
would bypass the code under test. ``DriveManager`` stands in as a concrete
Component; nothing here is specific to it.

``Component.inputs()`` / ``.outputs()`` used to update the topic lists but
leave ``callbacks`` / ``publishers_dict`` stale (built once in the base
constructor), so post-construction topics were silently ignored (e.g.
``use_plugin`` never seen by the robot-plugin binding) or produced a
misleading "Unknown output" error at activation.
"""

import pytest

pytest.importorskip("rclpy")

from kompass.components import DriveManager, DriveManagerConfig  # noqa: E402
from kompass.components.defaults import TopicsKeys  # noqa: E402
from kompass.ros import Topic  # noqa: E402


def _make_driver() -> DriveManager:
    return DriveManager(
        component_name="reconfig_test_driver",
        config=DriveManagerConfig(disable_safety_stop=True),
    )


def test_outputs_after_init_rebuilds_publishers_dict():
    """The exact silent failure from the report: a default-named topic set
    with use_plugin=True after construction must be the one reachable from
    publishers_dict (that is what the robot-plugin binding iterates)."""
    driver = _make_driver()
    new_topic = Topic(name="/cmd_vel", msg_type="Twist", use_plugin=True)
    driver.outputs(**{"robot_command": new_topic})

    publisher = driver.get_publisher(TopicsKeys.FINAL_COMMAND)
    # Topic normalizes names at construction; compare against ITS name
    assert publisher.output_topic.name == new_topic.name
    assert publisher.output_topic.use_plugin


def test_outputs_after_init_with_renamed_topic():
    """The misleading-error variant from the report: renaming the topic used
    to raise "Unknown output 'robot_command'" at the first get_publisher."""
    driver = _make_driver()
    driver.outputs(
        **{"robot_command": Topic(name="Twist", msg_type="Twist", use_plugin=True)}
    )

    publisher = driver.get_publisher(TopicsKeys.FINAL_COMMAND)
    assert publisher.output_topic.name == "Twist"
    assert "Twist" in driver.publishers_dict
    assert "/cmd_vel" not in driver.publishers_dict


def test_inputs_after_init_rebuilds_callbacks():
    driver = _make_driver()
    new_topic = Topic(name="/my_control", msg_type="Twist")
    driver.inputs(**{"command": new_topic})

    callback = driver.get_callback(TopicsKeys.INTERMEDIATE_CMD)
    assert callback.input_topic.name == new_topic.name
    assert new_topic.name in driver.callbacks
    assert "control" not in driver.callbacks


def test_reconfigure_after_launch_is_refused():
    """A live component must refuse a plain rebuild (it would leak ROS
    entities and drop attached processors)."""
    driver = _make_driver()
    # Simulate a launched component: one publisher wrapper holds a live
    # ROS publisher
    next(iter(driver.publishers_dict.values()))._publisher = object()
    with pytest.raises(RuntimeError, match="after launch"):
        driver.outputs(
            **{"robot_command": Topic(name="/cmd_vel", msg_type="Twist")}
        )
    with pytest.raises(RuntimeError, match="after launch"):
        driver.inputs(**{"command": Topic(name="/my_control", msg_type="Twist")})


def test_get_publisher_unknown_key_names_the_key():
    driver = _make_driver()
    with pytest.raises(KeyError, match="Unknown output key 'not_a_key'"):
        driver.get_publisher("not_a_key")


def test_get_publisher_stale_dict_reports_mismatch_not_unknown_key():
    """If topics and publishers ever disagree again, the error must say so
    instead of blaming the (valid) key."""
    driver = _make_driver()
    # Manufacture the staleness the old code produced
    driver.publishers_dict.pop(driver.out_topic_name(TopicsKeys.FINAL_COMMAND))
    with pytest.raises(KeyError, match="out of sync"):
        driver.get_publisher(TopicsKeys.FINAL_COMMAND)


def test_get_callback_unknown_key_names_the_key():
    driver = _make_driver()
    with pytest.raises(KeyError, match="Unknown input key 'not_a_key'"):
        driver.get_callback("not_a_key")
