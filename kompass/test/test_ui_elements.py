"""Unit tests for the Kompass browser UI elements in ``kompass.ui_elements``.

Rendered into a real Sugarcoat logging card; no node or browser is needed, but
importing the modules requires rclpy/ros_sugar and the browser UI dependencies
(FastHTML/MonsterUI) on the path.
"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fasthtml")
pytest.importorskip("monsterui")
from kompass_interfaces.msg import MissionStatus  # noqa: E402

# The components package must be imported before the other kompass modules, as
# they import each other and only that order resolves
import kompass.components  # noqa: E402, F401
from kompass.ros import UI_EXTENSIONS  # noqa: E402
from fastcore.xml import to_xml  # noqa: E402
from ros_sugar.ui_node import elements  # noqa: E402


def _status(**fields) -> MissionStatus:
    msg = MissionStatus()
    for name, value in fields.items():
        setattr(msg, name, value)
    return msg


def _log(card, msg: MissionStatus):
    return elements.update_logging_card(card, msg, "MissionStatus", "robot")


def _mission_entries(card):
    return [c for c in card.children if getattr(c, "id", None) == "mission-status"]


@pytest.fixture(scope="module", autouse=True)
def registered_elements():
    """Register the elements the way the Launcher and the UI node do: by name"""
    _, outputs = UI_EXTENSIONS["kompass"]()
    elements.add_additional_ui_elements(
        input_elements=[],
        output_elements=[
            (f"{k.__module__}.{k.__qualname__}", f"{e.__module__}.{e.__qualname__}")
            for k, e in outputs.items()
        ],
    )


def test_mission_status_has_an_output_element():
    assert "MissionStatus" in elements._OUTPUT_ELEMENTS


def test_unchanged_status_is_logged_once():
    """The status is republished at the cursor poll rate without changing"""
    card = elements.initial_logging_card()
    for _ in range(5):
        _log(card, _status(mission_id="1", state=MissionStatus.STATE_NAVIGATING))
    assert len(_mission_entries(card)) == 1


def test_unchanged_status_after_other_log_lines_is_not_logged_again():
    card = elements.initial_logging_card()
    _log(card, _status(state=MissionStatus.STATE_IDLE))
    elements.update_logging_card(card, "something else", "String", "robot")
    _log(card, _status(state=MissionStatus.STATE_IDLE))
    assert len(_mission_entries(card)) == 1


def test_changed_status_is_logged():
    card = elements.initial_logging_card()
    _log(card, _status(mission_id="1", total_goals=3, current_goal_idx=0))
    _log(card, _status(mission_id="1", total_goals=3, current_goal_idx=1))
    _log(
        card,
        _status(
            mission_id="1",
            state=MissionStatus.STATE_PAUSED_DWELL,
            total_goals=3,
            current_goal_idx=1,
        ),
    )
    assert len(_mission_entries(card)) == 3


def test_waypoint_is_shown_one_based_while_at_a_waypoint():
    card = elements.initial_logging_card()
    _log(
        card,
        _status(
            state=MissionStatus.STATE_NAVIGATING, total_goals=3, current_goal_idx=1
        ),
    )
    assert "Waypoint 2/3" in to_xml(_mission_entries(card)[-1])


def test_waypoint_is_not_shown_when_returning_to_start():
    card = elements.initial_logging_card()
    _log(card, _status(state=MissionStatus.STATE_RETURNING_TO_START, total_goals=3))
    assert "Waypoint" not in to_xml(_mission_entries(card)[-1])


def test_error_message_is_highlighted():
    card = elements.initial_logging_card()
    _log(
        card,
        _status(
            state=MissionStatus.STATE_IDLE,
            message_level=MissionStatus.LEVEL_ERROR,
            message="Mission 1 ended: timeout",
        ),
    )
    assert "tomorrow-night-red" in to_xml(_mission_entries(card)[-1])


@pytest.mark.parametrize(
    "state, badge",
    [
        (MissionStatus.STATE_COMPLETED, "completed"),
        (MissionStatus.STATE_CANCELED, "canceled"),
        (MissionStatus.STATE_ABORTED, "aborted"),
    ],
)
def test_a_final_state_is_shown_with_its_own_badge(state, badge):
    card = elements.initial_logging_card()
    _log(card, _status(mission_id="1", state=state, total_goals=2, current_goal_idx=1))
    entry = to_xml(_mission_entries(card)[-1])
    assert f"status-badge {badge}" in entry
    # The mission is over, there is no waypoint being worked on
    assert "Waypoint" not in entry
