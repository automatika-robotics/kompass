"""Browser UI elements for Kompass types (augment Sugarcoat UI elements)"""

from fasthtml.common import Div, Span, Strong
from kompass_interfaces.msg import MissionStatus as ROSMissionStatus
from ros_sugar.ui_node.elements import DEFAULT_STYLE, LOG_STYLES

from .data_types import MissionStatus

# Mission state -> (label, status-badge class from the Sugarcoat UI stylesheet)
_MISSION_STATES = {
    ROSMissionStatus.STATE_NAVIGATING: ("navigating", "running"),
    ROSMissionStatus.STATE_PAUSED_DWELL: ("dwelling", "accepted"),
    ROSMissionStatus.STATE_PAUSED_CONDITION: ("waiting", "canceled"),
    ROSMissionStatus.STATE_RETURNING_TO_START: ("returning to start", "active"),
    ROSMissionStatus.STATE_IDLE: ("idle", "inactive"),
}

# States in which current_goal_idx points at a waypoint
_WAYPOINT_STATES = (
    ROSMissionStatus.STATE_NAVIGATING,
    ROSMissionStatus.STATE_PAUSED_DWELL,
    ROSMissionStatus.STATE_PAUSED_CONDITION,
)

_MISSION_STATUS_ID = "mission-status"


def _log_mission_status_element(
    logging_card, output: ROSMissionStatus, data_src: str
):
    """Render MissionStatus output in the logging card.

    The status is republished at the mission's cursor poll rate whether or not
    it changed, so an entry is only added when it differs from the latest
    mission status entry in the card.
    """
    signature = "|".join(
        str(field)
        for field in (
            output.mission_id,
            output.state,
            output.total_goals,
            output.current_goal_idx,
            output.message_level,
            output.message,
        )
    )
    for child in reversed(logging_card.children):
        if getattr(child, "id", None) == _MISSION_STATUS_ID:
            if child.get("data_signature") == signature:
                return logging_card
            break

    style = LOG_STYLES.get(data_src, DEFAULT_STYLE)
    label, badge = _MISSION_STATES.get(output.state, ("unknown", "unknown"))
    entry = Div(
        Strong(f"{style['prefix']} ", cls=style["cls"]),
        Span(label, cls=f"status-badge {badge}"),
        cls="whitespace-pre-wrap ml-2 p-2 flex flex-wrap items-center gap-2",
        id=_MISSION_STATUS_ID,
        data_signature=signature,
    )
    if output.state in _WAYPOINT_STATES and output.total_goals:
        # current_goal_idx is the index of the waypoint in the mission goals
        entry(
            Span(
                f"Waypoint {output.current_goal_idx + 1}/{output.total_goals}",
                cls="font-bold",
            )
        )
    if output.message:
        entry(
            Span(
                output.message,
                cls="tomorrow-night-red"
                if output.message_level == ROSMissionStatus.LEVEL_ERROR
                else "",
            )
        )
    return logging_card(entry)


OUTPUT_ELEMENTS = {
    MissionStatus: _log_mission_status_element,
}

INPUT_ELEMENTS = {}
