"""Unit tests for the Kompass browser UI elements in ``kompass.ui_elements``.

Rendered into a real Sugarcoat logging card; no node or browser is needed, but
importing the modules requires rclpy/ros_sugar and the browser UI dependencies
(FastHTML/MonsterUI) on the path.
"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("fasthtml")
pytest.importorskip("monsterui")
from geometry_msgs.msg import Pose, PoseStamped  # noqa: E402
from kompass_interfaces.action import MultiGoalPlanPath  # noqa: E402
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
    _, outputs, _tasks = UI_EXTENSIONS["kompass"]()
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


def test_a_paused_mission_shows_the_waypoint_it_is_paused_at():
    card = elements.initial_logging_card()
    _log(
        card,
        _status(state=MissionStatus.STATE_PAUSED, total_goals=3, current_goal_idx=1),
    )
    entry = to_xml(_mission_entries(card)[-1])
    assert ">paused<" in entry
    assert "Waypoint 2/3" in entry


# ---------------------------------------------------------------------------
# The mission's own card, in place of the one an action client gets
# ---------------------------------------------------------------------------

FEEDBACK = MultiGoalPlanPath.Feedback


@pytest.fixture
def mission_card():
    """The card the UI builds for the mission action, registered the way the
    Launcher and the UI node do"""
    _, _, tasks = UI_EXTENSIONS["kompass"]()
    elements.add_additional_ui_elements(
        input_elements=[],
        output_elements=[],
        task_elements=[
            (f"{k.__module__}.{k.__qualname__}", f"{e.__module__}.{e.__qualname__}")
            for k, e in tasks.items()
        ],
    )
    card = elements._TASK_ELEMENTS["MultiGoalPlanPath"]
    return card(name="run_mission", client_type="MultiGoalPlanPath", fields={})


def _feedback(**fields):
    msg = FEEDBACK()
    for name, value in fields.items():
        setattr(msg, name, value)
    return msg


def test_the_mission_action_has_a_card_of_its_own():
    _, _, tasks = UI_EXTENSIONS["kompass"]()
    assert [key.__name__ for key in tasks] == ["MultiGoalPlanPath"]
    assert issubclass(tasks[MultiGoalPlanPath], elements.Task)


def test_the_card_shows_the_journey_waypoint_by_waypoint(mission_card):
    mission_card.update(status="running")
    mission_card.update(
        feedback=_feedback(state=FEEDBACK.STATE_NAVIGATING, current_goal_idx=1, total_goals=3)
    )

    # The journey itself, not the whole card, whose script names the same classes
    journey = to_xml(mission_card._waypoints)
    assert "Waypoint 1" in journey and "Waypoint 3" in journey
    # Reached, being driven to, still to come
    assert journey.count("routine-step done") == 1
    assert journey.count("routine-step active") == 1
    assert journey.count("routine-step pending") == 1
    assert "driving" in journey


def test_the_card_says_what_the_mission_is_doing_at_a_waypoint(mission_card):
    mission_card.update(status="running")
    mission_card.update(
        feedback=_feedback(
            state=FEEDBACK.STATE_PAUSED_DWELL,
            current_goal_idx=0,
            total_goals=2,
            time_paused=3.4,
        )
    )

    card = to_xml(mission_card.card)
    assert "dwelling" in card and "3s" in card


def test_the_log_gets_a_line_rather_than_the_whole_feedback(mission_card):
    mission_card.update(
        status="running",
        feedback=_feedback(
            state=FEEDBACK.STATE_PAUSED_CONDITION, current_goal_idx=1, total_goals=2
        ),
    )

    log = to_xml(mission_card.card)
    assert "Waypoint 2/2: waiting for the go-ahead" in log
    # Not the message as ROS prints it
    assert "current_pose" not in log


def test_the_log_says_nothing_while_nothing_moves_on(mission_card):
    """Feedback arrives several times a second and mostly repeats itself"""
    for _ in range(5):
        mission_card.update(
            status="running",
            feedback=_feedback(
                state=FEEDBACK.STATE_NAVIGATING, current_goal_idx=0, total_goals=2
            ),
        )

    assert len(mission_card._feedback) == 1


def test_the_log_says_a_waypoint_was_reached_and_the_next_one_started(mission_card):
    goal = Pose()
    goal.position.x, goal.position.y = 2.0, -1.0
    mission_card.update(
        status="running",
        feedback=_feedback(
            state=FEEDBACK.STATE_NAVIGATING, current_goal_idx=0, total_goals=2
        ),
    )
    mission_card.update(
        feedback=_feedback(
            state=FEEDBACK.STATE_NAVIGATING,
            current_goal_idx=1,
            total_goals=2,
            current_goal=goal,
        )
    )

    assert mission_card._feedback[-2] == "Reached waypoint 1/2"
    # Where it is driving to, not only that it is driving
    assert mission_card._feedback[-1] == "Waypoint 2/2: driving to x 2.00, y -1.00"


def test_the_log_says_when_the_mission_is_over(mission_card):
    mission_card.update(status="running")
    mission_card.update(status="completed")

    assert mission_card._feedback[-1] == "Mission completed"


def test_the_journey_shows_how_far_the_waypoint_still_is(mission_card):
    goal, here = Pose(), PoseStamped()
    goal.position.x = 3.0
    here.header.frame_id = "map"
    here.pose.position.x = 0.5
    mission_card.update(status="running")
    mission_card.update(
        feedback=_feedback(
            state=FEEDBACK.STATE_NAVIGATING,
            total_goals=1,
            current_goal=goal,
            current_pose=here,
        )
    )

    assert "2.5 m to go" in to_xml(mission_card._waypoints)


def test_the_journey_leaves_out_a_distance_it_cannot_know(mission_card):
    """A location the mission could not fill in carries no frame"""
    goal = Pose()
    goal.position.x = 3.0
    mission_card.update(status="running")
    mission_card.update(
        feedback=_feedback(
            state=FEEDBACK.STATE_NAVIGATING, total_goals=1, current_goal=goal
        )
    )

    assert "m to go" not in to_xml(mission_card._waypoints)


def test_the_card_says_when_no_mission_is_running(mission_card):
    assert "No mission running" in to_xml(mission_card.card)


def test_the_card_offers_pause_and_cancel_while_a_mission_runs(mission_card):
    mission_card.update(status="running")
    mission_card.update(feedback=_feedback(state=FEEDBACK.STATE_NAVIGATING, total_goals=1))

    card = to_xml(mission_card.card)
    assert "missionControl('pause_mission')" in card
    assert "missionCancel()" in card


def test_the_card_offers_resume_while_a_mission_is_paused(mission_card):
    mission_card.update(status="running")
    mission_card.update(feedback=_feedback(state=FEEDBACK.STATE_PAUSED, total_goals=1))

    card = to_xml(mission_card.card)
    assert "missionControl('resume_mission')" in card
    assert "pause_mission" not in card.split("missionCancel()")[0]


def test_the_card_has_nothing_to_pause_with_no_mission(mission_card):
    card = to_xml(mission_card.card)
    assert "missionControl(" not in card.split("<script>")[0]


def test_the_card_puts_the_next_mission_together(mission_card):
    """Waypoints picked on the map or typed in, and the dwell at each"""
    card = to_xml(mission_card.card)

    assert "missionAddWaypoint()" in card and "missionClearWaypoints()" in card
    assert "Send mission" in card
    assert 'id="run_mission-waypoint-list"' in card
    assert 'id="run_mission-dwell"' in card
    # The script is told which action to send to, and which card it draws into
    assert '"/api/actions/" + action' in card
    assert 'const action = "run_mission"' in card


def test_the_mission_settings_say_what_they_are(mission_card):
    """A box that starts filled in, like the dwell, hides its placeholder"""
    card = to_xml(mission_card.card)

    for label in (
        "Dwell at each waypoint (s)",
        "Wait at each for (topic)",
        "Arrival tolerance",
        "Distance (m)",
        "Heading (rad)",
    ):
        assert label in card
    for field in ("dwell", "go-ahead", "distance", "heading"):
        assert f'id="run_mission-{field}"' in card


def test_the_goal_the_card_sends_is_one_the_action_takes():
    """The card builds the goal in the browser: this is the shape it posts"""
    from ros_sugar.io.supported_types import set_ros_msg_from_dict, validate_msg_fields

    body = {
        "goals": [
            {
                "position": {"x": 1.0, "y": 2.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        ],
        "frame_id": "map",
        "pause_duration": [5.0],
        "end_tolerance": {"lateral_distance_error": 0.15, "orientation_error": 0.2},
        "pause_condition_topic": "/mission_go_on",
    }

    validate_msg_fields(MultiGoalPlanPath.Goal, body, "the mission goal")
    goal = set_ros_msg_from_dict(MultiGoalPlanPath.Goal, body)

    assert [(p.position.x, p.position.y) for p in goal.goals] == [(1.0, 2.0)]
    assert list(goal.pause_duration) == [5.0]
    assert goal.pause_condition_topic == "/mission_go_on"


def test_the_log_never_repeats_the_line_it_just_wrote(mission_card):
    """Coming back to the same waypoint and state, as resuming does, would
    otherwise write the line again"""
    driving = _feedback(
        state=FEEDBACK.STATE_NAVIGATING, current_goal_idx=0, total_goals=2
    )
    paused = _feedback(state=FEEDBACK.STATE_PAUSED, current_goal_idx=0, total_goals=2)
    mission_card.update(status="running", feedback=driving)
    mission_card.update(feedback=paused)
    mission_card.update(feedback=driving)
    mission_card.update(feedback=driving)

    assert mission_card._feedback == [
        "Waypoint 1/2: driving to x 0.00, y 0.00",
        "Waypoint 1/2: paused",
        "Waypoint 1/2: driving to x 0.00, y 0.00",
    ]


def test_a_paused_mission_does_not_say_it_is_running(mission_card):
    """The goal of a paused mission is still running, the mission is not"""
    mission_card.update(status="running")
    mission_card.update(feedback=_feedback(state=FEEDBACK.STATE_PAUSED, total_goals=1))

    badge = to_xml(mission_card._badge)
    assert "status-badge paused" in badge
    assert "running" not in badge


def test_a_mission_cancelled_while_paused_is_cancelled_not_paused(mission_card):
    """The last thing the mission was doing is not what became of it"""
    mission_card.update(status="running")
    mission_card.update(feedback=_feedback(state=FEEDBACK.STATE_PAUSED, total_goals=1))
    mission_card.update(status="canceled")

    badge = to_xml(mission_card._badge)
    assert "status-badge canceled" in badge
    assert "paused" not in badge
    assert mission_card._controls is None
