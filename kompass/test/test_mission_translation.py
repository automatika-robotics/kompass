"""Tests the mission goal to routine translation.

The MissionManager owns no sequencing: a `MultiGoalPlanPath` goal becomes a
routine specification and the Monitor runs it. That translation is where all
the mission's meaning lives, so it is a pure function and tested as one, with
no node and no stack.

What matters is that the step sequence really expresses what was asked: a
waypoint per goal in order, dwells and pauses between them where the goal said
so, and each `ON_TIMEOUT_*` policy mapped onto step policy that produces the
behaviour the action documents.
"""

import pytest
from geometry_msgs.msg import Pose
from kompass_interfaces.action import MultiGoalPlanPath
from kompass_interfaces.msg import PathTrackingError

from kompass.components.mission import (
    dwell_seconds,
    ended_on_pause_timeout,
    mission_routine_spec,
    reached_waypoints,
)

PLANNER_REF = "planner/navigate_to_goal"
STOP_REFS = ["controller/stop_path_tracking", "drive_manager/stop_robot"]
WAYPOINT_TIMEOUT = 120.0


def pose_at(x: float, y: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0
    return pose


def mission_goal(count: int = 2, **fields) -> MultiGoalPlanPath.Goal:
    goal = MultiGoalPlanPath.Goal()
    goal.goals = [pose_at(float(i), 0.0) for i in range(count)]
    goal.algorithm_name = "ompl_rrt"
    goal.end_tolerance = PathTrackingError(
        orientation_error=0.1, lateral_distance_error=0.2
    )
    for name, value in fields.items():
        setattr(goal, name, value)
    return goal


def spec_for(goal, start_pose=None) -> dict:
    return mission_routine_spec(
        goal,
        name="mission_test",
        planner_ref=PLANNER_REF,
        stop_refs=STOP_REFS,
        waypoint_timeout=WAYPOINT_TIMEOUT,
        start_pose=start_pose,
    )


def step_names(spec) -> list:
    return [step["name"] for step in spec["steps"]]


def step_named(spec, name: str) -> dict:
    return next(step for step in spec["steps"] if step["name"] == name)


# ---------------------------------------------------------------------------
# The waypoints themselves
# ---------------------------------------------------------------------------


def test_each_waypoint_becomes_a_goal_on_the_planner():
    spec = spec_for(mission_goal(count=3))
    assert step_names(spec) == ["goto_0", "goto_1", "goto_2"]
    assert all(step["ref"] == PLANNER_REF for step in spec["steps"])


def test_a_waypoints_goal_carries_the_pose_and_the_tolerance():
    """These are the only things the planner is actually told"""
    step = spec_for(mission_goal(count=1))["steps"][0]
    assert step["goal"]["goal"]["position"]["x"] == 0.0
    assert step["goal"]["algorithm_name"] == "ompl_rrt"
    assert step["goal"]["end_tolerance"]["lateral_distance_error"] == pytest.approx(0.2)


def test_step_names_are_distinct():
    """A routine refuses duplicates, since the name identifies it in the cursor"""
    names = step_names(spec_for(mission_goal(count=4, pause_duration=[1.0])))
    assert len(names) == len(set(names))


def test_a_waypoint_that_runs_out_of_time_is_not_retried():
    """Driving somewhere again on a second attempt is rarely what was wanted"""
    step = spec_for(mission_goal(count=1))["steps"][0]
    assert step["timeout"] == WAYPOINT_TIMEOUT
    assert step["on_timeout"] == "fail"
    assert step["on_fail"] == "abort"


def test_a_mission_with_no_waypoints_is_refused():
    with pytest.raises(ValueError, match="at least one"):
        spec_for(mission_goal(count=0))


# ---------------------------------------------------------------------------
# Dwelling
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "durations, index, expected",
    [
        ([], 0, 0.0),
        # One entry is the same dwell everywhere, so a caller need not repeat it
        ([2.0], 0, 2.0),
        ([2.0], 5, 2.0),
        ([1.0, 2.0, 3.0], 1, 2.0),
    ],
)
def test_a_dwell_can_be_given_once_or_per_waypoint(durations, index, expected):
    assert dwell_seconds(durations, index) == expected


def test_a_dwell_becomes_a_step_between_waypoints():
    spec = spec_for(mission_goal(count=2, pause_duration=[3.0]))
    assert [name for name in step_names(spec) if not name.startswith("stop_")] == [
        "goto_0",
        "dwell_0",
        "goto_1",
        "dwell_1",
    ]
    assert step_named(spec, "dwell_0")["kwargs"] == {"duration": 3.0}


def test_a_zero_dwell_adds_no_step():
    """Otherwise every mission would carry steps that do nothing"""
    spec = spec_for(mission_goal(count=2, pause_duration=[0.0]))
    assert step_names(spec) == ["goto_0", "goto_1"]


def test_the_robot_is_stopped_before_holding_position():
    """The planner is done once within tolerance, the controller may still drive"""
    spec = spec_for(mission_goal(count=1, pause_duration=[3.0]))
    assert step_names(spec) == [
        "goto_0",
        "stop_controller_0",
        "stop_drive_manager_0",
        "dwell_0",
    ]
    # Controller first, so no new commands reach the drive manager
    assert [step["ref"] for step in spec["steps"][1:3]] == STOP_REFS


def test_the_robot_is_stopped_once_before_a_dwell_and_a_condition():
    spec = spec_for(
        mission_goal(count=1, pause_duration=[2.0], pause_condition_topic="/go_on")
    )
    assert step_names(spec) == [
        "goto_0",
        "stop_controller_0",
        "stop_drive_manager_0",
        "dwell_0",
        "pause_0",
    ]


def test_a_waypoint_with_no_hold_is_not_stopped_at():
    """The next goal replaces the path, stopping in between would only jerk"""
    spec = spec_for(mission_goal(count=2, pause_duration=[0.0]))
    assert step_names(spec) == ["goto_0", "goto_1"]


def test_a_dwell_list_that_does_not_match_the_waypoints_is_refused():
    with pytest.raises(ValueError, match="pause durations"):
        spec_for(mission_goal(count=3, pause_duration=[1.0, 2.0]))


# ---------------------------------------------------------------------------
# Waiting on a condition, and what a timeout means
# ---------------------------------------------------------------------------


def test_a_pause_condition_becomes_a_step_that_waits_on_the_topic():
    spec = spec_for(mission_goal(count=1, pause_condition_topic="/go_on"))
    pause = step_named(spec, "pause_0")
    assert pause["success"]["topic_name"] == "go_on"


def test_a_dwell_is_served_before_the_condition():
    """The action says so: a dwell then a wait, not the other way round"""
    spec = spec_for(
        mission_goal(count=1, pause_duration=[2.0], pause_condition_topic="/go_on")
    )
    assert step_names(spec)[-2:] == ["dwell_0", "pause_0"]


def test_a_condition_with_no_timeout_waits_indefinitely():
    """Zero or negative means wait, which is a step with no deadline"""
    spec = spec_for(
        mission_goal(count=1, pause_condition_topic="/go_on", condition_timeout=0.0)
    )
    assert "timeout" not in step_named(spec, "pause_0")


def test_continue_makes_an_expired_wait_an_acceptable_outcome():
    spec = spec_for(
        mission_goal(
            count=2,
            pause_condition_topic="/go_on",
            condition_timeout=5.0,
            on_timeout=MultiGoalPlanPath.Goal.ON_TIMEOUT_CONTINUE,
        )
    )
    pause = step_named(spec, "pause_0")
    assert pause["timeout"] == 5.0
    assert pause["on_timeout"] == "succeed"
    # Nothing to unwind, so the mission simply carries on to the next waypoint
    assert "on_abort" not in spec


def test_abort_ends_the_mission_where_it_stands():
    spec = spec_for(
        mission_goal(
            count=2,
            pause_condition_topic="/go_on",
            condition_timeout=5.0,
            on_timeout=MultiGoalPlanPath.Goal.ON_TIMEOUT_ABORT,
        )
    )
    assert step_named(spec, "pause_0")["on_timeout"] == "fail"
    assert step_named(spec, "pause_0")["on_fail"] == "abort"
    assert "on_abort" not in spec


def test_return_to_start_drives_back_to_where_the_mission_began():
    """Not to the first waypoint: the robot may have started somewhere else"""
    start = pose_at(-5.0, -5.0)
    spec = spec_for(
        mission_goal(
            count=2,
            pause_condition_topic="/go_on",
            condition_timeout=5.0,
            on_timeout=MultiGoalPlanPath.Goal.ON_TIMEOUT_RETURN_TO_START,
        ),
        start_pose=start,
    )
    assert step_named(spec, "pause_0")["on_timeout"] == "fail"
    assert spec["on_abort"]["name"] == "return_to_start"
    assert spec["on_abort"]["goal"]["goal"]["position"]["x"] == -5.0


def test_return_to_start_without_a_known_start_is_refused():
    """A wrong pose to drive back to is worse than refusing the mission"""
    with pytest.raises(ValueError, match="started from"):
        spec_for(
            mission_goal(
                count=1,
                pause_condition_topic="/go_on",
                condition_timeout=5.0,
                on_timeout=MultiGoalPlanPath.Goal.ON_TIMEOUT_RETURN_TO_START,
            ),
            start_pose=None,
        )


# ---------------------------------------------------------------------------
# Reading progress back out of the cursor
# ---------------------------------------------------------------------------


def test_reached_waypoints_are_read_from_the_step_names():
    """Not counted: dwells and pauses sit between the waypoints"""
    # Dwelling after the second waypoint: both of the ones before it are done,
    # and the third has not been started
    cursor = {
        "status": "running",
        "index": 3,
        "steps": ["goto_0", "dwell_0", "goto_1", "dwell_1", "goto_2"],
    }
    assert reached_waypoints(cursor, 3) == [True, True, False]


def test_a_completed_mission_reached_every_waypoint():
    cursor = {
        "status": "completed",
        "index": 2,
        "steps": ["goto_0", "goto_1", "goto_2"],
    }
    assert reached_waypoints(cursor, 3) == [True, True, True]


def test_nothing_is_reached_before_the_first_waypoint_ends():
    cursor = {"status": "running", "index": 0, "steps": ["goto_0", "goto_1"]}
    assert reached_waypoints(cursor, 2) == [False, False]


PAUSED_MISSION_STEPS = ["goto_0", "pause_0", "goto_1", "pause_1"]


def test_failing_on_a_pause_is_a_timeout():
    """Read from the step, not the message: the message wording is Sugarcoat's"""
    cursor = {
        "status": "failed",
        "index": 1,
        "steps": PAUSED_MISSION_STEPS,
        "message": "step 'pause_0' failed: Action 'pause_0' did not settle within 5.0 secs",
    }
    assert ended_on_pause_timeout(cursor)


def test_failing_on_a_waypoint_is_not_a_timeout():
    cursor = {"status": "failed", "index": 2, "steps": PAUSED_MISSION_STEPS}
    assert not ended_on_pause_timeout(cursor)


@pytest.mark.parametrize("status", ["aborted", "completed", "running"])
def test_only_a_failed_mission_can_have_timed_out(status):
    """Canceling the mission while it waits on a pause is not a timeout"""
    cursor = {"status": status, "index": 1, "steps": PAUSED_MISSION_STEPS}
    assert not ended_on_pause_timeout(cursor)
