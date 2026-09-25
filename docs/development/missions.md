---
title: "Missions: Sequencing Goals as Routines"
---

# Missions: Sequencing Goals as Routines

The `MissionManager` runs a sequence of navigation goals as one ROS 2 action. It owns **no sequencing of its own**: a multi-waypoint mission is a sequence with per-step policy -- drive somewhere, wait, drive somewhere else, and decide what a failure or a timeout means -- which is exactly what a Sugarcoat `Routine` is. So the component translates a `MultiGoalPlanPath` goal into a routine specification, registers it on the Monitor over the runtime API, and reports the routine's cursor back as action feedback.

That split is the thing to keep in mind when extending it: retries, timeouts, preemption and step policy live in Sugarcoat's routine engine, while the component owns the translation, the progress reporting and the robot-specific parts (frames, stopping, arrival tolerance).

Read [Creating a Custom Component](./custom_component.md) first for component basics, and [Advanced: Health Status, Fallbacks & Events](./advanced_component.md) for component actions and events.

## Wiring a MissionManager

```python
from kompass.components import (
    Controller, DriveManager, MissionManager, MissionManagerConfig, Planner,
)

planner = Planner(component_name="planner")
planner.run_type = "ActionServer"          # the mission drives waypoints through it
controller = Controller(component_name="controller")
driver = DriveManager(component_name="drive_manager")

mission = MissionManager(
    component_name="mission",
    planner=planner,
    controller=controller,
    drive_manager=driver,
    config=MissionManagerConfig(waypoint_timeout=180.0, cursor_poll_rate=5.0),
)
```

The three components are passed as objects but **only their names are kept** in the config, so the mission survives being serialized and launched in its own process. Passing a planner that is not an `ActionServer`, or a wrong type for the controller or drive manager, raises immediately. A mission built from a config file instead can set the same three fields directly:

```toml
[mission]
planner_action = "planner/navigate_to_goal"
controller_name = "controller"
drive_manager_name = "drive_manager"
```

They are checked again in `custom_on_configure()`, since a config file is only applied at configure time.

| Property | Value |
|---|---|
| Run type | `ActionServer` only -- a mission is one long goal, not a loop |
| Action | `run_mission`, type `kompass_interfaces/action/MultiGoalPlanPath` |
| Input | `TopicsKeys.ROBOT_LOCATION` -- `Odometry`, `PoseStamped` or `Pose` |
| Output | `TopicsKeys.MISSION_STATUS` -- `kompass_interfaces/msg/MissionStatus` |

The location input is optional in practice: it is read for the return-to-start policy, for `current_pose` in the feedback, and for the end displacement in the result.

## The goal contract

| Field | Meaning |
|---|---|
| `goals` | Waypoints as `geometry_msgs/Pose`, visited in order |
| `frame_id` | Frame the waypoints are in. Empty means the configured world frame |
| `algorithm_name` | Planning algorithm, empty for the planner's configured default |
| `end_tolerance` | Arrival tolerance (`PathTrackingError`), applied at every waypoint |
| `pause_duration` | Seconds to dwell at a reached waypoint: empty for none, one entry for all of them, or one per waypoint |
| `pause_condition_topic` | `std_msgs/Bool` topic to wait on at a reached waypoint before continuing. Named per mission, not configured on the component |
| `condition_timeout` | Seconds to wait for that condition. Zero or negative waits indefinitely |
| `on_timeout` | `ON_TIMEOUT_CONTINUE` (0), `ON_TIMEOUT_RETURN_TO_START` (1) or `ON_TIMEOUT_ABORT` (2) |

Waypoints given in another frame are transformed into the world frame before the routine is built, waiting for the transform for up to `topic_subscription_timeout` (the component config field used for topics). A transform that never arrives refuses the mission rather than driving to an unconverted pose.

```bash
ros2 action send_goal /run_mission kompass_interfaces/action/MultiGoalPlanPath \
  "{goals: [{position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}},
            {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}],
    end_tolerance: {orientation_error: 0.2, lateral_distance_error: 0.15},
    pause_duration: [5.0]}" --feedback
```

## From goal to routine

`mission_routine_spec()` is a pure function: a goal in, a routine specification out. It is where every mission policy is decided, and it can be called and asserted on without a ROS graph (see [Testing](#testing)).

| Step | Ref | Purpose |
|---|---|---|
| `goto_<i>` | `<planner>/<action>` | Send waypoint `i` to the planner's action server. `timeout` is `waypoint_timeout`, `on_timeout: fail`, `on_fail: abort` |
| `stop_<component>_<i>` | `<controller>/stop_path_tracking` | Stop following the path, named after the component it calls. Runs before a wait, since the planner is done once within tolerance while the controller may still be driving |
| `stop_<component>_<i>` | `<drive_manager>/stop_robot` | Stop the robot in closed loop. Both stops carry `max_retries: retries` |
| `dwell_<i>` | `monitor/wait` | Hold for `duration` seconds |
| `pause_<i>` | `monitor/wait` | Hold with `duration: 0.0` and a success condition on the goal's condition topic, which is what keeps the step open |

Two waypoints with a five second dwell produce:

```json
["goto_0", "stop_controller_0", "stop_drive_manager_0", "dwell_0",
 "goto_1", "stop_controller_1", "stop_drive_manager_1", "dwell_1"]
```

and a conditional pause adds a step whose success condition carries its own topic as plain data, so there is nothing to declare in advance:

```json
{
  "ref": "monitor/wait",
  "name": "pause_0",
  "kwargs": {"duration": 0.0},
  "success": {"type": "simple", "topic_name": "mission_go_on",
              "topic_msg_type": "Bool", "attribute_path": ["data"],
              "operator": "equals", "ref_value": true},
  "on_fail": "abort",
  "timeout": 60.0,
  "on_timeout": "fail"
}
```

The rules the translation applies:

- **Stops are only inserted before a wait.** A waypoint with no dwell and no condition goes straight on to the next `goto_`, which replaces the plan being driven anyway.
- **No condition at the last waypoint.** There is nothing to continue to, so waiting there would only hold up success. A dwell at the last waypoint still runs.
- **The timeout policy is the same at every waypoint.** `CONTINUE` makes a wait running out an acceptable outcome (`on_timeout: succeed`), the other two end the mission (`on_timeout: fail`). A non-positive `condition_timeout` adds no timeout at all.
- **`on_pause` holds position.** Pausing preempts the step in flight, which cancels a waypoint's planner goal but leaves the robot rolling, so the spec's `on_pause` is the same stop steps, named `stop_<component>_on_pause`.
- **`on_abort` returns to start**, when the policy asks for it and a start pose is known: a `goto_` step named `return_to_start`, which the routine runs when it aborts -- a failed waypoint, a cancellation, or a condition that ran out under an ending policy.

:::{admonition} Return-to-start is not followed to the end yet
:class: caution

A routine publishes its terminal status *before* dispatching `on_abort`, and the mission ends its goal on that status and then removes the routine, which halts whatever the routine still had in flight. So the drive back is started and then preempted rather than driven to completion. Making `ON_TIMEOUT_RETURN_TO_START` work end to end means following the routine through its terminal action before reporting the outcome.
:::

A goal that describes no mission, or one that cannot be carried out as asked, raises `ValueError` and the mission is refused before any routine is registered: no waypoints, a `pause_duration` list that is neither empty, one entry, nor one per waypoint, an `on_timeout` that is no policy, or a return-to-start policy with no known start pose.

## Following a mission

Three views of the same run, all driven by the routine's cursor, which is polled at `cursor_poll_rate`:

1. **Action feedback** on `run_mission`, one message per poll.
2. **`mission_status`**, the same progress for anything that did not send the goal. Published transient local by default, so a late subscriber still gets the latest status.
3. **`routine/<routine_name>/state`**, the cursor itself, published by the Monitor.

The state is read from the name of the step in flight:

| Step in flight | Feedback / status state |
|---|---|
| `goto_<i>` | `STATE_NAVIGATING` (0) |
| `dwell_<i>` | `STATE_PAUSED_DWELL` (1) |
| `pause_<i>` | `STATE_PAUSED_CONDITION` (2) |
| `return_to_start` | `STATE_RETURNING_TO_START` (3) |
| any, while the routine is paused | `STATE_PAUSED` (8) |

Feedback also carries `current_goal_idx` and `current_goal` (taken from the step's index suffix), `current_pose` from the location input, and `time_paused`, measured from the first poll that saw the current pause -- it can therefore be short by up to one poll period, and pausing the mission during a dwell starts a new count.

`MissionStatus` adds the states a feedback message cannot have, because they are only true once the goal is over: `STATE_IDLE` (4) before the first mission, and `STATE_COMPLETED` (5), `STATE_CANCELED` (6) and `STATE_ABORTED` (7). The final status is published **once, before the cleanup that follows the mission**, and stays the last status until the next mission starts, so a late subscriber still learns how this one ended. Every status also carries the latest message from the component and its level (`LEVEL_INFO` / `LEVEL_ERROR`).

The result says how far the mission got:

| Field | Meaning |
|---|---|
| `outcome` | `OUTCOME_COMPLETED`, `OUTCOME_CANCELED`, `OUTCOME_TIMED_OUT` (a condition ran out under an ending policy), `OUTCOME_FAILED` |
| `reached_waypoints` | One flag per goal, read from the `goto_` steps the cursor got past, not counted |
| `last_reached_index` | `-1` if none were reached |
| `end_displacement` | Distance and heading error between the robot and the last waypoint reached, measured by the component -- the routine does not carry the planner's own results back |

## Pausing and resuming

`pause_mission` and `resume_mission` are [component actions](./advanced_component.md#built-in-component-actions), so they can be called as event actions in a recipe, over the component's `execute_method` service, or from a UI:

```bash
ros2 service call /mission/execute_method automatika_ros_sugar/srv/ExecuteMethod \
  "{name: 'pause_mission'}"
```

Pausing preempts the step in flight and runs the spec's `on_pause` stops, so the robot holds position where it is. On resuming, the step starts over: a waypoint that was being driven to is driven to again, a dwell restarts, and a conditional pause waits for a new go-ahead. Both refuse when there is no ongoing mission, and both report back what the Monitor answered.

## Ending a mission

A mission ends by itself, by cancellation, or because the node asked it to:

```bash
ros2 service call /mission/cancel_main_action std_srvs/srv/Trigger   # without the goal handle
```

Either way the routine is aborted, which cancels the planner goal in flight. That is what stops the robot: a canceled or aborted planner goal publishes an **empty global plan**, and the controller stops tracking when it receives a plan with fewer than two poses. The `stop_` steps cover the other direction, where a waypoint was reached normally and the robot is still rolling towards it.

A mission also ends itself when the Monitor stops answering: a routine whose cursor cannot be read `retries` times over in a row is not being followed by anything and may not even be running, so the mission aborts it and reports the failure. Without that it would poll forever, holding the one goal the action server takes and leaving every mission after it to be rejected. Each call is bounded by a share of `end_mission_timeout`, both waiting for the service and waiting for its answer, which is what keeps a Monitor that has gone away from costing a minute a poll -- and what lets a mission end within the time a deactivation waits for it.

Deactivating the component ends the ongoing mission first, waiting up to `end_mission_timeout` for it: destroying the action server takes the goal handle with it, so a client would never get a result and the routine would carry on with nothing following it. Whatever happens, the routine belongs to the goal and is removed with it (forced, since an abort mid-step leaves it running), and a leftover routine from a mission that could not clean up is replaced rather than blocking the next mission.

## Configuration

`MissionManagerConfig` extends `ComponentConfig`, so the usual fields (`frames`, `loop_rate`, `topic_subscription_timeout`, ...) apply as well.

| Parameter | Default | Purpose |
|---|---|---|
| `waypoint_timeout` | `300.0` | Seconds allowed for one waypoint before the mission gives up on it |
| `cursor_poll_rate` | `5.0` | How often the cursor is read, in Hz. Only affects how promptly feedback is published |
| `retries` | `2` | Extra attempts at what a mission can retry -- stopping the robot, reading the routine's cursor -- and running out of them on any of it ends the mission |
| `end_mission_timeout` | `10.0` | Seconds a deactivation waits for the ongoing mission to end, and the budget one runtime API call gets a share of |
| `ui_waypoints_topic` | `"/mission_waypoints"` | Where the UI publishes a waypoint picked on the map |
| `routine_name` | `"navigation_mission"` | Name of the routine carrying out a mission, the same for every one |
| `planner_action` | `None` | The planner's action server as `<component>/<action>`, filled in from the planner |
| `controller_name` | `None` | Filled in from the controller |
| `drive_manager_name` | `None` | Filled in from the drive manager |

Missions run one at a time, which is why one `routine_name` serves them all: the name is known before any mission starts, which is what lets a UI follow it.

## Driving it from a UI

The component gathers everything its browser card needs, so a recipe stays short:

```python
launcher.enable_ui(
    inputs=[*mission.ui_inputs],    # the action, the execute_method service, the waypoints topic
    outputs=[*mission.ui_outputs],  # mission_status and the same waypoints topic
    serve_browser=True,
)
```

Kompass registers its own card for the `MultiGoalPlanPath` action, which sends the goal, follows the journey and pauses or resumes it in one place. See [Extending the Browser UI](./ui_elements.md).

## Testing

| What | Where |
|---|---|
| The translation, without a ROS graph: step order, dwell rules, condition placement, timeout policies, refusals | `kompass/test/test_mission_translation.py` |
| A mission on a running stack: feedback, status, cancellation, deactivation, pausing | `kompass/test/mission_launch_test.py` |

`mission_routine_spec()`, `dwell_seconds()`, `reached_waypoints()` and `ended_on_pause_timeout()` are module-level functions for exactly this reason -- every mission policy can be asserted on as data.
