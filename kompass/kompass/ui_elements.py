"""Browser UI elements for Kompass types (augment Sugarcoat UI elements)"""

import math

from typing import List, Optional

from fasthtml.common import Button, Div, Input, P, Script, Span, Strong
from kompass_interfaces.action import MultiGoalPlanPath as ROSMultiGoalPlanPath
from kompass_interfaces.msg import MissionStatus as ROSMissionStatus
from monsterui.all import Card, DivHStacked, DivVStacked, Grid, H6, LabelInput
from ros_sugar.ui_node.elements import DEFAULT_STYLE, LOG_STYLES, Task

from .data_types import MissionStatus

_FEEDBACK = ROSMultiGoalPlanPath.Feedback

# Mission state -> (label, status-badge class from the Sugarcoat UI stylesheet)
_MISSION_STATES = {
    ROSMissionStatus.STATE_NAVIGATING: ("navigating", "running"),
    ROSMissionStatus.STATE_PAUSED_DWELL: ("dwelling", "accepted"),
    ROSMissionStatus.STATE_PAUSED_CONDITION: ("waiting", "canceled"),
    ROSMissionStatus.STATE_RETURNING_TO_START: ("returning to start", "active"),
    ROSMissionStatus.STATE_IDLE: ("idle", "inactive"),
    ROSMissionStatus.STATE_COMPLETED: ("completed", "completed"),
    ROSMissionStatus.STATE_CANCELED: ("canceled", "canceled"),
    ROSMissionStatus.STATE_ABORTED: ("aborted", "aborted"),
    ROSMissionStatus.STATE_PAUSED: ("paused", "inactive"),
}

# States in which current_goal_idx points at a waypoint
_WAYPOINT_STATES = (
    ROSMissionStatus.STATE_NAVIGATING,
    ROSMissionStatus.STATE_PAUSED_DWELL,
    ROSMissionStatus.STATE_PAUSED_CONDITION,
    ROSMissionStatus.STATE_PAUSED,
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


#: What the card's controls need, none of which the page knows in advance: the
#: service the mission's actions run through, and the topic a waypoint picked
#: on the map arrives on, which is the point topic declared to the UI both ways.
#: Runs again on every redraw of the card, so it repaints the picked waypoints
_CARD_SCRIPT = """
(function () {
  const action = "__ACTION__";
  const state = (window.kompassMission = window.kompassMission || {
    waypoints: [], frame: "", service: null, topic: null, socket: null,
  });

  function notify(message) {
    if (typeof UIkit !== "undefined") {
      UIkit.notification({ message: message, status: "danger",
                           pos: "top-center", timeout: 6000 });
    } else {
      console.error(message);
    }
  }

  function render() {
    const list = document.getElementById("__DOM__-waypoint-list");
    if (!list) return;
    if (!state.waypoints.length) {
      list.innerHTML = '<span class="routine-waiting">No waypoints picked yet</span>';
      return;
    }
    list.innerHTML = state.waypoints.map(function (point, index) {
      return '<div class="routine-step pending gap-2">' +
        '<span class="routine-step-mark">' + (index + 1) + '</span>' +
        '<span>x ' + point.x.toFixed(2) + ', y ' + point.y.toFixed(2) + '</span></div>';
    }).join("");
  }

  async function post(url, body) {
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    if (!response.ok) {
      const detail = await response.json().catch(function () { return {}; });
      notify(detail.error || ("HTTP " + response.status));
    }
    return response.ok;
  }

  function follow(topic) {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    state.socket = new WebSocket(scheme + "://" + location.host +
                                 "/api/outputs/" + topic);
    state.socket.onmessage = function (event) {
      // Every output frame is {topic, payload}, the point itself one level in
      const payload = (JSON.parse(event.data) || {}).payload || {};
      const point = payload.data;
      if (!point || point.length < 2) return;
      state.frame = payload.frame_id || state.frame;
      state.waypoints.push({ x: point[0], y: point[1] });
      render();
    };
    state.socket.onclose = function () { state.socket = null; };
  }

  async function discover() {
    if (state.discovered) return;
    state.discovered = true;
    let interfaces;
    try {
      interfaces = await (await fetch("/api/interfaces")).json();
    } catch (error) {
      notify("The mission card could not read what this UI offers");
      return;
    }
    const services = (interfaces.services || []).filter(function (service) {
      return service.type === "ExecuteMethod";
    });
    state.service = services.length === 1 ? services[0].name : null;
    const streamed = (interfaces.outputs || []).filter(function (topic) {
      return topic.msg_type === "PointStamped";
    }).map(function (topic) { return topic.name; });
    const picked = (interfaces.inputs || []).filter(function (topic) {
      return topic.msg_type === "PointStamped" && streamed.indexOf(topic.name) >= 0;
    });
    state.topic = picked.length ? picked[0].name : null;
    if (state.topic) follow(state.topic);
  }

  window.missionAddWaypoint = function () {
    const x = parseFloat((document.getElementById("__DOM__-x") || {}).value);
    const y = parseFloat((document.getElementById("__DOM__-y") || {}).value);
    if (isNaN(x) || isNaN(y)) {
      notify("A waypoint needs an x and a y");
      return;
    }
    state.waypoints.push({ x: x, y: y });
    render();
  };

  window.missionClearWaypoints = function () {
    state.waypoints = [];
    render();
  };

  window.missionControl = function (method) {
    if (!state.service) {
      notify("This UI was not given the mission's service to pause it through");
      return;
    }
    post("/api/services/" + state.service, { name: method });
  };

  window.missionCancel = function () {
    post("/api/actions/" + action + "/cancel", {});
  };

  window.missionSend = async function () {
    if (!state.waypoints.length) {
      notify("Pick the waypoints of the mission first");
      return;
    }
    const field = function (name) {
      return ((document.getElementById("__DOM__-" + name) || {}).value || "").trim();
    };
    const dwell = parseFloat(field("dwell") || "0");
    const distance = parseFloat(field("distance"));
    const heading = parseFloat(field("heading"));
    const goAhead = field("go-ahead");
    const goal = {
      goals: state.waypoints.map(function (point) {
        return {
          position: { x: point.x, y: point.y, z: 0.0 },
          orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        };
      }),
      frame_id: state.frame,
      pause_duration: dwell > 0 ? [dwell] : [],
    };
    // Left out when not given, so the planner keeps its own tolerances
    if (!isNaN(distance) || !isNaN(heading)) {
      goal.end_tolerance = {
        lateral_distance_error: isNaN(distance) ? 0.0 : distance,
        orientation_error: isNaN(heading) ? 0.0 : heading,
      };
    }
    if (goAhead) {
      goal.pause_condition_topic = goAhead;
    }
    const sent = await post("/api/actions/" + action, goal);
    if (sent) {
      state.waypoints = [];
      render();
    }
  };

  discover();
  render();
})();
"""


class MissionTask(Task):
    """The card of a mission: how far it has got, waypoint by waypoint.

    The card an action client gets shows its feedback as a log of messages. A
    mission's feedback says which waypoint it is on and what it is doing there,
    which is worth a checklist of the journey instead.
    """

    #: The statuses a mission ends in, as the action reports them
    _ENDED = ("completed", "aborted", "canceled")

    #: Mission state -> what the card says about the waypoint it is on
    _STATES = {
        _FEEDBACK.STATE_NAVIGATING: "driving to",
        _FEEDBACK.STATE_PAUSED_DWELL: "dwelling at",
        _FEEDBACK.STATE_PAUSED_CONDITION: "waiting for the go-ahead at",
        _FEEDBACK.STATE_PAUSED: "paused at",
    }

    def __init__(self, name: str, client_type: str, fields):
        # The latest feedback of the mission being carried out
        self._mission = None
        super().__init__(name=name, client_type=client_type, fields=fields)

    def update(self, *, status=None, feedback=None, duration=None, timestep=None):
        """Keep the mission's progress, and tell the log what moved on.

        Feedback arrives several times a second and mostly says the same thing.
        What belongs in a log is what changed: a waypoint reached, the next one
        started, a wait begun, the mission over
        """
        lines = []
        if feedback is not None and not isinstance(feedback, str):
            lines = self._lines(feedback)
            self._mission = feedback
            feedback = None
        if status and status != self._status and status in self._ENDED:
            lines.append(f"Mission {status}")
        super().update(
            status=status, feedback=feedback, duration=duration, timestep=timestep
        )
        for line in lines:
            # Never the same sentence twice over: the log is what changed
            if self._feedback and self._feedback[-1] == line:
                continue
            super().update(feedback=line)

    def cleanup(self):
        """A new mission starts with a clean card"""
        self._mission = None
        super().cleanup()

    @property
    def card(self):
        """The journey, what can be done to it, the log, and where the next
        mission is put together"""
        mission_card = DivVStacked(
            self._badge, cls="mt-0 gap-2", id=self._dom_id, ws_send=True
        )
        inside = Grid(cls="gap-2 ml-1 mr-1 place-items-center", cols=1)
        inside(self._waypoints)
        controls = self._controls
        if controls is not None:
            inside(controls)
        log = self._feedback_card(title="Mission Log")
        if log is not None:
            inside(log)
        inside(self._picker)
        inside(
            Script(
                _CARD_SCRIPT.replace("__DOM__", self._dom_id).replace(
                    "__ACTION__", self._name
                )
            )
        )
        return mission_card(inside)

    @property
    def _controls(self):
        """Pausing, resuming and cancelling, while there is a mission to do it to.

        A mission is started from the waypoints picked below, not from here
        """
        if not self.is_active():
            return None
        # The same button class throughout, so they are the same size
        return DivHStacked(
            Button(
                "Resume" if self._paused else "Pause",
                cls="primary-button",
                type="button",
                onclick="missionControl('%s')"
                % ("resume_mission" if self._paused else "pause_mission"),
            ),
            Button(
                "Cancel",
                cls="primary-button",
                type="button",
                onclick="missionCancel()",
            ),
            cls="gap-2",
        )

    @property
    def _paused(self) -> bool:
        """Whether the mission is holding position until it is resumed.

        Only while it is still under way: a mission cancelled while paused is
        cancelled, not paused
        """
        return (
            self.is_active()
            and self._mission is not None
            and self._mission.state == _FEEDBACK.STATE_PAUSED
        )

    @property
    def _badge(self):
        """What the mission is doing, which is not what its goal is doing.

        The goal of a paused mission is still running, as far as the action is
        concerned, so the status it reports would say so
        """
        if not self._paused:
            return super()._badge
        badge = DivHStacked(
            Span("paused", cls="status-badge paused", id="status-badge-div")
        )
        if self._duration is not None:
            badge(self._timer)
        return badge

    @property
    def _picker(self):
        """Where the next mission is put together: the waypoints, picked on the
        map or typed in, and how long to dwell at each of them"""
        return Card(
            Div(id=f"{self._dom_id}-waypoint-list", cls="routine-steps"),
            DivHStacked(
                Input(
                    id=f"{self._dom_id}-x",
                    placeholder="x",
                    type="number",
                    autocomplete="off",
                ),
                Input(
                    id=f"{self._dom_id}-y",
                    placeholder="y",
                    type="number",
                    autocomplete="off",
                ),
                Button(
                    "Add",
                    cls="secondary-button",
                    type="button",
                    onclick="missionAddWaypoint()",
                ),
                Button(
                    "Clear",
                    cls="secondary-button",
                    type="button",
                    onclick="missionClearWaypoints()",
                ),
                cls="gap-2",
            ),
            Grid(
                LabelInput(
                    label="Dwell at each waypoint (s)",
                    id=f"{self._dom_id}-dwell",
                    type="number",
                    value="0",
                    autocomplete="off",
                    cls="form-input space-y-1",
                ),
                LabelInput(
                    label="Wait at each for (topic)",
                    id=f"{self._dom_id}-go-ahead",
                    type="text",
                    placeholder="none",
                    autocomplete="off",
                    cls="form-input space-y-1",
                ),
                cols=2,
                cls="gap-2 m-1",
            ),
            P("Arrival tolerance", cls="cool-subtitle-mini-blue m-2"),
            Grid(
                LabelInput(
                    label="Distance (m)",
                    id=f"{self._dom_id}-distance",
                    type="number",
                    placeholder="system default",
                    autocomplete="off",
                    cls="form-input space-y-1",
                ),
                LabelInput(
                    label="Heading (rad)",
                    id=f"{self._dom_id}-heading",
                    type="number",
                    placeholder="system default",
                    autocomplete="off",
                    cls="form-input space-y-1",
                ),
                cols=2,
                cls="gap-2 m-1",
            ),
            DivHStacked(
                Button(
                    "Send mission",
                    cls="primary-button",
                    type="button",
                    onclick="missionSend()",
                ),
                cls="gap-2",
            ),
            header=H6("Waypoints", cls="tomorrow-night-green"),
            cls="inner-main-card ml-2 mr-2 mt-0",
        )

    @property
    def _waypoints(self):
        """The waypoints as a checklist: reached, the one being worked on, and
        the ones still to come"""
        mission = self._mission
        if mission is None or not mission.total_goals:
            return P("No mission running", cls="routine-waiting")
        current = mission.current_goal_idx
        done = self._status == "completed"
        checklist = Div(cls="routine-steps", id=f"{self._dom_id}-waypoints")
        for index in range(mission.total_goals):
            if done or index < current:
                mark, kind = "✓", "done"
            elif index > current:
                mark, kind = "○", "pending"
            elif self.is_active():
                mark, kind = "●", "active"
            else:
                # Ended on this one, which it did not reach
                mark, kind = "✕", "stopped"
            line = DivHStacked(
                Span(mark, cls="routine-step-mark"),
                Span(f"Waypoint {index + 1}"),
                cls=f"routine-step {kind} gap-2",
            )
            if kind == "active":
                line(Span(self._doing(mission), cls="routine-step-feedback"))
            checklist(line)
        return checklist

    def _lines(self, feedback) -> List[str]:
        """What is worth saying about this feedback, which is usually nothing"""
        previous = self._mission
        reached = (
            previous is not None
            and feedback.current_goal_idx > previous.current_goal_idx
        )
        moved_on = previous is None or (feedback.state, feedback.current_goal_idx) != (
            previous.state,
            previous.current_goal_idx,
        )
        lines = []
        if reached:
            lines.append(
                f"Reached waypoint {previous.current_goal_idx + 1}"
                f"/{previous.total_goals}"
            )
        if moved_on:
            lines.append(self._describe(feedback))
        return lines

    @staticmethod
    def _describe(feedback) -> str:
        """The line a change of state puts in the log"""
        if feedback.state == _FEEDBACK.STATE_RETURNING_TO_START:
            return "Returning to where the mission started"
        doing = MissionTask._STATES.get(feedback.state, "")
        if feedback.state == _FEEDBACK.STATE_NAVIGATING:
            # Where it is driving to, not only that it is driving
            goal = feedback.current_goal.position
            doing = f"{doing} x {goal.x:.2f}, y {goal.y:.2f}"
        else:
            doing = doing.removesuffix(" at")
        waypoint = f"{feedback.current_goal_idx + 1}/{feedback.total_goals}"
        return f"Waypoint {waypoint}: {doing}"

    @staticmethod
    def _doing(feedback) -> str:
        """What the mission is doing at the waypoint it is on, as it happens"""
        if feedback.state == _FEEDBACK.STATE_RETURNING_TO_START:
            return "returning to start"
        # The same words as the log line, without what they lead into
        doing = MissionTask._STATES.get(feedback.state, "").removesuffix(
            " to"
        ).removesuffix(" at")
        if feedback.state == _FEEDBACK.STATE_NAVIGATING:
            left = MissionTask._distance_left(feedback)
            return f"{doing}, {left:.1f} m to go" if left is not None else doing
        held = feedback.time_paused
        return f"{doing}, {held:.0f}s" if held else doing

    @staticmethod
    def _distance_left(feedback) -> Optional[float]:
        """How far the robot still is from the waypoint, when it knows where it
        is. A location the mission could not fill in carries no frame
        """
        if not feedback.current_pose.header.frame_id:
            return None
        here = feedback.current_pose.pose.position
        goal = feedback.current_goal.position
        return math.hypot(goal.x - here.x, goal.y - here.y)


#: The card of an action type Kompass shows its own way
TASK_ELEMENTS = {ROSMultiGoalPlanPath: MissionTask}
