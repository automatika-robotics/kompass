"""Mission manager: run a sequence of navigation goals as one action.

A multi-waypoint mission is a sequence with per-step policy: drive somewhere,
wait, drive somewhere else, and decide what a failure or a timeout means. That
is what a Sugarcoat `Routine` is, so this component owns no sequencing of its
own. It translates a `MultiGoalPlanPath` goal into a routine specification,
registers it on the Monitor over the runtime API, and reports the routine's
cursor back as action feedback.
"""

import json
import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple, Union

from attrs import define, field

from automatika_ros_sugar.srv import ExecuteMethod
from geometry_msgs.msg import Pose, PoseStamped
from kompass_core.models import RobotState
from tf2_geometry_msgs import do_transform_pose
from kompass_interfaces.action import MultiGoalPlanPath as MultiGoalPlanPathAction
from kompass_interfaces.msg import MissionStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ros_sugar.base_clients import ServiceClientConfig, ServiceClientHandler
from ros_sugar.core import Monitor, SystemActionRegistry
from ros_sugar.io import Topic

from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..callbacks import GenericCallback
from .component import Component
from .controller import Controller
from .defaults import (
    TopicsKeys,
    mission_allowed_inputs,
    mission_allowed_outputs,
    mission_default_inputs,
    mission_default_outputs,
)
from .drive_manager import DriveManager
from .ros import ActionReturnType, component_action

__all__ = ["MissionManager", "MissionManagerConfig"]

#: The Monitor action a step uses to hold itself open. A dwell waits on it; a
#: conditional pause calls it with no duration and lets its success condition
#: decide when to move on
WAIT_ACTION = "monitor/wait"

#: What a mission goal can ask for when a conditional pause runs out of time
ON_TIMEOUT_POLICIES = (
    MultiGoalPlanPathAction.Goal.ON_TIMEOUT_CONTINUE,
    MultiGoalPlanPathAction.Goal.ON_TIMEOUT_RETURN_TO_START,
    MultiGoalPlanPathAction.Goal.ON_TIMEOUT_ABORT,
)


# ---------------------------------------------------------------------------
# Action Translation ----------------------------------------------------------

def _yaw_of(pose) -> float:
    """Heading of a geometry_msgs/Pose, from its quaternion"""
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


def dwell_seconds(pause_duration: List[float], index: int) -> float:
    """How long to dwell at one waypoint.

    One entry applies to every waypoint, so a caller with a uniform dwell does
    not have to repeat it.

    :param pause_duration: The goal's dwell list, empty, of length one, or one
        entry per waypoint
    :param index: Which waypoint
    :rtype: float
    """
    if not pause_duration:
        return 0.0
    if len(pause_duration) == 1:
        return float(pause_duration[0])
    if index < len(pause_duration):
        return float(pause_duration[index])
    return 0.0


def goto_step(
    index: int,
    pose,
    *,
    planner_ref: str,
    algorithm_name: str,
    tolerance,
    timeout: float,
) -> Dict[str, Any]:
    """One waypoint, as a goal on the planner's action server.

    Named `goto_<index>`, because a routine requires distinct step names and
    the name is what identifies the step in the cursor.
    """
    return {
        "ref": planner_ref,
        "name": f"goto_{index}",
        "goal": {
            "goal": {
                "position": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                },
                "orientation": {
                    "x": float(pose.orientation.x),
                    "y": float(pose.orientation.y),
                    "z": float(pose.orientation.z),
                    "w": float(pose.orientation.w),
                },
            },
            "algorithm_name": algorithm_name,
            "end_tolerance": {
                "orientation_error": float(tolerance.orientation_error),
                "lateral_distance_error": float(tolerance.lateral_distance_error),
            },
        },
        "timeout": timeout,
        # A navigation goal that ran out of time has not arrived, and driving
        # somewhere again on a second attempt is rarely what was wanted
        "on_timeout": "fail",
        "on_fail": "abort",
    }


def stop_step(index: Union[int, str], ref: str, retries: int = 0) -> Dict[str, Any]:
    """Call one component action that stops the robot before holding position.

    Named after the component and where it runs, so the stops stay distinct

    :param index: The waypoint the stop is at, or what else it runs for
    :param retries: Extra attempts at stopping. A stop reports failure while
        the robot is still rolling, or before its location has arrived, and
        a step that runs out of attempts ends the mission
    """
    owner, _ = SystemActionRegistry.parse_ref(ref)
    return {"ref": ref, "name": f"stop_{owner}_{index}", "max_retries": retries}


def mission_routine_spec(
    goal,
    *,
    name: str,
    planner_ref: str,
    stop_refs: List[str],
    waypoint_timeout: float,
    start_pose=None,
    waypoints: Optional[List[Pose]] = None,
    stop_retries: int = 0,
) -> Dict[str, Any]:
    """Turn a mission goal into a routine specification.

    :param goal: A `MultiGoalPlanPath.Goal`
    :param name: Unique routine name, which is how it is controlled and where
        its cursor is published
    :param planner_ref: The planner's action server, as `component/action`
    :param stop_refs: Component actions that stop the robot, as
        `component/action`, called in order before holding position at a waypoint
    :param waypoint_timeout: Seconds allowed for one waypoint
    :param start_pose: Where the robot was when the mission began, needed only
        for the return-to-start policy
    :param waypoints: The goal's waypoints in the frame the planner drives in.
        Defaults to the goal's own poses, which is what a goal that named no
        frame already gives
    :param stop_retries: Extra attempts at each stop before it ends the mission
    :raises ValueError: If the goal describes no mission, or describes one that
        cannot be carried out as asked
    :rtype: Dict[str, Any]
    """
    if not goal.goals:
        raise ValueError("A mission needs at least one goal waypoint")
    if len(goal.pause_duration) not in (0, 1, len(goal.goals)):
        raise ValueError(
            f"Got {len(goal.pause_duration)} pause durations for "
            f"{len(goal.goals)} waypoints. Give none, one for all of them, or "
            "one each"
        )
    if goal.on_timeout not in ON_TIMEOUT_POLICIES:
        raise ValueError(
            f"Got on_timeout={goal.on_timeout}, which is no policy. Give "
            "CONTINUE (0), RETURN_TO_START (1) or ABORT (2)"
        )
    points = list(goal.goals) if waypoints is None else list(waypoints)
    last = len(points) - 1
    returning = goal.on_timeout == MultiGoalPlanPathAction.Goal.ON_TIMEOUT_RETURN_TO_START
    if returning and goal.pause_condition_topic and last > 0 and start_pose is None:
        raise ValueError(
            "The return-to-start policy needs the pose the mission started "
            "from, and none was given"
        )

    # What a go-ahead that never arrives means, the same at every waypoint.
    # CONTINUE makes the wait running out an acceptable outcome, both other
    # policies end the mission. A non positive timeout waits indefinitely
    timeout_policy: Dict[str, Any] = (
        {
            "timeout": float(goal.condition_timeout),
            "on_timeout": "succeed"
            if goal.on_timeout == MultiGoalPlanPathAction.Goal.ON_TIMEOUT_CONTINUE
            else "fail",
        }
        if goal.condition_timeout > 0
        else {}
    )

    steps: List[Dict[str, Any]] = []
    for index, pose in enumerate(points):
        steps.append(
            goto_step(
                index,
                pose,
                planner_ref=planner_ref,
                algorithm_name=goal.algorithm_name,
                tolerance=goal.end_tolerance,
                timeout=waypoint_timeout,
            )
        )
        dwell = dwell_seconds(list(goal.pause_duration), index)
        # A condition is a wait to go on to the next waypoint. At the last one
        # there is nothing to go on to, and waiting would only hold up success
        waits = bool(goal.pause_condition_topic) and index < last
        if dwell > 0 or waits:
            # The planner is done once within tolerance, while the controller
            # may still be driving
            steps.extend(
                stop_step(index, ref, retries=stop_retries) for ref in stop_refs
            )
        if dwell > 0:
            steps.append(
                {"ref": WAIT_ACTION, "name": f"dwell_{index}", "kwargs": {"duration": dwell}}
            )
        if waits:
            # Its work is nothing: the success condition is what holds it open
            go_ahead = Topic(name=goal.pause_condition_topic, msg_type="Bool")
            steps.append({
                "ref": WAIT_ACTION,
                "name": f"pause_{index}",
                "kwargs": {"duration": 0.0},
                "success": go_ahead.msg.data.is_true().to_dict(),
                "on_fail": "abort",
                **timeout_policy,
            })

    spec: Dict[str, Any] = {"name": name, "steps": steps}
    # Pausing preempts the step in flight, which cancels the goal of a waypoint
    # being driven but leaves the robot moving. It holds position until resumed
    spec["on_pause"] = [
        stop_step("on_pause", ref, retries=stop_retries) for ref in stop_refs
    ]
    if returning and start_pose is not None:
        # Runs when the routine aborts, which is what every ending policy
        # except CONTINUE produces
        spec["on_abort"] = goto_step(
            -1,
            start_pose,
            planner_ref=planner_ref,
            algorithm_name=goal.algorithm_name,
            tolerance=goal.end_tolerance,
            timeout=waypoint_timeout,
        )
        spec["on_abort"]["name"] = "return_to_start"
    return spec


def reached_waypoints(cursor: Dict[str, Any], total: int) -> List[bool]:
    """Which waypoints the cursor says were reached.

    Read from the step names rather than counted, because a mission's steps are
    not one per waypoint: dwells and conditional pauses sit between them.
    """
    reached = [False] * total
    steps = cursor.get("steps", [])
    index = cursor.get("index", 0)
    done = steps[: index + 1] if cursor.get("status") == "completed" else steps[:index]
    for step_name in done:
        if step_name.startswith("goto_"):
            try:
                reached[int(step_name[len("goto_") :])] = True
            except (ValueError, IndexError):
                continue
    return reached


def ended_on_pause_timeout(cursor: Dict[str, Any]) -> bool:
    """Whether the mission failed because a conditional pause expired.

    Read from the step the routine failed on rather than from its message.
    A pause step's work always succeeds and its success condition is what
    holds it open, so the only way it fails is its timeout running out under
    an ending policy. An aborted routine is not a timeout, whichever step it
    was stopped at.
    """
    if cursor.get("status") != "failed":
        return False
    steps = cursor.get("steps", [])
    index = cursor.get("index", -1)
    return 0 <= index < len(steps) and steps[index].startswith("pause_")


# ---------------------------------------------------------------------------
# The component
# ---------------------------------------------------------------------------

@define(kw_only=True)
class MissionManagerConfig(ComponentConfig):
    """Mission manager configuration

    :param waypoint_timeout: Seconds allowed for one waypoint before the
        mission gives up on it
    :param cursor_poll_rate: How often the routine's cursor is read while a
        mission runs, in Hz. Only affects how promptly feedback is published
    :param retries: Extra attempts at whatever a mission does that is worth
        trying again before giving up on it, and giving up on any of them ends
        the mission: stopping the robot before it holds position at a waypoint,
        which reports failure while the robot is still rolling or before its
        location has arrived, and reading the routine's cursor, which fails
        while nothing answers for the routine
    :param end_mission_timeout: Seconds a deactivation waits for the ongoing
        mission to end before taking its action server down. Ending it cancels
        the planner goal in flight, which the planner notices once a loop.
        Also what one call to the Monitor's runtime API is given a share of,
        so that a Monitor that stopped answering is noticed rather than waited
        for and the mission still ends within this
    :param ui_waypoints_topic: Where the UI publishes a waypoint picked on the
        map, for the mission's card to collect into a journey
    :param routine_name: Name of the routine carrying out a mission on the
        Monitor, the same for every mission as they run one at a time. What a
        UI is told to follow, with `launcher.enable_ui(routines=[...])`
    :param planner_action: The planner's main action server, as
        `<planner component name>/<action name>` (e.g. `planner/navigate_to_goal`).
        Filled in from the planner when one is given to the MissionManager
    :param controller_name: Name of the Controller component, stopped before
        holding position at a waypoint. Filled in from the controller when given
    :param drive_manager_name: Name of the DriveManager component, stopped
        before holding position at a waypoint. Filled in from the drive manager
        when given
    """

    waypoint_timeout: float = field(
        default=300.0,
        validator=BaseValidators.in_range(min_value=1.0, max_value=1e6),
    )
    cursor_poll_rate: float = field(
        default=5.0, validator=BaseValidators.in_range(min_value=0.1, max_value=100.0)
    )
    retries: int = field(
        default=2, validator=BaseValidators.in_range(min_value=0, max_value=100)
    )
    end_mission_timeout: float = field(
        default=10.0, validator=BaseValidators.in_range(min_value=1.0, max_value=1e3)
    )
    ui_waypoints_topic: str = field(default="/mission_waypoints")
    routine_name: str = field(default="navigation_mission")
    planner_action: Optional[str] = field(default=None)
    controller_name: Optional[str] = field(default=None)
    drive_manager_name: Optional[str] = field(default=None)


class MissionManager(Component):
    """Runs a sequence of navigation goals as one action.

    ```python
    from kompass.components import MissionManager, MissionManagerConfig

    planner = Planner(component_name="planner")
    planner.run_type = "ActionServer"
    controller = Controller(component_name="controller")
    driver = DriveManager(component_name="drive_manager")
    mission = MissionManager(
        component_name="mission",
        planner=planner,
        controller=controller,
        drive_manager=driver,
    )
    ```

    Sends `MultiGoalPlanPath` goals to `mission/run_mission`. Each mission is
    identified by the time it was received, and the ongoing one is published on
    `mission_status`. The mission is carried out by a Sugarcoat routine on the
    Monitor, named `routine_name` for every mission as they run one at a time.
    Its progress is also visible on `routine/<routine_name>/state`, it can be
    controlled by name through the Monitor's runtime API, and it can be followed
    in the UI with `launcher.enable_ui(routines=[mission.routine_name])`.
    """

    def __init__(
        self,
        component_name: str,
        planner: Optional[Component] = None,
        controller: Optional[Controller] = None,
        drive_manager: Optional[DriveManager] = None,
        config: Optional[MissionManagerConfig] = None,
        config_file: Optional[str] = None,
        inputs: Optional[Dict[TopicsKeys, Any]] = None,
        outputs: Optional[Dict[TopicsKeys, Any]] = None,
        **kwargs,
    ) -> None:
        config = config or MissionManagerConfig()
        # Only the names are kept, so they survive the config being serialized
        # when the mission is launched in its own process
        if planner is not None:
            if planner.run_type != ComponentRunType.ACTION_SERVER:
                raise ValueError(
                    f"Planner '{planner.node_name}' runs as '{planner.run_type}'. "
                    "A MissionManager sends its waypoints to the planner's action "
                    "server, set 'planner.run_type = ComponentRunType.ACTION_SERVER'"
                )
            action_name = SystemActionRegistry.short_name(
                planner.main_action_name, planner.node_name
            )
            config.planner_action = f"{planner.node_name}/{action_name}"
        if controller is not None:
            if not isinstance(controller, Controller):
                raise TypeError(
                    f"'controller' must be a Controller component, got {type(controller)}"
                )
            config.controller_name = controller.node_name
        if drive_manager is not None:
            if not isinstance(drive_manager, DriveManager):
                raise TypeError(
                    f"'drive_manager' must be a DriveManager component, got {type(drive_manager)}"
                )
            config.drive_manager_name = drive_manager.node_name
        if not config_file:
            # A config file is only applied at configure, checked again there
            self.__check_components(config)

        super().__init__(
            config=config,
            config_file=config_file,
            component_name=component_name,
            inputs=inputs or mission_default_inputs,
            outputs=outputs or mission_default_outputs,
            allowed_inputs=mission_allowed_inputs,
            allowed_outputs=mission_allowed_outputs,
            allowed_run_types=[ComponentRunType.ACTION_SERVER],
            **kwargs,
        )
        self.config: MissionManagerConfig = config
        self.action_type = MultiGoalPlanPathAction
        self.main_action_name = "run_mission"
        # The only way it runs: a mission is one long goal, not a loop
        self.run_type = ComponentRunType.ACTION_SERVER
        self._monitor_client: Optional[ServiceClientHandler] = None
        # Ongoing mission and the latest message, for the mission status
        self._mission_id: str = ""
        self._message: str = ""
        self._message_level: int = MissionStatus.LEVEL_INFO
        # Latest progress of the ongoing mission, which its final status carries
        self._last_feedback = None
        # Set when the node asks the ongoing mission to end, and why
        self._end_requested = threading.Event()
        self._end_reason: str = ""

    @staticmethod
    def __check_components(config: MissionManagerConfig) -> None:
        """Raise if a component the mission drives is not known"""
        missing = [
            key
            for key in ("planner_action", "controller_name", "drive_manager_name")
            if not getattr(config, key)
        ]
        if missing:
            raise ValueError(
                f"MissionManager is missing {missing}. Pass the 'planner', "
                "'controller' and 'drive_manager' components, or set these in "
                "its config (e.g. planner_action='planner/navigate_to_goal', "
                "controller_name='controller', drive_manager_name='drive_manager')"
            )

    def custom_on_configure(self):
        """Check the components once the config file has been applied"""
        super().custom_on_configure()
        self.__check_components(self.config)

    def on_deactivate(self, state):
        """End an ongoing mission before the action server is taken down.

        Deactivating destroys the action server, and with it the goal handle:
        a client left waiting on a mission would never get a result, and the
        routine would carry on with nothing following it. So the mission ends
        itself first: its loop aborts the routine, which cancels the planner
        goal in flight, and reports the goal as aborted. The action server only
        sends that result once the mission callback has returned, which is
        what the wait is for
        """
        with self._main_goal_lock:
            ongoing = self._main_goal_handle is not None
        if ongoing:
            self._end_reason = "the mission manager was deactivated"
            self._end_requested.set()
            deadline = time.monotonic() + self.config.end_mission_timeout
            while ongoing and time.monotonic() < deadline:
                time.sleep(0.05)
                with self._main_goal_lock:
                    ongoing = self._main_goal_handle is not None
        if ongoing:
            self.get_logger().error(
                f"Mission {self._mission_id} did not end within "
                f"{self.config.end_mission_timeout}s, "
                "its client may never get a result"
            )
        return super().on_deactivate(state)

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "pause_mission",
                "description": "Pause the ongoing mission: the robot stops where "
                "it is and holds position until the mission is resumed.",
                "parameters": {"type": "object", "properties": {}},
            },
        }
    )
    def pause_mission(self, **_) -> ActionReturnType:
        """Pause the ongoing mission, stopping the robot until `resume_mission`.

        On resuming, a waypoint that was being driven to is driven to again, a
        dwell starts over and a conditional pause waits for a new go-ahead

        :rtype: ActionReturnType
        """
        return self.__control_routine("pause_routine", "pause")

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "resume_mission",
                "description": "Resume a paused mission from the waypoint it was "
                "paused at.",
                "parameters": {"type": "object", "properties": {}},
            },
        }
    )
    def resume_mission(self, **_) -> ActionReturnType:
        """Resume a paused mission where it was paused

        :rtype: ActionReturnType
        """
        return self.__control_routine("resume_routine", "resume")

    def __control_routine(self, method: str, what: str) -> ActionReturnType:
        """Pause or resume the routine of the ongoing mission"""
        if not self._mission_id:
            return False, f"No ongoing mission to {what}"
        response = self.call_monitor(method, routine_name=self.routine_name)
        if response is None:
            return False, f"Could not {what} the mission: the monitor did not answer"
        if not response.success:
            return False, f"Could not {what} the mission: {response.error_msg}"
        self.__report(f"Mission {self._mission_id}: {what} requested")
        return True, f"Mission {self._mission_id}: {what} requested"

    @property
    def ui_waypoints(self) -> Topic:
        """The topic a waypoint picked on the UI's map is published on.

        Declared to the UI both ways: the map publishes a click on it, and the
        mission's card reads it back to collect the journey

        :rtype: Topic
        """
        return Topic(name=self.config.ui_waypoints_topic, msg_type="PointStamped")

    @property
    def ui_inputs(self) -> List[Any]:
        """What the mission's card needs, for `launcher.enable_ui(inputs=...)`.

        The action a mission is sent to, the service the card pauses and resumes
        through, and the topic it collects waypoints from

        :rtype: List[Any]
        """
        return [
            self.ui_main_action_input,
            ServiceClientConfig(
                name=f"{self.node_name}/execute_method", srv_type=ExecuteMethod
            ),
            self.ui_waypoints,
        ]

    @property
    def ui_outputs(self) -> List[Topic]:
        """What the mission's card follows, for `launcher.enable_ui(outputs=...)`.

        Its status, and the waypoints picked on the map

        :rtype: List[Topic]
        """
        return [self.get_out_topic(TopicsKeys.MISSION_STATUS), self.ui_waypoints]

    @property
    def routine_name(self) -> str:
        """Name of the routine carrying out a mission, the same for every one.

        Missions run one at a time, so one name serves them all, and it is known
        before any mission starts, which is what lets a UI follow it
        """
        return self.config.routine_name

    @property
    def stop_refs(self) -> List[str]:
        """Component actions that stop the robot, controller first so no new
        commands reach the drive manager"""
        return [
            f"{self.config.controller_name}/{Controller.stop_path_tracking.__name__}",
            f"{self.config.drive_manager_name}/{DriveManager.stop_robot.__name__}",
        ]

    def _execution_step(self, *_, **__):
        """Nothing runs outside a mission; the routine does the work"""
        pass

    def __robot_state(self) -> Optional[RobotState]:
        """Where the robot is, in the world frame, or None if nothing knows yet.

        Read the way every other component reads it: through the location
        callback, with the transform from the frame the location messages are
        stamped in to the world frame. Without it the mission would take a pose
        in the odometry frame for a pose in the frame the planner drives in
        """
        try:
            callback: Optional[GenericCallback] = self.get_callback(
                TopicsKeys.ROBOT_LOCATION
            )
        except KeyError:
            return None
        if callback is None:
            return None
        listener = self.odom_tf_listener
        return callback.get_output(
            transformation=listener.transform if listener else None
        )

    @staticmethod
    def __pose_of(state: RobotState) -> Pose:
        """A robot state as a geometry_msgs/Pose"""
        pose = Pose()
        pose.position.x = float(state.x)
        pose.position.y = float(state.y)
        pose.orientation.z = float(math.sin(state.yaw / 2))
        pose.orientation.w = float(math.cos(state.yaw / 2))
        return pose

    def start_pose(self) -> Optional[Pose]:
        """Where the robot is now, as a geometry_msgs/Pose in the world frame.

        Read when a mission starts, so the return-to-start policy has somewhere
        to return to. None when nothing has published a location yet, which
        mission_routine_spec turns into a refusal rather than a wrong pose.
        """
        state = self.__robot_state()
        return self.__pose_of(state) if state is not None else None

    def current_pose(self) -> Optional[PoseStamped]:
        """Where the robot is now, as a geometry_msgs/PoseStamped, or None.

        Stamped in the world frame, which is the frame the pose was brought
        into, whatever frame the location messages themselves are in
        """
        state = self.__robot_state()
        if state is None:
            return None
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.config.frames.world
        pose_stamped.header.stamp = self.get_ros_time()
        pose_stamped.pose = self.__pose_of(state)
        return pose_stamped

    # ---- Talking to the Monitor -------------------------------------------

    @property
    def monitor(self) -> ServiceClientHandler:
        """Client for the Monitor's runtime API.

        NOTE: created lazily, in a callback group of its own. The mission blocks
        in its action callback while calling this, so the response has to be
        free to arrive on another thread of the component's executor, including
        while a lifecycle transition, which runs in the node's default group,
        waits for the mission to end.
        """
        if self._monitor_client is None:
            self._monitor_client = ServiceClientHandler(
                client_node=self,
                config=ServiceClientConfig(
                    srv_type=ExecuteMethod,
                    name=Monitor.RUNTIME_API_SERVICE,
                    # NOTE: Ending a mission is two calls and can interrupt a third,
                    # and a call that finds no service costs about one and a
                    # half times its timeout, so a fifth each keeps the whole
                    # ending inside the time a deactivation waits for it
                    timeout_secs=self.config.end_mission_timeout / 5,
                    # Looked for twice within that, so a Monitor that is not
                    # there costs a call about as much as one that is slow
                    attempt_period_secs=self.config.end_mission_timeout / 10,
                ),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        return self._monitor_client

    def call_monitor(self, method: str, **kwargs) -> Any:
        """Run one runtime API method, returning its response or None"""
        request = ExecuteMethod.Request()
        request.name = method
        request.kwargs_json = json.dumps(kwargs) if kwargs else ""
        return self.monitor.send_request(request)

    def cursor(self, routine_name: str) -> Optional[Dict[str, Any]]:
        """Where the routine has got to, or None if it could not be read"""
        response = self.call_monitor("get_routine_state", routine_name=routine_name)
        if response is None or not response.success:
            return None
        try:
            return json.loads(response.response_json)
        except json.JSONDecodeError:
            return None

    # ---- Mission status ----------------------------------------------------

    def custom_on_activate(self):
        """Publish that there is no ongoing mission yet"""
        super().custom_on_activate()
        self.__publish_status()

    def __report(self, message: str, error: bool = False) -> None:
        """Log a message and keep it as the latest one in the mission status"""
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)
        self._message = message
        self._message_level = (
            MissionStatus.LEVEL_ERROR if error else MissionStatus.LEVEL_INFO
        )

    def __publish_status(self, feedback=None, state: Optional[int] = None) -> None:
        """Publish the mission status.

        :param feedback: Progress of the ongoing mission, none before the first
        :param state: State to report instead of the feedback's, for the final
            status of a mission
        """
        status = MissionStatus()
        status.mission_id = self._mission_id
        status.state = MissionStatus.STATE_IDLE
        if feedback is not None:
            status.state = feedback.state
            status.total_goals = feedback.total_goals
            status.current_goal_idx = feedback.current_goal_idx
        if state is not None:
            status.state = state
        status.message_level = self._message_level
        status.message = self._message
        self.get_publisher(TopicsKeys.MISSION_STATUS).publish(status)

    # ---- Running a mission -------------------------------------------------

    def main_action_callback(self, goal_handle):
        """Carry out one mission, as a routine on the Monitor.

        NOTE: The action server takes one goal at a time, a new mission is
        rejected until this callback returns
        """
        # The time it was received identifies the mission
        self._mission_id = str(self.get_clock().now().nanoseconds)
        try:
            return self.__run_mission(goal_handle)
        finally:
            self._mission_id = ""
            self._last_feedback = None
            self._end_requested.clear()

    def __run_mission(self, goal_handle):
        """Register the mission as a routine and run it"""
        goal = goal_handle.request
        result = MultiGoalPlanPathAction.Result()
        total = len(goal.goals)
        result.reached_waypoints = [False] * total
        result.last_reached_index = -1
        self._last_feedback = MultiGoalPlanPathAction.Feedback(total_goals=total)

        routine_name = self.routine_name

        try:
            waypoints = list(goal.goals)
            world = self.config.frames.world
            if goal.frame_id and goal.frame_id != world:
                # Brought into the world frame the planner drives in, the
                # transform waited for as long as a topic would be
                listener = self.get_transform_listener(goal.frame_id, world)
                deadline = time.monotonic() + self.config.topic_subscription_timeout
                while not listener.got_transform and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not listener.got_transform:
                    raise ValueError(
                        f"The waypoints are given in the '{goal.frame_id}' frame, "
                        f"and its transform to the '{world}' frame is not available"
                    )
                waypoints = [
                    do_transform_pose(pose, listener.transform) for pose in goal.goals
                ]
            spec = mission_routine_spec(
                goal,
                name=routine_name,
                planner_ref=self.config.planner_action,
                stop_refs=self.stop_refs,
                waypoint_timeout=self.config.waypoint_timeout,
                start_pose=self.start_pose(),
                waypoints=waypoints,
                stop_retries=self.config.retries,
            )
        except ValueError as e:
            self.__report(f"Mission {self._mission_id} refused: {e}", error=True)
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_FAILED
            )

        # Replacing one left over by a mission whose routine could not be
        # removed, which would otherwise keep every later mission from starting
        registered = self.call_monitor("add_routine", routine=spec, replace=True)
        if registered is None or not registered.success:
            reason = registered.error_msg if registered else "the monitor did not answer"
            self.__report(
                f"Mission {self._mission_id} could not be registered: {reason}",
                error=True,
            )
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_FAILED
            )

        try:
            return self.__run_routine(
                goal_handle, waypoints, routine_name, result, total
            )
        finally:
            # Whatever happened, the routine belongs to this goal and goes with
            # it. Forced, since an abort mid-step leaves it running
            self.call_monitor("remove_routine", routine_name=routine_name, force=True)

    def __run_routine(self, goal_handle, waypoints, routine_name, result, total):
        """Start the routine and follow its cursor until it ends"""
        started = self.call_monitor("start_routine", routine_name=routine_name)
        if started is None or not started.success:
            reason = started.error_msg if started else "the monitor did not answer"
            self.__report(
                f"Mission {self._mission_id} could not start: {reason}", error=True
            )
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_FAILED
            )

        self.__report(f"Mission {self._mission_id} started with {total} waypoint(s)")
        self._last_feedback = self.__feedback_from_cursor({}, waypoints, total)
        self.__publish_status(self._last_feedback)
        period = 1.0 / self.config.cursor_poll_rate
        cursor: Dict[str, Any] = {}
        # Cursor reads that failed in a row, which is how a Monitor that no
        # longer answers ends the mission instead of holding it open forever
        missed = 0
        # The pause step being held and when it was first seen, for time_paused
        pause: Tuple[Optional[Tuple], float] = (None, 0.0)

        while True:
            if self._end_requested.is_set() or goal_handle.is_cancel_requested:
                # Canceled by the client, or asked to end by the node. Aborting
                # the routine cancels the planner goal in flight, and the planner
                # drops the plan it was driving, which stops the robot
                requested = self._end_requested.is_set()
                reason = self._end_reason if requested else "mission canceled"
                self.__fill_progress(result, cursor, total, waypoints)
                self.call_monitor(
                    "abort_routine", routine_name=routine_name, reason=reason
                )
                self.__report(
                    f"Mission {self._mission_id} "
                    f"{f'ended: {reason}' if requested else 'canceled'}",
                    error=requested,
                )
                return self.__finish(
                    goal_handle,
                    result,
                    MultiGoalPlanPathAction.Result.OUTCOME_FAILED
                    if requested
                    else MultiGoalPlanPathAction.Result.OUTCOME_CANCELED,
                )

            latest = self.cursor(routine_name)
            if latest is None:
                missed += 1
                if missed > self.config.retries:
                    # Nothing can be said about a routine that cannot be read,
                    # and it may not even be running. Ending the mission here
                    # aborts it and hands the action server back, rather than
                    # holding one goal open and rejecting every mission after it
                    self._end_reason = (
                        f"the mission routine could not be read {missed} times "
                        "in a row"
                    )
                    self._end_requested.set()
            else:
                missed = 0
                cursor = latest
                # An ended routine has no active step to report, only a result
                if cursor.get("status") in ("completed", "failed", "aborted"):
                    break
                feedback = self.__feedback_from_cursor(cursor, waypoints, total)
                # Timed from the first poll that sees a pause, so it can be short
                # by up to one poll period. Told apart by state and step, so
                # pausing the mission during a dwell starts a new one
                if feedback.state in (
                    MultiGoalPlanPathAction.Feedback.STATE_PAUSED_DWELL,
                    MultiGoalPlanPathAction.Feedback.STATE_PAUSED_CONDITION,
                    MultiGoalPlanPathAction.Feedback.STATE_PAUSED,
                ):
                    current = (feedback.state, cursor.get("active_step"))
                    if current != pause[0]:
                        pause = (current, time.monotonic())
                    feedback.time_paused = time.monotonic() - pause[1]
                else:
                    pause = (None, 0.0)
                if (pose := self.current_pose()) is not None:
                    feedback.current_pose = pose
                goal_handle.publish_feedback(feedback)
                self._last_feedback = feedback
                self.__publish_status(feedback)
            # Woken early by a request to end the mission
            self._end_requested.wait(period)

        self.__fill_progress(result, cursor, total, waypoints)
        if cursor.get("status") == "completed":
            self.__report(f"Mission {self._mission_id} completed")
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_COMPLETED
            )

        self.__report(
            f"Mission {self._mission_id} ended: {cursor.get('message', '')}",
            error=True,
        )
        # A pause that ran out under an ending policy is its own outcome: the
        # mission did not fail, it was told to stop waiting
        return self.__finish(
            goal_handle,
            result,
            MultiGoalPlanPathAction.Result.OUTCOME_TIMED_OUT
            if ended_on_pause_timeout(cursor)
            else MultiGoalPlanPathAction.Result.OUTCOME_FAILED,
        )

    def __finish(self, goal_handle, result, outcome: int):
        """End the mission with an outcome: its final status, then the goal.

        Published once, before whatever cleans up after the mission, which can
        take a while or fail. It is the last status until the next mission, so
        a late subscriber still learns how this one ended
        """
        result.outcome = outcome
        if outcome == MultiGoalPlanPathAction.Result.OUTCOME_COMPLETED:
            state, settle = MissionStatus.STATE_COMPLETED, goal_handle.succeed
        elif outcome == MultiGoalPlanPathAction.Result.OUTCOME_CANCELED:
            state, settle = MissionStatus.STATE_CANCELED, goal_handle.canceled
        else:
            state, settle = MissionStatus.STATE_ABORTED, goal_handle.abort
        self.__publish_status(self._last_feedback, state=state)
        settle()
        return result

    def __fill_progress(
        self, result, cursor: Dict[str, Any], total: int, waypoints: List[Pose]
    ) -> None:
        """Record how far the mission got, from the cursor"""
        result.reached_waypoints = reached_waypoints(cursor, total)
        reached = [i for i, done in enumerate(result.reached_waypoints) if done]
        result.last_reached_index = reached[-1] if reached else -1
        pose = self.start_pose()
        if result.last_reached_index < 0 or pose is None:
            return
        # How far the robot ended up from the last waypoint reached. Measured
        # here, as the routine does not carry back the planner's results
        waypoint = waypoints[result.last_reached_index]
        result.end_displacement.lateral_distance_error = float(
            math.hypot(
                pose.position.x - waypoint.position.x,
                pose.position.y - waypoint.position.y,
            )
        )
        error = _yaw_of(pose) - _yaw_of(waypoint)
        # Into [-pi, pi], so turning the short way around is a small error
        result.end_displacement.orientation_error = float(
            math.atan2(math.sin(error), math.cos(error))
        )

    @staticmethod
    def __feedback_from_cursor(cursor: Dict[str, Any], waypoints: List[Pose], total):
        """The routine's cursor, as mission feedback"""
        feedback = MultiGoalPlanPathAction.Feedback()
        feedback.total_goals = total
        step = cursor.get("active_step") or ""
        # Every step at a waypoint ends in its index
        suffix = step.rpartition("_")[2]

        if step.startswith("goto_"):
            feedback.state = MultiGoalPlanPathAction.Feedback.STATE_NAVIGATING
        elif step.startswith("dwell_"):
            feedback.state = MultiGoalPlanPathAction.Feedback.STATE_PAUSED_DWELL
        elif step.startswith("pause_"):
            # Only once the pause step runs: its condition is watched from then
            feedback.state = MultiGoalPlanPathAction.Feedback.STATE_PAUSED_CONDITION
        elif step == "return_to_start":
            feedback.state = MultiGoalPlanPathAction.Feedback.STATE_RETURNING_TO_START
        # Still names the step it was paused at, which gives the waypoint
        if cursor.get("status") == "paused":
            feedback.state = MultiGoalPlanPathAction.Feedback.STATE_PAUSED

        if suffix.isdigit():
            index = int(suffix)
            feedback.current_goal_idx = index
            if index < total:
                feedback.current_goal = waypoints[index]
        return feedback
