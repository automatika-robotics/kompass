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
from typing import Any, Dict, List, Optional, Tuple

from attrs import define, field

from automatika_ros_sugar.srv import ExecuteMethod
from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs import do_transform_pose
from kompass_interfaces.action import MultiGoalPlanPath as MultiGoalPlanPathAction
from kompass_interfaces.msg import MissionStatus
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ros_sugar.base_clients import ServiceClientHandler
from ros_sugar.config import QoSConfig
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

def _pose_to_dict(pose) -> Dict[str, Any]:
    """A geometry_msgs/Pose as the nested dict a goal spec carries"""
    return {
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
    }


def _yaw_of(pose) -> float:
    """Heading of a geometry_msgs/Pose, from its quaternion"""
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


def _tolerance_to_dict(tolerance) -> Dict[str, float]:
    """A PathTrackingError as a dict"""
    return {
        "orientation_error": float(tolerance.orientation_error),
        "lateral_distance_error": float(tolerance.lateral_distance_error),
    }


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
            "goal": _pose_to_dict(pose),
            "algorithm_name": algorithm_name,
            "end_tolerance": _tolerance_to_dict(tolerance),
        },
        "timeout": timeout,
        # A navigation goal that ran out of time has not arrived, and driving
        # somewhere again on a second attempt is rarely what was wanted
        "on_timeout": "fail",
        "on_fail": "abort",
    }


def stop_step(index: int, ref: str, retries: int = 0) -> Dict[str, Any]:
    """Call one component action that stops the robot before holding position.

    Named after the component, so the stops at one waypoint stay distinct

    :param retries: Extra attempts at stopping. A stop reports failure while
        the robot is still rolling, or before its location has arrived, and
        a step that runs out of attempts ends the mission
    """
    owner, _ = SystemActionRegistry.parse_ref(ref)
    return {"ref": ref, "name": f"stop_{owner}_{index}", "max_retries": retries}


def dwell_step(index: int, seconds: float) -> Dict[str, Any]:
    """Hold position for a fixed time before starting the next waypoint"""
    return {
        "ref": WAIT_ACTION,
        "name": f"dwell_{index}",
        "kwargs": {"duration": seconds},
    }


def condition_step(
    index: int, topic_name: str, timeout: float, on_timeout: int
) -> Dict[str, Any]:
    """Hold position until an external topic says to continue.

    The step's work is nothing: its success condition is what holds it open, and
    the timeout policy is what decides the meaning of the condition never
    arriving. A non positive timeout waits indefinitely.
    """
    condition = Topic(name=topic_name, msg_type="Bool").msg.data.is_true()
    step: Dict[str, Any] = {
        "ref": WAIT_ACTION,
        "name": f"pause_{index}",
        "kwargs": {"duration": 0.0},
        "success": condition.to_dict(),
        "on_fail": "abort",
    }
    if timeout and timeout > 0:
        step["timeout"] = float(timeout)
        # CONTINUE means the wait expiring is an acceptable outcome. Both other
        # policies end the mission; which of them applies is decided by the
        # routine's on_abort, not here
        step["on_timeout"] = (
            "succeed"
            if on_timeout == MultiGoalPlanPathAction.Goal.ON_TIMEOUT_CONTINUE
            else "fail"
        )
    return step


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
            steps.append(dwell_step(index, dwell))
        if waits:
            steps.append(
                condition_step(
                    index,
                    goal.pause_condition_topic,
                    goal.condition_timeout,
                    goal.on_timeout,
                )
            )

    spec: Dict[str, Any] = {"name": name, "steps": steps}
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
    :param stop_retries: Extra attempts at stopping the robot before holding
        position at a waypoint. A stop reports failure while the robot is still
        rolling, or before its location has arrived, and one that runs out of
        attempts ends the mission
    :param end_mission_timeout: Seconds a deactivation waits for the ongoing
        mission to end before taking its action server down. Ending it cancels
        the planner goal in flight, which the planner notices once a loop
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
    stop_retries: int = field(
        default=2, validator=BaseValidators.in_range(min_value=0, max_value=10)
    )
    end_mission_timeout: float = field(
        default=10.0, validator=BaseValidators.in_range(min_value=0.0, max_value=1e3)
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
    Monitor, so its progress is also visible on `routine/mission_<node>_<id>/state`
    and it can be controlled by name through the Monitor's runtime API.
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
        routine would carry on with nothing following it. So the mission is
        ended first, and given a while to unwind
        """
        self.__end_ongoing_mission("the mission manager was deactivated")
        return super().on_deactivate(state)

    def __end_ongoing_mission(self, reason: str) -> None:
        """Ask the ongoing mission to end, and wait a while until it has.

        The mission ends itself: its loop aborts the routine, which cancels the
        planner goal in flight, and reports the goal as aborted. Waiting here is
        what lets that result reach the client, as the action server sends it
        only once the mission callback has returned
        """
        with self._main_goal_lock:
            if self._main_goal_handle is None:
                return
        self._end_reason = reason
        self._end_requested.set()
        deadline = time.monotonic() + END_MISSION_TIMEOUT
        while time.monotonic() < deadline:
            with self._main_goal_lock:
                if self._main_goal_handle is None:
                    return
            time.sleep(0.05)
        self.get_logger().error(
            f"Mission {self._mission_id} did not end within {END_MISSION_TIMEOUT}s, "
            "its client may never get a result"
        )

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

    def __location_message(self):
        """The latest robot location message, or None if none was received"""
        try:
            callback: Optional[GenericCallback] = self.get_callback(
                TopicsKeys.ROBOT_LOCATION
            )
        except KeyError:
            return None
        if callback is None:
            return None
        return getattr(callback, "msg", None)

    @staticmethod
    def __pose_of(message):
        """The geometry_msgs/Pose in a location message"""
        # Odometry nests it, PoseStamped wraps it, Pose is already one
        pose = getattr(message, "pose", message)
        return getattr(pose, "pose", pose)

    def start_pose(self):
        """Where the robot is now, as a geometry_msgs/Pose, or None.

        Read when a mission starts, so the return-to-start policy has somewhere
        to return to. None when nothing has published a location yet, which
        mission_routine_spec turns into a refusal rather than a wrong pose.
        """
        message = self.__location_message()
        if message is None:
            return None
        return self.__pose_of(message)

    def current_pose(self) -> Optional[PoseStamped]:
        """Where the robot is now, as a geometry_msgs/PoseStamped, or None.

        A Pose location has no header, so its stamp and frame are left empty
        """
        message = self.__location_message()
        if message is None:
            return None
        pose_stamped = PoseStamped()
        header = getattr(message, "header", None)
        if header is not None:
            pose_stamped.header = header
        pose_stamped.pose = self.__pose_of(message)
        return pose_stamped

    def __waypoints_in_world(self, goal) -> List[Pose]:
        """The goal's waypoints in the world frame, which the planner drives in.

        A goal that names no frame is taken to be in the world frame already.
        Otherwise its transform is waited for as long as a topic would be

        :raises ValueError: If the goal names a frame whose transform to the
            world frame does not arrive in time
        """
        world = self.config.frames.world
        frame = goal.frame_id
        if not frame or frame == world:
            return list(goal.goals)
        listener = self.get_transform_listener(frame, world)
        deadline = time.monotonic() + self.config.topic_subscription_timeout
        while not listener.got_transform and time.monotonic() < deadline:
            time.sleep(0.1)
        if not listener.got_transform:
            raise ValueError(
                f"The waypoints are given in the '{frame}' frame, and its "
                f"transform to the '{world}' frame is not available"
            )
        return [do_transform_pose(pose, listener.transform) for pose in goal.goals]

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
                srv_type=ExecuteMethod,
                srv_name=Monitor.RUNTIME_API_SERVICE,
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

        # Unique per mission: the name is how the routine is controlled
        routine_name = f"mission_{self.node_name}_{self._mission_id}"

        try:
            waypoints = self.__waypoints_in_world(goal)
            spec = mission_routine_spec(
                goal,
                name=routine_name,
                planner_ref=self.config.planner_action,
                stop_refs=self.stop_refs,
                waypoint_timeout=self.config.waypoint_timeout,
                start_pose=self.start_pose(),
                waypoints=waypoints,
                stop_retries=self.config.stop_retries,
            )
        except ValueError as e:
            self.__report(f"Mission {self._mission_id} refused: {e}", error=True)
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_FAILED
            )

        registered = self.call_monitor("add_routine", routine=spec)
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
        # The pause step being held and when it was first seen, for time_paused
        pause: Tuple[Optional[str], float] = (None, 0.0)

        while True:
            if self._end_requested.is_set() or goal_handle.is_cancel_requested:
                self.__fill_progress(result, cursor, total, waypoints)
                return self.__end_early(goal_handle, routine_name, result)

            latest = self.cursor(routine_name)
            if latest is not None:
                cursor = latest
                # An ended routine has no active step to report, only a result
                if cursor.get("status") in ("completed", "failed", "aborted"):
                    break
                feedback = self.__feedback_from_cursor(cursor, waypoints, total)
                pause = self.__time_pause(feedback, cursor.get("active_step"), pause)
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

    @staticmethod
    def __time_pause(
        feedback, step: Optional[str], pause: Tuple[Optional[str], float]
    ) -> Tuple[Optional[str], float]:
        """Fill in how long the current pause has been held.

        Timed from the first poll that sees the pause, so it can be short by up
        to one poll period

        :param step: The routine's active step
        :param pause: The pause step being held and when it was first seen
        :return: The same, after this poll
        """
        if feedback.state not in (
            MultiGoalPlanPathAction.Feedback.STATE_PAUSED_DWELL,
            MultiGoalPlanPathAction.Feedback.STATE_PAUSED_CONDITION,
        ):
            return None, 0.0
        held, since = pause
        if step != held:
            held, since = step, time.monotonic()
        feedback.time_paused = time.monotonic() - since
        return held, since

    def __end_early(self, goal_handle, routine_name: str, result):
        """End the mission before its routine has: canceled by the client, or
        asked to end by the node.

        Aborting the routine cancels the planner goal in flight, and the planner
        drops the plan it was driving, which stops the robot
        """
        requested = self._end_requested.is_set()
        reason = self._end_reason if requested else "mission canceled"
        self.call_monitor("abort_routine", routine_name=routine_name, reason=reason)
        if requested:
            self.__report(f"Mission {self._mission_id} ended: {reason}", error=True)
            return self.__finish(
                goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_FAILED
            )
        self.__report(f"Mission {self._mission_id} canceled")
        return self.__finish(
            goal_handle, result, MultiGoalPlanPathAction.Result.OUTCOME_CANCELED
        )

    def __fill_progress(
        self, result, cursor: Dict[str, Any], total: int, waypoints: List[Pose]
    ) -> None:
        """Record how far the mission got, from the cursor"""
        result.reached_waypoints = reached_waypoints(cursor, total)
        reached = [i for i, done in enumerate(result.reached_waypoints) if done]
        result.last_reached_index = reached[-1] if reached else -1
        if result.last_reached_index >= 0:
            self.__fill_end_displacement(
                result, waypoints[result.last_reached_index]
            )

    def __fill_end_displacement(self, result, waypoint: Pose) -> None:
        """Record how far the robot ended up from the last waypoint reached.

        Measured here rather than taken from the planner, whose per-waypoint
        results the routine does not carry back
        """
        pose = self.start_pose()
        if pose is None:
            return
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

        if suffix.isdigit():
            index = int(suffix)
            feedback.current_goal_idx = index
            if index < total:
                feedback.current_goal = waypoints[index]
        return feedback
