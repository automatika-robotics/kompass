from typing import Any, Dict, Optional
import time
import numpy as np
from attrs import field, define

# ROS INTERFACES
from geometry_msgs.msg import PoseStamped
from kompass_core.utils.geometry import from_euler_to_quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header

# KOMPASS NAVIGATION
from kompass_core.models import Robot, RobotState
from kompass_core.vision import DepthDetector
from kompass_core.third_party.ompl.planner import OMPLGeometric

# KOMPASS INTERFACES
from kompass_interfaces.action import PlanPath as PlanPathAction
from kompass_interfaces.msg import PathTrackingError
from kompass_interfaces.srv import PathFromToFile, StartPathRecording
from kompass_interfaces.srv import PlanPath as PlanPathSrv

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..data_types import Path as KompassPath
from ..callbacks import PointsOfInterestCallback, DetectionsCallback
from .ros import Topic, update_topics, ActionClientHandler
from .component import Component
from .defaults import (
    TopicsKeys,
    planner_allowed_inputs,
    planner_allowed_outputs,
    planner_default_inputs,
    planner_default_outputs,
)


@define
class PlannerConfig(ComponentConfig):
    """
    Planner component config parameters
    """

    points_per_meter: int = field(
        default=10, validator=BaseValidators.in_range(min_value=1, max_value=1e9)
    )
    distance_tolerance: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=0.0, max_value=1e9)
    )
    depth_conversion_factor: float = field(
        default=1e-3, validator=BaseValidators.in_range(min_value=1e-6, max_value=1e3)
    )  # Depth conversion factor to convert depth image values to meters, default is 1e-3 for millimeters to meters
    depth_range: np.ndarray = field(
        default=np.array([0.1, 10.0], dtype=np.float32),
        validator=BaseValidators.array_shape((2,), dtype=np.float32),
        # Automatically cast incoming lists/arrays to float32
        converter=lambda x: np.array(x, dtype=np.float32),
    )


class Planner(Component):
    """
    Planner Component used for path planning during navigation.

    ## Input Topics:
    - *map_layer*: Global map used for planning.<br />
                 Default: ``` Topic(name="/map", msg_type="OccupancyGrid" qos_profile=QoSConfig(durability=qos.DurabilityPolicy.TRANSIENT_LOCAL))```
    - *location*: the robot current location.<br /> Default ```Topic(name="/odom", msg_type="Odometry")```
    - *goal_point*: 2D navigation goal point on the map.<br /> Default ``` Topic(name="/goal", msg_type="PointStamped") ```

    ## Outputs:
    - *plan*: Path to reach the goal point from start location.<br /> Default ``` Topic(name="/plan", msg_type="Path")```
    - *reached_end*: Flag indicating that the current planning target is reached<br /> Default ``` Topic(name="/reached_end", msg_type="Bool")```

    ## Available Run Types:
    Set directly from Planner 'run_type' property.

    - *TIMED*: Compute a new plan periodically from current location (last message received on location input Topic) to the goal location (last message received on goal_point input Topic)
    - *EVENT*: Compute a new plan from current location on every new message received on goal_point input Topic
    - *SERVER*: Offers a PlanPath ROS service and computes a new plan on every service request
    - *ACTIONSERVER*: Offers a PlanPath ROS action and continuously computes a plan once an action request is received until goal point is reached


    ## Usage Example:
    ```python
        from kompass.components import Planner, PlannerConfig
        from kompass.config import ComponentRunType
        from kompass_core.models import RobotType, Robot, LinearCtrlLimits, AngularCtrlLimits
        import numpy as np

        # Configure your robot
        my_robot = RobotConfig(
                    model_type=RobotType.DIFFERENTIAL_DRIVE,
                    geometry_type=RobotGeometry.Type.CYLINDER,
                    geometry_params=np.array([0.1, 0.3]),
                    ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5),
                    ctrl_omega_limits=AngularCtrlLimits(
                        max_vel=1.0, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
                    ),
                )

        # Setup the planner config
        config = PlannerConfig(
            robot=my_robot,
            loop_rate=1.0
        )

        planner = Planner(component_name="planner", config=config)

        # Add rviz clicked_point as input topic
        planner.run_type = ComponentRunType.EVENT   # Can also pass a string "Event"
        goal_topic = Topic(name="/clicked_point", msg_type="PoseStamped")
        planner.inputs(goal_point=goal_topic)
    ```

    """

    def __init__(
        self,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[PlannerConfig] = None,
        inputs: Optional[Dict[str, Topic]] = None,
        outputs: Optional[Dict[str, Topic]] = None,
        **kwargs,
    ) -> None:
        config = config or PlannerConfig()

        # Update defaults from custom topics if provided
        in_topics = (
            update_topics(planner_default_inputs, **inputs)
            if inputs
            else planner_default_inputs
        )
        out_topics = (
            update_topics(planner_default_outputs, **outputs)
            if outputs
            else planner_default_outputs
        )

        super().__init__(
            config=config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=planner_allowed_inputs,
            allowed_outputs=planner_allowed_outputs,
            component_name=component_name,
            allowed_run_types=[
                ComponentRunType.TIMED,
                ComponentRunType.ACTION_SERVER,
                ComponentRunType.EVENT,
            ],
            **kwargs,
        )

        self.config: PlannerConfig = config

        # Main service and action types of the planner component
        self.service_type = PlanPathSrv
        self.action_type = PlanPathAction
        self.main_action_name = "navigate_to_goal"

    def inspect_component(self) -> str:
        """
        Method to return a string representation of the component configuration, used for inspection and LLM-based reasoning about the component

        :return: String representation of the component configuration
        :rtype: str
        """
        base_info = super().inspect_component()
        # Get all planning algorithms
        try:
            import omplpy as ompl
            from kompass_core.third_party.ompl.config import initializePlanners

            initializePlanners()
            planners = ompl.geometric.planners.getPlanners()
            planning_algorithms = "Available planning algorithms are:\n"
            for planner_name in planners.keys():
                planning_algorithms += f"- {planner_name}\n"
        except Exception:
            planning_algorithms = "Could not retrieve available planning algorithms.\n"

        action_goal_info = (
            f"If the Run Type is ACTION_SERVER, the Planner main action server is '{self.main_action_name}'.\n"
            "To construct a goal request:\n"
            "- 'goal' is a geometry_msgs/Pose representing the target location. "
            "When the user specifies a 2D point like 'go to (-4, 4)', map the first value to goal.position.x and the second to goal.position.y. "
            "Set goal.position.z to 0.0. If no orientation is given, use the identity quaternion (x=0, y=0, z=0, w=1.0).\n"
            "- 'algorithm_name' selects the planning algorithm. Leave it empty to use the currently configured default. Or send one of the planner's available algorithms names.\n"
            "- 'end_tolerance' defines when the goal is considered reached. "
            "lateral_distance_error is in meters, orientation_error is in radians. "
            "If not specified by the user, use a reasonable default (e.g. 0.1 m, 0.1 rad).\n"
        )

        services_info = (
            "Additional ROS services:\n"
            f"- '{self.node_name}/save_plan_to_file' (PathFromToFile): Save the current plan to a JSON file. "
            "Use this after a plan has been generated or a motion has been recorded. "
            "Provide 'file_location' (directory path) and 'file_name' (the JSON file name). "
            "Calling this while recording is active will stop the recording and save the recorded path.\n"
            f"- '{self.node_name}/load_plan_from_file' (PathFromToFile): Load a previously saved plan from a JSON file and publish it to the Controller. "
            "Use this to replay or reuse a saved path without re-planning. "
            "Provide 'file_location' and 'file_name' pointing to the saved JSON file.\n"
            f"- '{self.node_name}/start_path_recording' (StartPathRecording): Start recording the robot's actual motion as a path. "
            "Use this when the user wants to teach a path by manually driving the robot or capture the executed trajectory. "
            "Provide 'recording_time_step' (seconds between recorded points, e.g. 0.1). "
            "To stop recording and save, call the save_plan_to_file service.\n"
        )

        return (
            base_info
            + "\n"
            + planning_algorithms
            + "\n"
            + action_goal_info
            + "\n"
            + services_info
        )

    def get_ros_entrypoints(self) -> Dict[str, Dict[str, Any]]:
        """Get the component ROS entry points (additional services and actions) as a dictionary."""
        entry_points = {"services": {}, "actions": {}}
        entry_points["services"].update({
            f"{self.node_name}/save_plan_to_file": PathFromToFile,
            f"{self.node_name}/load_plan_from_file": PathFromToFile,
            f"{self.node_name}/start_path_recording": StartPathRecording,
        })
        return entry_points

    def config_from_file(self, config_file: str):
        """
        Configure the planner node and the algorithm from file

        :param config_file: Path to config file (yaml, json, toml)
        :type config_file: str
        """
        super().config_from_file(config_file)
        if hasattr(self, "ompl_planner"):
            self.ompl_planner.configure(config_file, self.node_name)

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.goal: Dict[int, RobotState] = {}
        self.robot_state: Optional[RobotState] = None
        self.map: Optional[np.ndarray] = None
        self.map_data: Optional[Dict] = None
        self.reached_end: bool = False
        self._depth_image_info: Optional[Dict] = None

        self.__robot = Robot(
            robot_type=self.config.robot.model_type,
            geometry_type=self.config.robot.geometry_type,
            geometry_params=self.config.robot.geometry_params,
        )

        # Init OMPL with collision checking
        self.ompl_planner = OMPLGeometric(
            robot=self.__robot, log_level=self.config.core_log_level
        )

        if self._config_file:
            self.config_from_file(self._config_file)

        # Path and ROS path message
        self.path = None
        self._recorded_motion = None
        self._recording_on = False
        self.ros_path = None

        self._attach_callbacks()

    def create_all_services(self):
        """
        Creates all node services
        """
        self.__save_plan_srv = self.create_service(
            PathFromToFile,
            f"{self.node_name}/save_plan_to_file",
            self._save_plan_to_file_srv_callback,
        )
        self.__load_plan_srv = self.create_service(
            PathFromToFile,
            f"{self.node_name}/load_plan_from_file",
            self._load_plan_from_file_srv_callback,
        )
        self.__record_motion_srv = self.create_service(
            StartPathRecording,
            f"{self.node_name}/start_path_recording",
            self._start_path_recording_callback,
        )
        # Call component service creation (for main service if running as a server)
        super().create_all_services()

    def destroy_all_services(self):
        """
        Destroys all node services
        """
        self.destroy_service(self.__save_plan_srv)
        self.destroy_service(self.__load_plan_srv)
        self.destroy_service(self.__record_motion_srv)
        # Call component service destruction (for main service if running as a server)
        super().destroy_all_services()

    def create_all_subscribers(self):
        super().create_all_subscribers()
        # If a vision goal input is provided -> Init the depth detector
        num_goal_inputs = self._inputs_keys.count(TopicsKeys.GOAL_POINT)
        for idx in range(num_goal_inputs):
            callback = self.get_callback(TopicsKeys.GOAL_POINT, idx)
            if isinstance(callback, PointsOfInterestCallback) or isinstance(
                callback, DetectionsCallback
            ):
                # Init the depth detector
                depth_detector = self.__setup_depth_detector()
                if not depth_detector:
                    self.get_logger().error(
                        "Failed to initialize depth detector for vision-based goals. Please ensure that the depth camera info topic is provided and the TF from depth camera to robot base can be obtained within the topic subscription timeout."
                    )
                    self.health_status.set_fail_system(
                        topic_names=[self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO)]
                    )
                else:
                    self.get_logger().debug(
                        f"Setting up Depth Detector for input goals on topic {callback.input_topic.name}"
                    )
                    callback.set_depth_detector(depth_detector)
                break

    def trigger_main_action_server(
        self,
        goal_x: float = 0.0,
        goal_y: float = 0.0,
        goal_orientation: float = 0.0,
        tolerance_dist: float = 0.1,
        tolerance_ori: float = 0.1,
        algorithm_name: Optional[str] = None,
        **_,
    ) -> None:
        """A component action to trigger the main planner action (Plan path to point until reached)

        :param goal_x: _description_, defaults to 0.0
        :type goal_x: float, optional
        :param goal_y: _description_, defaults to 0.0
        :type goal_y: float, optional
        :param tolerance_dist: _description_, defaults to 0.1
        :type tolerance_dist: float, optional
        :param tolerance_ori: _description_, defaults to 0.1
        :type tolerance_ori: float, optional
        :param algorithm_name: _description_, defaults to None
        :type algorithm_name: Optional[str], optional
        """
        if self.run_type != ComponentRunType.ACTION_SERVER:
            self.get_logger().error(
                f"Cannot trigger main action server for component '{self.node_name}' that is running in '{self.run_type}' execution"
            )
            return
        try:
            action_client = ActionClientHandler(
                client_node=self,
                action_name=self.main_action_name,
                action_type=self.action_type,
            )
            goal = self.action_type.Goal()
            goal.goal.position.x = goal_x
            goal.goal.position.y = goal_y
            goal.goal.orientation.z = float(np.sin(goal_orientation / 2.0))
            goal.goal.orientation.w = float(np.cos(goal_orientation / 2.0))
            goal.end_tolerance.lateral_distance_error = tolerance_dist
            goal.end_tolerance.orientation_error = tolerance_ori
            if algorithm_name:
                goal.algorithm_name = algorithm_name
            action_client.send_request(goal)
        except Exception as e:
            self.get_logger().error(
                f"Failed to trigger '{self.main_action_name}' action on '{self.node_name}': {e}"
            )
            self.health_status.set_fail_component()

    def _clear_path(self, *_, **__):
        """
        Clear the last computed path
        """
        self.path = None
        # Set last path cost to inf to clear last path
        self._last_path_cost = float("inf")

    def _attach_callbacks(self):
        """
        Attaches planning method to goal_point topic callback if run_type is event based
        """
        require_cam_info = False
        num_goal_inputs = self._inputs_keys.count(TopicsKeys.GOAL_POINT)
        if self.run_type == ComponentRunType.EVENT:
            self.get_logger().info(
                "Attaching Planning Callback for Event-Based Planner Component"
            )
            for idx in range(num_goal_inputs):
                callback = self.get_callback(TopicsKeys.GOAL_POINT, idx)
                if isinstance(callback, PointsOfInterestCallback) or isinstance(
                    callback, DetectionsCallback
                ):
                    # Points of interest callback requires camera info input topic to be set
                    require_cam_info = True
                callback.on_callback_execute(
                    lambda msg, _cb=callback, _idx=idx, **__: self._plan_on_goal(
                        msg, callback=_cb, goal_index=_idx
                    )
                )
        if require_cam_info and not self.got_input(TopicsKeys.DEPTH_CAM_INFO):
            raise ValueError(
                f"At least one of the goal point callbacks is a {callback.__class__.__name__} which requires depth camera info input. Please provide a topic for {TopicsKeys.DEPTH_CAM_INFO} to ensure proper functionality."
            )

    def main_service_callback(
        self, request: PlanPathSrv.Request, response: PlanPathSrv.Response
    ):
        """
        Path planning service callback

        :param request: Path planning request containing goal location
        :type request: PlanPath.Request
        :param response: Path planning response (success and plan)
        :type response: PlanPath.Response

        :return: Path planning response
        :rtype: PlanPath.Response
        """
        # Clear any previous path
        self._clear_path()

        self._update_state()
        goal_yaw = 2 * np.arctan2(
            request.goal.orientation.z, request.goal.orientation.w
        )

        goal_state = RobotState(
            x=request.goal.position.x, y=request.goal.position.y, yaw=goal_yaw
        )

        if request.from_robot_pose:
            start_state = self.robot_state
        else:
            # Get start from request
            start_yaw = 2 * np.arctan2(
                request.start.orientation.z, request.start.orientation.w
            )
            start_state = RobotState(
                x=request.start.position.x, y=request.start.position.y, yaw=start_yaw
            )

        if start_state:
            self._plan(start_state, goal_state, publish_path=True)
        else:
            self.get_logger().warning(
                "Cannot execute planning service, robot location is unavailable"
            )
            response.success = False
            return response

        if self.path:
            response.success = True
            response.plan = self.path
        else:
            response.success = False
        return response

    def reached_point(
        self, goal_point: RobotState, tolerance: PathTrackingError
    ) -> bool:
        """
        Checks if the current robot state is close to a given goal point

        :param goal_point: Goal point
        :type goal_point: RobotState
        :param tolerance: Tolerance to goal
        :type tolerance: PathTrackingError

        :return: If the distance to the goal is less than the given tolerance
        :rtype: bool
        """
        if not self.robot_state:
            return False
        dist: float = self.robot_state.distance(goal_point)
        return dist <= tolerance.lateral_distance_error

    def main_action_callback(self, goal_handle: PlanPathAction.Goal):
        """
        Callback for the planner main action server

        :param goal_handle: Incoming action goal
        :type goal_handle: PlanPathAction.Goal

        :return: Action result
        :rtype: PlanPathAction.Result
        """
        self.get_logger().info("Executing Action to plan path...")
        # Clear any previous path
        self._clear_path()

        self._update_state()

        # Get request
        end_goal_tolerance: PathTrackingError = goal_handle.request.end_tolerance

        # TODO: get planner id from the request
        # planner_id = goal_handle.request.algorithm_name

        # Set goal
        goal_yaw = 2 * np.arctan2(
            goal_handle.request.goal.orientation.z,
            goal_handle.request.goal.orientation.w,
        )

        goal_state = RobotState(
            x=goal_handle.request.goal.position.x,
            y=goal_handle.request.goal.position.y,
            yaw=goal_yaw,
        )

        # Setup response and feedback of the action
        action_feedback_msg = PlanPathAction.Feedback()

        action_result = PlanPathAction.Result()

        action_result.end_displacement = PathTrackingError()

        action_result.reached_end = False

        while not self.got_all_inputs(
            inputs_to_check=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
        ):
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info("Goal Canceled")
                return action_result
            self.get_logger().warning(
                f"Location input topic '{self.in_topic_name(TopicsKeys.ROBOT_LOCATION)}' is not available, waiting...",
                once=True,
            )
            time.sleep(1 / self.config.loop_rate)

        try:
            while not self.reached_point(goal_state, end_goal_tolerance):
                if not goal_handle.is_active or goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal Canceled")
                    return action_result

                # update state from input
                self._update_state()

                # plan a new path
                self._plan(self.robot_state, goal_state)

                action_feedback_msg.current_pose = self.__robot_state_to_pose_stamped()
                action_feedback_msg.reached_end = False

                goal_handle.publish_feedback(action_feedback_msg)
                self.get_logger().debug(f"Action Feedback: {action_feedback_msg}")
                # NOTE: using Python time directly, as ros rate sleep (from self.create_rate) was not functioning as expected
                time.sleep(1 / self.config.loop_rate)

        except Exception as e:
            self.get_logger().error(f"Action execution error - {e}")
            with self._main_goal_lock:
                goal_handle.abort()
            return action_result

        # Get the displacement at the end of the path
        end_state_error: RobotState = self.robot_state - goal_state
        action_result.end_displacement.lateral_distance_error = abs(end_state_error)
        action_result.end_displacement.orientation_error = end_state_error.yaw

        action_result.reached_end = True
        self.get_logger().info(
            f"End Goal Reached with result {action_result} -> Ending Action"
        )
        self.get_publisher(TopicsKeys.REACHED_END).publish(bool(True))
        # Publish empty path
        self.ros_path = Path()
        self.ros_path.header.frame_id = self.config.frames.world
        self.ros_path.header.stamp = self.get_ros_time()
        self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(self.ros_path)

        with self._main_goal_lock:
            goal_handle.succeed()

        return action_result

    def _plan(
        self, start: RobotState, goal: RobotState, publish_path: bool = True
    ) -> bool:
        """
        Plans and publishes a path from current robot location to current goal
        """
        # Check if all inputs are available
        # goal_point is excluded since goal can be provided by either a topic, service call or action goal
        if self.got_all_inputs(
            inputs_to_check=[self.in_topic_name(TopicsKeys.GLOBAL_MAP)]
        ):
            self.get_logger().debug(
                f"Setting planning problem with {self.ompl_planner.planner_id} from [{start.x},{start.y}] to [{goal.x}, {goal.y}] and map data {self.map_data}"
            )

            self.ompl_planner.setup_problem(
                self.map_data,
                start.x,
                start.y,
                start.yaw,
                goal.x,
                goal.y,
                goal.yaw,
                self.map,
            )

            try:
                # Solve the planning problem
                path = self.ompl_planner.solve()
            except Exception as e:
                self.get_logger().error(
                    f"OMPL failed to find a solution. Got exception: {e}"
                )
                self.health_status.set_fail_algorithm(
                    algorithm_names=[self.ompl_planner.planner_id]
                )
                return False

            if path:
                # Add cost as last cost if it does not exist
                if not hasattr(self, "_last_path_cost"):
                    self._last_path_cost = self.ompl_planner.path_cost
                current_cost = self.ompl_planner.path_cost
                self.get_logger().debug(f"Got new plan with cost: {current_cost}")
                # If the current cost is less -> Update the path
                # This is to prevent publishing less optimal paths produced by sampling methods
                if current_cost <= self._last_path_cost:
                    # Simplify solution
                    self.path = path
                    self._last_path_cost = current_cost

                self.health_status.set_healthy()

            if not self.path:
                self.get_logger().error("Planner failed to find a path!")
                # Failed to find a path -> algorithm failure
                self.health_status.set_fail_algorithm(
                    algorithm_names=[self.ompl_planner.planner_id]
                )
                return False
        else:
            missing_topics = self.get_missing_inputs()
            # Failed to plan due to missing input(s)
            self.get_logger().error(
                f"Cannot plan due to missing inputs {missing_topics}"
            )
            self.health_status.set_fail_system(topic_names=missing_topics)
            return False

        # If a plan was produced publish it
        if publish_path and self.path:
            self.ros_path = Path()
            points = self.path.getStates()

            for point in points:
                ros_point = PoseStamped()
                ros_point.header = self.ros_path.header
                ros_point.pose.position.x = point.getX()
                ros_point.pose.position.y = point.getY()
                q_rot = from_euler_to_quaternion(
                    yaw=point.getYaw(), pitch=0.0, roll=0.0
                )
                ros_point.pose.orientation.z = q_rot[3]
                ros_point.pose.orientation.w = q_rot[0]
                self.ros_path.poses.append(ros_point)
            try:
                self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(
                    self.ros_path, frame_id=self.config.frames.world
                )

            except Exception as e:
                self.get_logger().error(f"Publishing exception: {e}")

        return True

    def _update_state(self):
        """
        Updates all inputs
        """
        self.map: Optional[np.ndarray] = self.get_callback(
            TopicsKeys.GLOBAL_MAP
        ).get_output()

        self.robot_state: Optional[RobotState] = self.get_callback(
            TopicsKeys.ROBOT_LOCATION
        ).get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

        self.map_data: Optional[Dict] = self.get_callback(
            TopicsKeys.GLOBAL_MAP
        ).get_output(get_metadata=True)

        num_goal_inputs = self._inputs_keys.count(TopicsKeys.GOAL_POINT)
        for idx in range(num_goal_inputs):
            callback = self.get_callback(TopicsKeys.GOAL_POINT, idx)
            self.goal[idx] = callback.get_output(
                to_robot_state=True, robot_state=self.robot_state
            )

    def __setup_depth_detector(self) -> Optional[DepthDetector]:
        """Setup and configure a DepthDetector for usage with Vision-based goals"""
        timeout = 0.0
        # Get the depth image transform if the input is provided
        if self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO):
            while (
                not self.depth_tf_listener.got_transform or not self._depth_image_info
            ) and timeout <= self.config.topic_subscription_timeout:
                # Depth image cam info
                if not self._depth_image_info:
                    depth_img_info_callback = self.get_callback(
                        TopicsKeys.DEPTH_CAM_INFO
                    )
                    self._depth_image_info: Optional[dict] = (
                        depth_img_info_callback.get_output()
                        if depth_img_info_callback
                        else None
                    )
                time.sleep(1 / self.config.loop_rate)
                timeout += 1 / self.config.loop_rate
        else:
            self.get_logger().error(
                f"Depth camera info topic '{self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO)}' is not provided, cannot initialize Vision Depth Detector required for vision-based planner goals. Please provide an input topic for {TopicsKeys.DEPTH_CAM_INFO} to ensure proper functionality."
            )
            self.health_status.set_fail_system(
                topic_names=[self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO)]
            )
            return None

        if not self.depth_tf_listener.got_transform:
            self.get_logger().error(
                f"Could not obtain transformation between the Depth camera frame '{self.depth_tf_listener.config.source_frame}' to the robot body frame '{self.depth_tf_listener.config.goal_frame}'"
            )
            return None

        if not self._depth_image_info:
            self.get_logger().error(
                f"Depth camera info topic '{self.in_topic_name(TopicsKeys.DEPTH_CAM_INFO)}' did not publish any message. TIMEOUT."
            )
            return None

        self.get_logger().info(
            f"Got Depth camera to body TF -> {self._depth_image_info} Setting up Vision Depth Detector"
        )

        return DepthDetector(
            depth_range=self.config.depth_range,
            camera_in_body_translation=self.depth_tf_listener.translation,
            camera_in_body_rotation=self.depth_tf_listener.rotation,
            focal_length=self._depth_image_info["focal_length"]
            if self._depth_image_info
            else None,
            principal_point=self._depth_image_info["principal_point"]
            if self._depth_image_info
            else None,
            depth_conversion_factor=self.config.depth_conversion_factor,
        )

    def _plan_on_goal_core(
        self, goal_state: RobotState, goal_index: int = 0, **_
    ) -> bool:
        if (
            self.got_all_inputs(
                inputs_to_check=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
            )
            and not self.reached_end
        ):
            self._plan(self.robot_state, goal_state)
            self.reached_end = self.reached_point(
                goal_state,
                PathTrackingError(
                    lateral_distance_error=self.config.distance_tolerance
                ),
            )
            return True
        elif self.reached_end:
            self.get_logger().info("Goal Reached!")
            self.get_publisher(TopicsKeys.REACHED_END).publish(bool(True))
            self.reached_end = False
            self.goal[goal_index] = None
            self._clear_path()
            # Publish empty path to clear the plan in any subscribers
            path = Path()
            path.header.frame_id = self.config.frames.world
            path.header.stamp = self.get_ros_time()
            self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(path)
            return True
        return False

    def _plan_on_goal(self, msg, *, callback=None, goal_index: int = 0, **_) -> None:
        """
        Generate a new plan on a msg published to goal_point topic

        :param msg: Goal point topic message (PointStamped, PoseStamped, Detections2D, Trackings, etc.)
        :param callback: Callback that produced the message, used to convert the
            raw msg to a RobotState via its get_output(to_robot_state=True, ...)
            contract. If None, falls back to msg.point field access.
        :param goal_index: Index of the goal_point input when multiple are
            configured; used to key into self.goal[...] inside _plan_on_goal_core.
        """
        # Clear any previous path
        self._clear_path()

        self._update_state()

        if callback is not None:
            goal_state: Optional[RobotState] = callback.get_output(
                to_robot_state=True, robot_state=self.robot_state
            )
        else:
            goal_state = RobotState(x=msg.point.x, y=msg.point.y)

        if not goal_state:
            self.get_logger().error(
                "Could not extract goal state from incoming goal message"
            )
            return

        if not self._plan_on_goal_core(goal_state, goal_index=goal_index):
            # Robot location not known -> cannot plan
            self.get_logger().error(
                f"Got goal point {goal_state} but cannot plan. Robot location is not known"
            )
            self.health_status.set_fail_system(
                topic_names=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
            )

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        self._update_state()

        if (
            self.run_type == ComponentRunType.TIMED
            and any(self.goal.values())
            and self.robot_state
            and not self.reached_end
        ):
            for idx, goal_state in self.goal.items():
                self.get_logger().debug(f"Planning for goal {idx}: {goal_state}")
                self._plan_on_goal_core(goal_state, goal_index=idx)
        else:
            self.get_logger().debug(
                f"No planning executed for current step:\nGoal: {self.goal}, robot_state: {self.robot_state}, reached_end: {self.reached_end}",
                throttle_duration_sec=30.0,
            )

    # SERVICES CALLBACKS
    def _start_path_recording_callback(
        self, request: StartPathRecording.Request, response: StartPathRecording.Response
    ) -> StartPathRecording.Response:
        """Method executed on a new StartPathRecording incoming request

        :param request: Path recording service request
        :type request: StartPathRecording.Request
        :param response: Path recording service response
        :type response: StartPathRecording.Response
        :return: Updated path recording service response
        :rtype: StartPathRecording.Response
        """
        self.get_logger().info("RECEIVED RECORDING PATH FROM MOTION SERVICE REQUEST")
        if self.robot_state:
            self._recorded_motion = Path()
            self._recorded_motion.header.frame_id = self.config.frames.world
            self._recording_on = True
            recording_step = max(
                request.recording_time_step, 1e-3
            )  # Step should be greater than zero
            self.__motion_recording_timer = self.create_timer(
                recording_step,
                self.__record_new_point_callback,
            )
            response.started_recording = True
            response.message = (
                f"Started recording motion from odom topic {TopicsKeys.ROBOT_LOCATION}"
            )
        else:
            self._recorded_motion = None
            self._recording_on = False
            response.started_recording = False
            response.message = f"Cannot start recording. Location not available on topic: {TopicsKeys.ROBOT_LOCATION}"
            self.get_logger().info(f"Cannot record returning {response}")
        return response

    def __robot_state_to_pose_stamped(self) -> PoseStamped:
        new_pose = PoseStamped()
        new_pose.header = Header()
        new_pose.header.frame_id = self.config.frames.world
        new_pose.header.stamp = self.get_ros_time()
        new_pose.pose.position.x = float(self.robot_state.x)
        new_pose.pose.position.y = float(self.robot_state.y)
        new_pose.pose.orientation.z = float(np.sin(self.robot_state.yaw / 2))
        new_pose.pose.orientation.w = float(np.cos(self.robot_state.yaw / 2))
        return new_pose

    def __record_new_point_callback(self):
        """Timer callback to record new motion point while recording is on"""
        if not self._recording_on:
            self.destroy_timer(self.__motion_recording_timer)
            return
        self._update_state()
        new_pose = self.__robot_state_to_pose_stamped()
        self._recorded_motion.poses.append(new_pose)

    def _save_plan_to_file_srv_callback(
        self, request: PathFromToFile.Request, response: PathFromToFile.Response
    ) -> PathFromToFile.Response:
        """
        Saves current plan to JSON file if the plan is available

        :param request: ROS service request
        :type request: PathFromToFile.request
        :param response: ROS service response
        :type response: PathFromToFile.response
        :return: Response
        :rtype: PathFromToFile.response
        """
        self.get_logger().info("RECEIVED SAVE PATH SERVICE REQUEST")
        # If planning and recording is happening simultaneously -> will save the recorded motion
        if self._recording_on:
            path = self._recorded_motion
        else:
            # If no recording is on -> save the planned path
            path = self.ros_path

        if path:
            self.get_logger().info(f"Saving path of {len(path.poses)} points")

            KompassPath.to_json(
                path, json_file=f"{request.file_location}/{request.file_name}"
            )

            # Log saved path and return service response
            self.get_logger().info(
                f"Path saved to {request.file_location}/{request.file_name}"
            )
            response.path_length = KompassPath.length(path)
            response.path_num_points = len(path.poses)

            if self._recording_on:
                self._recording_on = False
                self._recorded_motion = None

        else:
            self.get_logger().warning("No plan or motion points are available to save")

            # Return response with empty path of length zero
            response.path_length = 0.0
            response.path_num_points = 0

        return response

    def _load_plan_from_file_srv_callback(
        self, request: PathFromToFile.Request, response: PathFromToFile.Response
    ) -> PathFromToFile.Response:
        """
        Loads ROS path message from json file

        :param request: ROS service request
        :type request: PathFromToFile.request
        :param response: ROS service response
        :type response: PathFromToFile.response
        :return: Response
        :rtype: PathFromToFile.response
        """
        self.get_logger().info("RECEIVED LOAD PATH SERVICE REQUEST")
        path = KompassPath.from_json(f"{request.file_location}/{request.file_name}")
        if path:
            self.ros_path = path
            # Log saved path and return service response
            self.get_logger().info(
                f"Path loaded from {request.file_location}/{request.file_name}"
            )
            response.path_num_points = len(self.ros_path.poses)
            response.path_length = KompassPath.length(self.ros_path)

            self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(
                self.ros_path, frame_id=self.ros_path.header.frame_id
            )

        else:
            self.get_logger().warning("Invalid file -> No plan is loaded")

            # Return response with empty path of length zero
            response.path_length = 0.0
            response.path_num_points = 0

        return response
