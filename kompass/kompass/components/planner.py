from typing import Dict, Optional
import time
import numpy as np
from attrs import field

# ROS INTERFACES
from geometry_msgs.msg import PoseStamped
from kompass_core.utils.geometry import from_euler_to_quaternion
from nav_msgs.msg import Path

# KOMPASS NAVIGATION
from kompass_core.models import Robot, RobotState
from kompass_core.third_party.ompl.planner import OMPLGeometric

# KOMPASS INTERFACES
from kompass_interfaces.action import PlanPath as PlanPathAction
from kompass_interfaces.msg import PathTrackingError
from kompass_interfaces.srv import PathFromToFile, StartPathRecording
from kompass_interfaces.srv import PlanPath as PlanPathSrv

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..data_types import Path as KompassPath
from .ros import (
    Topic,
    update_topics,
)
from .component import Component
from .defaults import (
    TopicsKeys,
    planner_allowed_inputs,
    planner_allowed_outputs,
    planner_default_inputs,
    planner_default_outputs,
)


class PlannerConfig(ComponentConfig):
    """
    Planner component config parameters
    """

    points_per_meter: int = field(
        default=10, validator=BaseValidators.in_range(min_value=1, max_value=1e9)
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
    Set from PlannerConfig class or directly from Planner 'run_type' property.

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

    def config_from_yaml(self, config_file: str):
        """
        Configure the planner node and the algorithm from file

        :param config_file: Yaml file
        :type config_file: str
        """
        super().config_from_yaml(config_file)
        if hasattr(self, "ompl_planner"):
            self.ompl_planner.configure(config_file, self.node_name)

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.goal: Optional[RobotState] = None
        self.robot_state: Optional[RobotState] = None
        self.map: Optional[np.ndarray] = None
        self.map_data: Optional[Dict] = None
        self.reached_end: bool = False

        self.__robot = Robot(
            robot_type=self.config.robot.model_type,
            geometry_type=self.config.robot.geometry_type,
            geometry_params=self.config.robot.geometry_params,
        )

        # Init OMPL with collision checking
        self.ompl_planner = OMPLGeometric(robot=self.__robot)

        if self._config_file:
            self.config_from_yaml(self._config_file)

        # Path and ROS path message
        self.path = None
        self.recorded_motion = None
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

    def _clear_path(self, *_, **__):
        """
        Clear the last computed path
        """
        if hasattr(self, "ompl_planner"):
            self.ompl_planner.clear()
        self.path = None
        # Set last path cost to inf to clear last path
        self._last_path_cost = float("inf")

    def _attach_callbacks(self):
        """
        Attaches planning method to goal_point topic callback if run_type is event based
        """
        if self.run_type == ComponentRunType.EVENT:
            self.get_logger().info(
                "Attaching Planning Callback for Event-Based Planner Component"
            )
            self.attach_custom_callback(
                self.get_in_topic(TopicsKeys.GOAL_POINT), self._plan_on_goal
            )

        if self.run_type in [ComponentRunType.EVENT, ComponentRunType.TIMED]:
            self.attach_custom_callback(
                self.get_in_topic(TopicsKeys.GOAL_POINT), self._clear_path
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
            self.get_logger().warn(
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
        # self.reached_end = dist <= tolerance.lateral_distance_error
        # self.get_publisher(PlannerOutputs.REACHED_GOAL.key).publish(self.reached_end)
        return dist <= tolerance.lateral_distance_error

    def main_action_callback(self, goal_handle: PlanPathAction.Goal):
        """
        Callback for the planner main action server

        :param goal_handle: Incoming action goal
        :type goal_handle: PlanPathAction.Goal

        :return: Action result
        :rtype: PlanPathAction.Result
        """
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

        action_feedback_msg.plan = Path()

        while not self.got_all_inputs(
            inputs_to_check=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
        ):
            self.get_logger().warn(
                f"Location input topic '{self.in_topic_name(TopicsKeys.ROBOT_LOCATION)}' is not available, waiting...",
                once=True,
            )

        try:
            while not self.reached_point(goal_state, end_goal_tolerance):
                # update state from input
                self._update_state()

                # plan a new path
                got_plan = self._plan(self.robot_state, goal_state)

                if got_plan and self.ros_path:
                    # publish feedback
                    action_feedback_msg.plan = self.ros_path
                else:
                    action_feedback_msg.plan = None
                goal_handle.publish_feedback(action_feedback_msg)
                self.get_logger().info(f"Action Feedback: {action_feedback_msg}")
                # NOTE: using Python time directly, as ros rate sleep (from self.create_rate) was not functioning as expected
                time.sleep(1 / self.config.loop_rate)

        except Exception as e:
            self.get_logger().error(f"Action execution error - {e}")
            goal_handle.abort()
            goal_handle.reset()

        # Get the displacement at the end of the path
        end_state_error: RobotState = self.robot_state - goal_state
        action_result.end_displacement.lateral_distance_error = abs(end_state_error)
        action_result.end_displacement.orientation_error = end_state_error.yaw

        action_result.reached_end = True
        self.get_logger().error(
            f"End Goal Reached with result {action_result} -> Ending Action"
        )
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
                    f"OMPL find to find a solution. Got exception: {e}"
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
                    self.path = self.ompl_planner.simplify_solution()
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
            self.ros_path.header.frame_id = self.config.frames.world
            self.ros_path.header.stamp = self.get_ros_time()
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
                self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(self.ros_path)

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

        self.goal: Optional[RobotState] = self.get_callback(
            TopicsKeys.GOAL_POINT
        ).get_output()

    def _plan_on_goal(self, msg, **_):
        """
        Generate a new plan on a msg published to goal_point topic

        :param goal: Goal point topic message
        :type goal: RobotState
        """
        # Clear any previous path
        self._clear_path()

        self._update_state()

        goal_state = RobotState(x=msg.point.x, y=msg.point.y)

        if self.got_all_inputs(
            inputs_to_check=[self.in_topic_name(TopicsKeys.ROBOT_LOCATION)]
        ):
            self._plan(self.robot_state, goal_state)
        else:
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

        if self.run_type == ComponentRunType.TIMED and self.goal and self.robot_state:
            self._plan(self.robot_state, self.goal)

    # SERVICES CALLBACKS
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
        if self._recording_on:
            KompassPath = self.recorded_motion
        else:
            KompassPath = self.ros_path

        if KompassPath:
            self.get_logger().info(f"Saving path of {len(self.ros_path.poses)} points")

            KompassPath.to_json(
                self.ros_path, json_file=f"{request.file_location}/{request.file_name}"
            )

            # Log saved path and return service response
            self.get_logger().info(
                f"Path saved to {request.file_location}/{request.file_name}"
            )
            response.path_length = len(KompassPath.poses)
            response.global_path = KompassPath

            if self._recording_on:
                self._recording_on = False
                self.recorded_motion = None

        else:
            self.get_logger().warning("No plan is available to save")

            # Return response with empty path of length zero
            response.path_length = 0.0
            response.global_path = Path()

        return response

    def _start_path_recording_callback(
        self, request: StartPathRecording.Request, response: StartPathRecording.Response
    ) -> StartPathRecording.Response:
        self.get_logger().info("RECEIVED RECORDING PATH FROM MOTION SERVICE REQUEST")
        if self.robot_state:
            self.recorded_motion = Path()
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
            self.recorded_motion = None
            self._recording_on = False
            response.started_recording = False
            response.message = f"Cannot start recording. Location not available on topic: {TopicsKeys.ROBOT_LOCATION}"
        return response

    def __record_new_point_callback(self):
        """Timer callback to record new motion point while recording is on"""
        if not self._recording_on:
            self.destroy_timer(self.__motion_recording_timer)
        new_pose = PoseStamped()
        new_pose.pose.position.x = self.robot_state.x
        new_pose.pose.position.y = self.robot_state.y
        new_pose.pose.orientation.z = np.sin(self.robot_state.yaw / 2)
        new_pose.pose.orientation.w = np.cos(self.robot_state.yaw / 2)
        self.recorded_motion.poses.append(new_pose)

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
        self.ros_path = KompassPath.from_json(
            f"{request.file_location}/{request.file_name}"
        )
        if self.ros_path:
            # Log saved path and return service response
            self.get_logger().info(
                f"Path loaded from {request.file_location}/{request.file_name}"
            )
            response.path_length = len(self.ros_path.poses)
            response.global_path = self.ros_path

            self.get_publisher(TopicsKeys.GLOBAL_PLAN).publish(self.ros_path)

        else:
            self.get_logger().warning("Invalid file -> No plan is loaded")

            # Return response with empty path of length zero
            response.path_length = 0.0
            response.global_path = Path()

        return response
