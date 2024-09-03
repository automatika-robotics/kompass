from enum import Enum
from typing import Dict, List, Optional, Union

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ros_sugar.monitor import Monitor as BaseMonitor

from .actions import Action
from .components.component import Component
from .config import BaseConfig
from .event import Event


class MotionStatus(Enum):
    """Robot motion status to track if the motion is healthy or the robot is frozen or blocked"""

    HEALTHY = "Healthy"
    FREEZING = "Freezing Robot - Emergency stop is ON"
    ERROR_LOW_LEVEL = "Low-Level Error - Commands not getting executed by robot"
    ERROR_CONTROLLER = "Controller is not sending motion commands"
    ERROR_PLANNER = "No plan is available to execute"
    ERROR_ODOM = "Robot Odometry is not getting published"


class Monitor(BaseMonitor):
    """
    Kompass Monitor extends the base monitor to handle robot blocking monitoring during navigation
    """

    def __init__(
        self,
        componenets_names: List[str],
        enable_health_status_monitoring: bool = True,
        events: Optional[List[Event]] = None,
        actions: Optional[Dict[str, Action]] = None,
        config: Optional[BaseConfig] = None,
        services_components: Optional[List[Component]] = None,
        action_servers_components: Optional[List[Component]] = None,
        activate_on_start: Optional[List[Component]] = None,
        start_on_init: bool = False,
        component_name: str = "monitor",
        callback_group: Optional[
            Union[MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup]
        ] = None,
        **kwargs,
    ):
        super().__init__(
            componenets_names=componenets_names,
            enable_health_status_monitoring=enable_health_status_monitoring,
            events=events,
            actions=actions,
            config=config,
            services_components=services_components,
            action_servers_components=action_servers_components,
            activate_on_start=activate_on_start,
            start_on_init=start_on_init,
            component_name=component_name,
            callback_group=callback_group,
            **kwargs,
        )

    def init_flags(self):
        """
        Set up node flags
        """
        self.velocity_getting_published = False
        self.path_getting_published = False
        self.odom_getting_published = False
        self.emergency_stop_on = False

    def init_variables(self):
        """
        Set up node variables
        """

        self.motion_status = MotionStatus()
        self.previous_odometry = None
        self.current_odometry = None

    # TODO: refactor old monitor
    # def create_all_subscribers(self):
    #     """
    #     create all subscribers

    #     Currently the node subcribes to three messages:
    #     1) velocity command
    #     2) odometry / position message
    #     3) path message
    #     """
    #     self.create_subscription(
    #         Twist,
    #         self.params.robot_cmd_topic,
    #         self._velocity_callback,
    #     )

    #     self.create_subscription(
    #         Odometry,
    #         self.params.robot_odom_topic,
    #         self._odometry_callback,
    #     )

    #     self.create_subscription(
    #         Path,
    #         self.params.path_topic,
    #         self._path_callback,
    #     )

    #     self.create_subscription(
    #         Bool,
    #         self.params.emergency_stop_topic,
    #         self._emergency_stop_callback,
    #     )

    # def create_all_publishers(self):
    #     """
    #     create all publishers

    #     currently, the node publish one message -> Blocking status
    #     """
    #     self.motion_check_pub = self.create_publisher(
    #         MotionStatus, self.params.motion_status_topic
    #     )
    #     Component.create_all_publishers(self)

    # def _velocity_callback(self, msg: Twist):
    #     """
    #     callback tp notify that the velocity has been published

    #     :param      msg: velocity command message
    #     :type       msg: Twist
    #     """
    #     self.velocity_getting_published = True

    # def _path_callback(self, msg: Path):
    #     """
    #     Callback to notify that there's a path is getting executed

    #     :param      msg: path ros message
    #     :type       msg: Path
    #     """
    #     self.path_getting_published = True

    # def _emergency_stop_callback(self, msg: Bool):
    #     """
    #     Callback to check if the emergency stop is turned on

    #     :param      msg: Emegency stop status
    #     :type       msg: Bool
    #     """
    #     self.emergency_stop_on = msg.data

    # def _odometry_callback(self, msg: Odometry):
    #     """
    #     call back to save the current odometery

    #     :param      msg: Odometry ros message
    #     :type       msg: Odometry
    #     """
    #     self.odom_getting_published = True
    #     self.current_odometry = PoseData()
    #     self.current_odometry.set_pose(
    #         x=msg.pose.pose.position.x,
    #         y=msg.pose.pose.position.y,
    #         z=msg.pose.pose.position.z,
    #         qw=msg.pose.pose.orientation.w,
    #         qx=msg.pose.pose.orientation.x,
    #         qy=msg.pose.pose.orientation.y,
    #         qz=msg.pose.pose.orientation.z,
    #     )

    # def main(self):
    #     """
    #     Continous health check callback
    #     """
    #     # Get current status
    #     motion_status = self.motion_check()

    #     self.previous_odometry = self.current_odometry

    #     # reset flags
    #     self.velocity_getting_published = False
    #     self.path_getting_published = False
    #     self.odom_getting_published = False

    #     self.motion_status.status = motion_status
    #     self.motion_status.message = motion_status_msg[self.motion_status.status]
    #     self.motion_check_pub.publish(self.motion_status)

    # def motion_check(self) -> int:
    #     """
    #     Check the current status of the robot navigation

    #     0 STATUS_HEALTHY ---> MOTION COMMANDS ARE GETTING PUBLISHED AND EXECUTED BY THE ROBOT

    #     1 STATUS_FREEZING_ROBOT ---> PHYSICAL OBSTACLE IS BLOCKING THE ROBOT WAY AND THE ROBOT IS FROSEN

    #     2 STATUS_LOW_LEVEL_ERROR ---> ROBOT IS NOT MOVING WHILE THE MANAGER IS SENDING VELOCITY COMMANDS AND THERE IS NO PHYSICAL OBSTACLE

    #     3 STATUS_CONTROLLER_ERROR ---> CONTROLLER IS NOT SENDING VELOCITY COMMANDS

    #     4 STATUS_PATH_FOLLOWER_ERROR ---> PATH FOLLOWER IS NOT SENDING REFERENCE PATH COMMANDS
    #     """
    #     self.motion_status.status = MotionStatus.STATUS_HEALTHY

    #     if not self.odom_getting_published:
    #         return MotionStatus.STATUS_ODOMETRY_NOT_RECEIVED

    #     # TODO: treat case where path is read from file -> no topic
    #     # elif not self.path_getting_published:
    #     # return MotionStatus.STATUS_PATH_FOLLOWER_ERROR

    #     # check if the controller is not sending velocity commands
    #     elif not self.velocity_getting_published:
    #         return MotionStatus.STATUS_CONTROLLER_ERROR

    #     # check if the robot is not moving while velocity commands getting published
    #     elif self.previous_odometry:
    #         diff_in_position, diff_in_orientation = diff_between_poses(
    #             pose_a=self.previous_odometry,
    #             pose_b=self.current_odometry,
    #             in_2d=True,
    #         )

    #         if diff_in_position <= self.params.translation_threshold:
    #             if self.emergency_stop_on:
    #                 return MotionStatus.STATUS_FREEZING_ROBOT
    #             # Commands being sent, Robot is not moving and the emergency stop is not on -> low-level problem outside of Kompass
    #             else:
    #                 return MotionStatus.STATUS_LOW_LEVEL_ERROR

    #     return MotionStatus.STATUS_HEALTHY


# @attr.s
# class HealthMonitorConfig(ComponentConfig):
#     """
#     Health monitor config class
#     """

#     check_frequency: float = CommonUtils.set_attr(
#         float, default_value=0.20, min_value=1e-3, max_value=100.0
#     )

#     translation_threshold: float = CommonUtils.set_attr(
#         float, default_value=0.03, min_value=1e-4, max_value=100.0
#     )  # in meter

#     motion_status_topic: str = CommonUtils.set_attr(str, default_value="/motion_status")

#     path_topic: str = CommonUtils.set_attr(
#         str, default_value="/path_server/global_path"
#     )
