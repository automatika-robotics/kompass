"""Callback classes used to process input topics data for supported types in Kompass"""

from typing import Optional
import numpy as np

from ros_sugar.io import GenericCallback, OccupancyGridCallback
from ros_sugar.io import OdomCallback as BaseOdomCallback
from ros_sugar.io import PointCallback as BasePointCallback
from ros_sugar.io import PoseCallback as BasePoseCallback
from ros_sugar.io import LaserScanCallback, PointCloudCallback
from kompass_core.models import RobotState

from nav_msgs.msg import Odometry
from tf2_ros import TransformStamped
from geometry_msgs.msg import Point, Pose

from ._external_types import (
    DetectionsCallback,
    TrackingsCallback,
    PointsOfInterestCallback,
)


__all__ = [
    "OdomCallback",
    "PointCallback",
    "PointStampedCallback",
    "PoseCallback",
    "PoseStampedCallback",
    "LaserScanCallback",
    "PointCloudCallback",
    "OccupancyGridCallback",
    "TrackingsCallback",
    "DetectionsCallback",
    "PointsOfInterestCallback",
]


class OdomCallback(BaseOdomCallback):
    """
    ROS2 Odometry Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        get_front: Optional[bool] = False,
        robot_radius: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param transform:
        :type transform: Optional[TransformStamped]
        :param get_front:
        :type get_front: Optional[bool]
        :param robot_radius:
        :type robot_radius: Optional[float]
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self._get_front = get_front
        self._robot_radius = robot_radius

    @property
    def get_front(self) -> Optional[bool]:
        """get_front.

        :rtype: Optional[bool]
        """
        return self._get_front

    @get_front.setter
    def get_front(self, value: bool):
        """
        Select to get the coordinates of the front of the robot instead of the center

        :param value:
        :type value: bool
        """
        self._get_front = value

    def _get_output(
        self, transformation: Optional[TransformStamped] = None, **_
    ) -> Optional[RobotState]:
        """
        Gets the RobotState by applying the transform to the odometry message if given.
        :returns:   Topic content
        :rtype:     Any
        """
        if not self.msg:
            return None
        if transformation or self.transformation:
            # Apply transformation or self.transformation
            # If both transformation and self.transformation are provided -> apply transformation
            transformed_odom: Odometry = self._transform(
                self.msg, transformation or self.transformation
            )
            # Process to get Odometry to RobotState
            center_state: RobotState = self._process(transformed_odom)
        else:
            center_state = self._process(self.msg)

        if self._get_front and self._robot_radius:
            # Get the state of the front of the robot
            front_state = center_state.front_state_from_center_state(self._robot_radius)
            return front_state
        return center_state

    def _process(self, msg: Odometry) -> RobotState:
        """
        Takes Odometry ROS object and converts it to RobotState
        """
        trans_heading = 2 * np.arctan2(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )

        speed = np.sqrt(msg.twist.twist.linear.y**2 + msg.twist.twist.linear.y**2)

        return RobotState(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=trans_heading,
            vx=msg.twist.twist.linear.x,
            vy=msg.twist.twist.linear.y,
            omega=msg.twist.twist.angular.z,
            speed=speed,
        )


class PointCallback(BasePointCallback):
    """
    ROS2 Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        get_front: Optional[bool] = False,
        robot_radius: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param get_front:
        :type get_front: Optional[bool]
        :param robot_radius:
        :type robot_radius: Optional[float]
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self._get_front = get_front
        self._robot_radius = robot_radius

    def _get_output(self, **_) -> Optional[RobotState]:
        """
        Gets the RobotState by applying the transform to the odometry message if given.
        :returns:   Topic content
        :rtype:     Any
        """
        if not self.msg:
            return None

        center_state = self._process(self.msg)

        if self._get_front and self._robot_radius:
            # Get the state of the front of the robot
            robot_state = center_state.front_state_from_center_state(self._robot_radius)
            return robot_state
        return center_state

    def _process(self, msg: Point) -> RobotState:
        """
        Takes Odometry ROS object and converts it to RobotState
        """
        return RobotState(x=msg.x, y=msg.y)


class PointStampedCallback(PointCallback):
    """
    ROS2 Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        get_front: Optional[bool] = False,
        robot_radius: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param get_front:
        :type get_front: Optional[bool]
        :param robot_radius:
        :type robot_radius: Optional[float]
        :rtype: None
        """
        super().__init__(input_topic, node_name, get_front, robot_radius)

    def _get_output(self, **_) -> Optional[RobotState]:
        """
        Gets the RobotState by applying the transform to the odometry message if given.
        :returns:   Topic content
        :rtype:     Any
        """
        if not self.msg:
            return None

        center_state = self._process(self.msg.point)

        if self._get_front and self._robot_radius:
            # Get the state of the front of the robot
            robot_state = center_state.front_state_from_center_state(self._robot_radius)
            return robot_state
        return center_state


class PoseCallback(BasePoseCallback):
    """
    ROS2 Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        get_front: Optional[bool] = False,
        robot_radius: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param get_front:
        :type get_front: Optional[bool]
        :param robot_radius:
        :type robot_radius: Optional[float]
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self._get_front = get_front
        self._robot_radius = robot_radius

    def _get_output(
        self, transformation: Optional[TransformStamped] = None, **_
    ) -> Optional[RobotState]:
        """
        Gets the RobotState by applying the transform to the odometry message if given.
        :returns:   Pose message as a state
        :rtype:     Optional[RobotState]
        """
        if not self.msg:
            return None

        # If a transform is given apply it to the message
        if transformation or self.transformation:
            pose_transformed = self._transform(
                self.msg, transformation or self.transformation
            )
            center_state = self._process(pose_transformed)
        else:
            center_state = self._process(self.msg)

        if self._get_front and self._robot_radius:
            # Get the state of the front of the robot
            front_state = center_state.front_state_from_center_state(self._robot_radius)
            return front_state
        return center_state

    def _process(self, msg: Pose) -> RobotState:
        """Takes Pose ROS object and converts it to a numpy array with [x, y, z, heading]

        :param msg: Input ROS Pose message
        :type msg: Pose

        :return: [x, y, z, heading]
        :rtype: np.ndarray
        """
        heading = 2 * np.arctan2(msg.orientation.z, msg.orientation.w)

        position = msg.position

        return RobotState(x=position.x, y=position.y, yaw=heading)


class PoseStampedCallback(PoseCallback):
    """
    ROS2 PoseStamped Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        get_front: Optional[bool] = False,
        robot_radius: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param get_front:
        :type get_front: Optional[bool]
        :param robot_radius:
        :type robot_radius: Optional[float]
        :rtype: None
        """
        super().__init__(input_topic, node_name, get_front, robot_radius)

    def _get_output(
        self, transformation: Optional[TransformStamped] = None, **_
    ) -> Optional[RobotState]:
        """
        Gets the RobotState by applying the transform to the odometry message if given.
        :returns:   Topic content
        :rtype:     Any
        """
        if not self.msg:
            return None

        # If a transform is given apply it to the message
        if transformation or self.transformation:
            pose_transformed = self._transform(
                self.msg.pose, transformation or self.transformation
            )
            center_state = self._process(pose_transformed)
        else:
            center_state = self._process(self.msg.pose)

        if self._get_front and self._robot_radius:
            # Get the state of the front of the robot
            robot_state = center_state.front_state_from_center_state(self._robot_radius)
            return robot_state
        return center_state


class TwistStampedCallback(GenericCallback):
    def __init__(self, input_topic, node_name=""):
        super().__init__(input_topic, node_name)

    def _get_output(self, **_):
        return self.msg.twist if self.msg else None
