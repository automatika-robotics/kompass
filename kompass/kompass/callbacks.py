"""Callback classes used to process input topics data for supported types in Kompass"""

from typing import Optional

import numpy as np
from ros_sugar.callbacks import GenericCallback, OccupancyGridCallback
from ros_sugar.callbacks import OdomCallback as BaseOdomCallback
from ros_sugar.callbacks import PointCallback as BasePointCallback
from ros_sugar.callbacks import PoseCallback as BasePoseCallback
from geometry_msgs.msg import Point, Pose
from kompass_core.datatypes.laserscan import LaserScanData
from kompass_core.datatypes.pointcloud import PointCloudData
from .utils import read_pc_points, read_pc_points_with_tf
from kompass_core.utils import geometry as GeometryUtils
from nav_msgs.msg import Odometry
from kompass_core.models import RobotState
from sensor_msgs.msg import LaserScan, PointCloud2

from tf2_ros import TransformStamped


__all__ = [
    "OdomCallback",
    "PointCallback",
    "PointStampedCallback",
    "PoseCallback",
    "PoseStampedCallback",
    "LaserScanCallback",
    "OccupancyGridCallback",
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

        # If a trnsform is given apply it to the message
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

        # If a trnsform is given apply it to the message
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


class LaserScanCallback(GenericCallback):
    """ROS2 LaserScan Callback Handler to process and transform sensor_msgs/LaserScan data"""

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        transformation: Optional[TransformStamped] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param transformation:
        :type transformation: Optional[TransformStamped]
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self.__tf = transformation

    @property
    def transformation(self) -> Optional[TransformStamped]:
        """Getter of the laserscan transformation

        :return: Frame transformation
        :rtype: Optional[TransformStamped]
        """
        return self.__tf

    @transformation.setter
    def transformation(self, transform: TransformStamped):
        """
        Sets a new transformation value

        :param transform: Detected transform from source to desired goal frame
        :type transform: TransformStamped
        """
        self.__tf = transform

    def _get_output(
        self,
        transformation: Optional[TransformStamped] = None,
        clear_last: bool = False,
        **_,
    ) -> Optional[LaserScanData]:
        """
        Gets the laserscan data by applying the transformation if given.
        :returns:   Topic content
        :rtype:     Optional[LaserScanData]
        """
        if not self.msg:
            return None
        if transformation or self.transformation:
            laser_scan_data = self._transform(
                self.msg, transformation or self.transformation
            )
        else:
            laser_scan_data = self._process(self.msg)

        # Clear last message to get only recent values of the scan
        if clear_last:
            self.msg = None

        return laser_scan_data

    def _process(self, msg: LaserScan) -> LaserScanData:
        """
        Takes LaserScan ROS message and converts it to LaserScanData
        :return: LaserScanData
        """
        _no_nan_ranges = np.nan_to_num(msg.ranges, nan=msg.range_max)
        ranges = _no_nan_ranges.clip(min=0.0, max=msg.range_max)

        return LaserScanData(
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=msg.range_max,
            ranges=ranges,
            intensities=np.array(msg.intensities),
        )

    def _transform(self, msg: LaserScan, transform: TransformStamped) -> LaserScanData:
        """
        Applies a transform to a given LaserScan message and converts it to LaserScanData

        :param msg: LaserScan message in source frame
        :type msg: LaserScan
        :param transform: LaserScan transform from current to goal frame
        :type transform: TransformStamped

        :return: LaserScanData in goal frame
        :rtype: LaserScanData
        """
        laserscan_transformed = LaserScanData()

        trans = transform.transform.translation
        quat = transform.transform.rotation

        _no_nan_ranges = np.nan_to_num(msg.ranges, nan=msg.range_max)
        ranges = _no_nan_ranges.clip(min=0.0, max=msg.range_max)

        # Get the transformed laser scan data
        laserscan_transformed = (
            GeometryUtils.get_laserscan_transformed_polar_coordinates(
                angle_min=msg.angle_min,
                angle_max=msg.angle_max,
                angle_increment=msg.angle_increment,
                laser_scan_ranges=ranges,
                max_scan_range=msg.range_max,
                translation=[trans.x, trans.y, trans.z],
                rotation=[quat.x, quat.y, quat.z, quat.w],
            )
        )

        laserscan_transformed.time_increment = msg.time_increment
        laserscan_transformed.scan_time = msg.scan_time

        return laserscan_transformed


class PointCloudCallback(GenericCallback):
    """ROS2 PointCloud Callback Handler to process and transform sensor_msgs/PointCloud2 data"""

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        transformation: Optional[TransformStamped] = None,
        max_range: Optional[float] = None,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param transformation:
        :type transformation: Optional[TransformStamped]
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self.__tf = transformation
        self.__max_range: Optional[float] = max_range

    @property
    def max_range(self) -> Optional[float]:
        """Getter of a maximum distance (meters) to limit the PointCloud2 range

        :return: Maximum range (m)
        :rtype: Optional[float]
        """
        return self.__max_range

    @max_range.setter
    def max_range(self, dist: float):
        """Setter of the maximum range to limit the PointCloud2 points

        :param dist: Maximum range (m)
        :type dist: float
        """
        self.__max_range = dist

    @property
    def transformation(self) -> Optional[TransformStamped]:
        """Getter of the transformation

        :return: Frame transformation
        :rtype: Optional[TransformStamped]
        """
        return self.__tf

    @transformation.setter
    def transformation(self, transform: TransformStamped):
        """
        Sets a new transformation value

        :param transform: Detected transform from source to desired goal frame
        :type transform: TransformStamped
        """
        self.__tf = transform

    def _transform(
        self,
        msg: PointCloud2,
        transform: TransformStamped,
        discard_negative_z: bool = False,
    ) -> PointCloudData:
        """Transform PointCloud data using ROS2 transform

        :param msg: Point Cloud data
        :type msg: PointCloud2
        :param transform: Transform to desired frame
        :type transform: TransformStamped

        :return: Transformed Point Cloud data
        :rtype: PointCloud2
        """
        xyz_points = PointCloudData()
        for point in read_pc_points_with_tf(msg, transform):
            if (point[2] > 0 if discard_negative_z else True) and (
                point[0] < self.max_range if self.max_range else True
            ):
                xyz_points.add(point[0], point[1], point[2])
        return xyz_points

    def _process(
        self, msg: PointCloud2, discard_negative_z: bool = False
    ) -> PointCloudData:
        xyz_points = PointCloudData()
        for point in read_pc_points(msg):
            if (point[2] > 0 if discard_negative_z else True) and (
                point[0] < self.max_range if self.max_range else True
            ):
                xyz_points.add(point[0], point[1], point[2])
        return xyz_points

    def _get_output(
        self,
        transformation: Optional[TransformStamped] = None,
        clear_last: bool = False,
        discard_underground: bool = True,
        **_,
    ) -> Optional[PointCloudData]:
        """
        Gets the laserscan data by applying the transformation if given.
        :returns:   Topic content
        :rtype:     Optional[LaserScanData]
        """
        if not self.msg:
            return None

        if transformation or self.transformation:
            xyz_points = self._transform(
                self.msg, transformation or self.transformation, discard_underground
            )
        else:
            xyz_points = self._process(self.msg, discard_underground)

        # Clear last message to get only recent values of the scan
        if clear_last:
            self.msg = None

        return xyz_points
