"""Callback classes used to process input topics data for supported types in Kompass"""

from typing import Optional, Union, Any
import numpy as np
from collections import deque

from ros_sugar.io import GenericCallback, OccupancyGridCallback
from ros_sugar.io import OdomCallback as BaseOdomCallback
from ros_sugar.io import PointCallback as BasePointCallback
from ros_sugar.io import PoseCallback as BasePoseCallback
from ros_sugar.io.utils import read_compressed_image
from kompass_core.datatypes import (
    LaserScanData,
    PointCloudData,
    TrackingData,
    ImageMetaData,
)
from .utils import read_pc_points, read_pc_points_with_tf
from kompass_core.utils import geometry as GeometryUtils
from kompass_core.models import RobotState

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import TransformStamped
from geometry_msgs.msg import Point, Pose

__all__ = [
    "OdomCallback",
    "PointCallback",
    "PointStampedCallback",
    "PoseCallback",
    "PoseStampedCallback",
    "LaserScanCallback",
    "OccupancyGridCallback",
    "TrackingsCallback",
    "DetectionsCallback",
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


class DetectionsCallback(GenericCallback):
    """ROS2 Detections Callback Handler to process and transform automatika_agents_interfaces/Detections data"""

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        buffer_size: Optional[int] = 10,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param buffer_size:
        :type buffer_size: Optional[Int], default 10
        :rtype: None
        """
        super().__init__(input_topic, node_name)
        self._buffer_items: int = 0
        self._max_buffer_size = buffer_size
        self._feature_items: int = (
            6  # (center_x, center_y, size_x, size_y, vel_x, vel_y)
        )
        self._detections_buffer = np.ones((
            buffer_size,
            self._feature_items,
        ))  # num_detections x num_features
        self._img_metadata: Optional[ImageMetaData] = None
        self._label: Optional[str] = None
        self._id: Optional[int] = None

    def _process_raw_data(self) -> None:
        """Process new raw detections data and add it to buffer if available"""
        # Remove a buffer item
        self._detections_buffer = np.roll(self._detections_buffer, -1, axis=0)

        if not self._label or not self.msg or (self.msg and not self.msg.detections):
            # No detection -> reduce buffer items
            self._buffer_items = max(self._buffer_items - 1, 0)
            return

        raw_detections = self.msg.detections

        # Get requested item from trackings using id or label
        detection_index = None
        label_index = None
        for idx, detection in enumerate(raw_detections):
            if self._label in detection.labels:
                detection_index = idx
                label_index = detection.labels.index(self._label)
                break

        # If requested label/id not in detections -> return None
        if detection_index is None or label_index is None:
            # No detection -> reduce buffer items
            self._buffer_items = max(self._buffer_items - 1, 0)
            return

        det = raw_detections[detection_index]
        bbox_2d = det.boxes[label_index]

        # Update the source image data if available
        if det.image.data:
            if (
                not self._img_metadata
                or det.image.header.frame_id != self._img_metadata.frame_id
            ):
                # Update img_metadata if None or a new frame_id is provided (new camera)
                self._img_metadata = ImageMetaData(
                    frame_id=det.image.header.frame_id,
                    width=det.image.width,
                    height=det.image.height,
                    encoding=det.image.encoding,
                )
                # If a new image meta data is detected -> clear the buffer (the detection is coming from a new camera -> clear old camera data)
                self._detections_buffer = np.ones((
                    self._max_buffer_size,
                    self._feature_items,
                ))
                self._buffer_items = 0

        elif det.compressed_image.data:
            if (
                not self._img_metadata
                or det.compressed_image.header.frame_id != self._img_metadata.frame_id
            ):
                data: np.ndarray = read_compressed_image(det.compressed_image)
                self._img_metadata = ImageMetaData(
                    frame_id=det.compressed_image.header.frame_id,
                    width=data.shape[1],
                    height=data.shape[0],
                    encoding=det.compressed_image.format,
                )
                self._detections_buffer = np.ones((
                    self._max_buffer_size,
                    self._feature_items,
                ))
                self._buffer_items = 0

        # Add new detection to the buffer
        self._detections_buffer[-1:] = [
            (bbox_2d.bottom_right_x + bbox_2d.top_left_x) / 2,  # center_x
            (bbox_2d.top_left_y + bbox_2d.bottom_right_y) / 2,  # center_y
            abs(bbox_2d.bottom_right_x - bbox_2d.top_left_x),  # size_x
            abs(bbox_2d.bottom_right_y - bbox_2d.top_left_y),  # size_y
            0.0,  # no estimate for vel_x
            0.0,  # no estimate for vel_y
        ]
        self._buffer_items = min(self._buffer_items + 1, self._max_buffer_size)

    def set_buffer_size(self, value: int, clear_old: bool = False) -> None:
        """Resizes detections buffer (while maintaining old buffer items in the resized buffer)

        :param value: New buffer size
        :type value: int
        """
        new_buffer = np.ones((value, self._feature_items))
        if clear_old:
            self._buffer_items = 0
        else:
            if value >= self._max_buffer_size:
                new_buffer[-self._max_buffer_size :] = self._detections_buffer
            else:
                new_buffer = self._detections_buffer[-self._max_buffer_size :]
        self._max_buffer_size = value
        self._detections_buffer = new_buffer

    def set_target(self, label: str, idx: Optional[int] = None):
        """Sets tracked target label and id prior to calling get_output

        :param label: Tracked target label
        :type label: str
        :param idx: Tracked target index, defaults to None
        :type idx: Optional[int], optional
        """
        self._label = label
        self._id = idx

    def callback(self, msg) -> None:
        """
        Topic subscriber callback

        :param msg: Received ros msg
        :type msg: Any
        """
        super().callback(msg)
        self._process_raw_data()

    def _get_output(
        self,
        label: Optional[str] = None,
        idx: Optional[int] = 0,
        **_,
    ) -> Union[Any, TrackingData, None]:
        """
        Gets the trackings data
        :returns:   Topic content
        :rtype:     Union[ROSTrackings, np.ndarray, None]
        """
        self._label = label or self._label
        self._id = idx

        if not self._buffer_items:
            return None

        last_detections = self._detections_buffer[-self._buffer_items :]

        # Create weights array: [1, 2, ..., n]
        weights = np.arange(1, self._buffer_items + 1).reshape(-1, 1)

        # Multiply each row by its weight then divide by the sum
        average_det = np.sum(last_detections * weights, axis=0) / np.sum(weights)
        return TrackingData(
            label=self._label,
            id=self._id,
            center_xy=[
                average_det[0],
                average_det[1],
            ],
            size_xy=[
                average_det[2],
                average_det[3],
            ],
            velocity_xy=[average_det[4], average_det[5]],
            img_meta=self._img_metadata,
        )


class TrackingsCallback(DetectionsCallback):
    """ROS2 Trackings Callback Handler to process and transform automatika_agents_interfaces/Trackings data"""

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        buffer_size: Optional[int] = 10,
    ) -> None:
        """__init__.

        :param input_topic:
        :param node_name:
        :type node_name: Optional[str]
        :param buffer_size:
        :type buffer_size: Optional[Int], default 10
        :rtype: None
        """
        super().__init__(input_topic, node_name, buffer_size)

    def _process_raw_data(self) -> None:
        """Process new raw trackings data and add it to buffer if available"""
        # Remove a buffer item
        self._detections_buffer = np.roll(self._detections_buffer, -1, axis=0)

        if not self._label or not self.msg.trackings:
            # No trackings -> reduce buffer items
            self._buffer_items = max(self._buffer_items - 1, 0)
            return

        raw_trackings = self.msg.trackings

        # Get requested item from trackings using id or label
        tracking_index = None
        id_index = None
        for idx, tracking in enumerate(raw_trackings):
            if self._id and self._id in tracking.ids:
                tracking_index = idx
                id_index = tracking.ids.index(id)
                break
            elif self._label and self._label in tracking.labels:
                tracking_index = idx
                id_index = tracking.labels.index(self._label)
                break

        # If requested label/id not in detections -> return None
        if tracking_index is None or id_index is None:
            # No detection -> reduce buffer items
            self._buffer_items = max(self._buffer_items - 1, 0)
            return

        track = raw_trackings[tracking_index]
        bbox_2d = track.boxes[id_index]

        # Update the source image data if available
        if track.image.data:
            if (
                not self._img_metadata
                or track.image.header.frame_id != self._img_metadata.frame_id
            ):
                # Update img_metadata if None or a new frame_id is provided (new camera)
                self._img_metadata = ImageMetaData(
                    frame_id=track.image.header.frame_id,
                    width=track.image.width,
                    height=track.image.height,
                    encoding=track.image.encoding,
                )
                # If a new image meta data is detected -> clear the buffer (the detection is coming from a new camera -> clear old camera data)
                self._detections_buffer = np.ones((
                    self._max_buffer_size,
                    self._feature_items,
                ))
                self._buffer_items = 0

        elif track.compressed_image.data:
            if (
                not self._img_metadata
                or track.compressed_image.header.frame_id != self._img_metadata.frame_id
            ):
                data: np.ndarray = read_compressed_image(track.compressed_image)
                self._img_metadata = ImageMetaData(
                    frame_id=track.compressed_image.header.frame_id,
                    width=data.shape[1],
                    height=data.shape[0],
                    encoding=track.compressed_image.format,
                )
                self._detections_buffer = np.ones((
                    self._max_buffer_size,
                    self._feature_items,
                ))
                self._buffer_items = 0

        # Add new detection to the buffer
        self._detections_buffer[-1:] = [
            track.centroids[id_index].x,  # center_x
            track.centroids[id_index].y,  # center_y
            abs(bbox_2d.bottom_right_x - bbox_2d.top_left_x),  # size_x
            abs(bbox_2d.bottom_right_y - bbox_2d.top_left_y),  # size_y
            track.estimated_velocities[id_index].x,  # vel_x
            track.estimated_velocities[id_index].y,  # vel_y
        ]
        self._buffer_items = min(self._buffer_items + 1, self._max_buffer_size)


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

        return xyz_points
