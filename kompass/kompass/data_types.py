"""Supported data types for inputs/outputs"""

import json
from typing import Optional, Union

import numpy as np
from auto_ros.supported_types import Bool, ComponentStatus, Float32, Float64
from auto_ros.supported_types import LaserScan as BaseLaserScan
from auto_ros.supported_types import OccupancyGrid
from auto_ros.supported_types import Odometry as BaseOdometry
from auto_ros.supported_types import Path as BasePath
from auto_ros.supported_types import Point as BasePoint
from auto_ros.supported_types import PointStamped as BasePointStamped
from auto_ros.supported_types import Pose as BasePose
from auto_ros.supported_types import PoseStamped as BasePoseStamped
from auto_ros.supported_types import SupportedType, Twist

# ROS MESSAGES
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from kompass_core.datatypes.laserscan import LaserScanData
from nav_msgs.msg import Odometry as ROSOdometry
from nav_msgs.msg import Path as ROSPath
from kompass_core.models import RobotState
from sensor_msgs.msg import LaserScan as ROSLaserScan
from sensor_msgs.msg import PointCloud2 as ROSPointCloud2

from kompass_interfaces.msg import TwistArray as ROSTwistArray

from .callbacks import (
    GenericCallback,
    LaserScanCallback,
    OdomCallback,
    PointCallback,
    PointStampedCallback,
    PoseCallback,
    PoseStampedCallback,
    PointCloudCallback,
)

__all__ = [
    "Pose",
    "PoseStamped",
    "Point",
    "PointStamped",
    "OccupancyGrid",
    "Path",
    "ComponentStatus",
    "Twist",
    "SupportedType",
    "TwistArray",
    "Bool",
    "Float32",
    "Float64",
]


class LaserScan(BaseLaserScan):
    """Class to support ROS2 sensor_msgs/msg/LaserScan message"""

    callback = LaserScanCallback

    @classmethod
    def convert(
        cls,
        output: LaserScanData,
        frame_id: Optional[str] = "map",
        time_sec: Optional[int] = 0,
        time_nanosec: Optional[int] = 0,
        **_,
    ) -> ROSLaserScan:
        """
        Takes LaserScanData object and converts it to ROS LaserScan message
        :return: LaserScan
        """
        msg = ROSLaserScan()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = time_sec
        msg.header.stamp.nanosec = time_nanosec
        msg.angle_min = output.angle_min
        msg.angle_max = output.angle_max
        msg.angle_increment = output.angle_increment
        msg.time_increment = output.time_increment
        msg.scan_time = output.scan_time
        msg.range_min = output.range_min
        msg.range_max = output.range_max
        msg.ranges = output.ranges
        msg.intensities = output.intensities

        return msg


class PointCloud2(SupportedType):
    """Class to support ROS2 sensor_msgs/msg/PointCloud2 message"""

    _ros_type = ROSPointCloud2
    callback = PointCloudCallback


class PointStamped(BasePointStamped):
    """Class to support ROS2 geometry_msgs/msg/PointStamped message"""

    callback = PointStampedCallback


class Point(BasePoint):
    """Class to support ROS2 geometry_msgs/msg/Point message"""

    callback = PointCallback


class Pose(BasePose):
    """Class to support ROS2 geometry_msgs/msg/Pose message"""

    callback = PoseCallback


class PoseStamped(BasePoseStamped):
    """Class to support ROS2 geometry_msgs/msg/PoseStamped message"""

    callback = PoseStampedCallback


class Odometry(BaseOdometry):
    """Class to support ROS2 nav_msgs/msg/Odometry message"""

    callback = OdomCallback

    @classmethod
    def convert(
        cls,
        output: RobotState,
        frame_id: Optional[str] = "map",
        time_sec: Optional[int] = 0,
        time_nanosec: Optional[int] = 0,
        **_,
    ) -> ROSOdometry:
        """
        Takes RobotState object and converts it to ROS Odometry message
        :return: Odometry
        """
        msg = ROSOdometry()
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = time_sec
        msg.header.stamp.nanosec = time_nanosec
        msg.pose.pose.position.x = output.x
        msg.pose.pose.position.y = output.y
        msg.pose.pose.orientation.w = np.cos(output.yaw / 2)
        msg.pose.pose.orientation.z = np.sin(output.yaw / 2)
        msg.twist.twist.linear.x = output.vx
        msg.twist.twist.linear.y = output.vy
        msg.twist.twist.angular.z = output.omega
        return msg


class TwistArray(SupportedType):
    """Class to support ROS2 kompass_interfaces/msg/TwistArray message"""

    _ros_type = ROSTwistArray
    callback = GenericCallback

    @classmethod
    def convert(cls, output: ROSTwistArray, **_):
        """convert.

        :param output:
        :type output: ROSTwistArray
        :param kwargs:
        """
        return output


class Path(BasePath):
    """Class to support ROS2 nav_msgs/msg/Path message"""

    @classmethod
    def to_json(cls, path: ROSPath, json_file: str):
        """
        Saves a ROS nav_msg.msg.Path in a json file

        :param path: ROS path message
        :type path: nav_msg.msg.Path
        :param json_file: Path to the saved file
        :type json_file: str
        """
        path_dict = {
            "header": {
                "stamp": {
                    "sec": path.header.stamp.sec,
                    "nanosec": path.header.stamp.nanosec,
                },
                "frame_id": path.header.frame_id,
            },
            "poses": [],
        }

        for pose_stamped in path.poses:
            pose = pose_stamped.pose
            path_dict["poses"].append({
                "header": {
                    "stamp": {
                        "sec": pose_stamped.header.stamp.sec,
                        "nanosec": pose_stamped.header.stamp.nanosec,
                    },
                    "frame_id": pose_stamped.header.frame_id,
                },
                "pose": {
                    "position": {
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z,
                    },
                    "orientation": {
                        "x": pose.orientation.x,
                        "y": pose.orientation.y,
                        "z": pose.orientation.z,
                        "w": pose.orientation.w,
                    },
                },
            })

        with open(json_file, "w") as f:
            json.dump(path_dict, f, indent=4)

    @classmethod
    def from_json(cls, json_file: str) -> Union[ROSPath, None]:
        """
        Reads a given json file and parse a ROS nav_msgs.msg.Path if exists

        :param json_file: Path to the json file
        :type json_file: str
        :return: ROS path message
        :rtype: Union[PathMsg, None]
        """
        try:
            with open(json_file, "r") as f:
                path_dict = json.load(f)

            path = ROSPath()
            path.header.stamp.sec = path_dict["header"]["stamp"]["sec"]
            path.header.stamp.nanosec = path_dict["header"]["stamp"]["nanosec"]
            path.header.frame_id = path_dict["header"]["frame_id"]

            for pose_dict in path_dict["poses"]:
                pose_stamped = ROSPoseStamped()
                pose_stamped.header.stamp.sec = pose_dict["header"]["stamp"]["sec"]
                pose_stamped.header.stamp.nanosec = pose_dict["header"]["stamp"][
                    "nanosec"
                ]
                pose_stamped.header.frame_id = pose_dict["header"]["frame_id"]

                pose_stamped.pose.position.x = pose_dict["pose"]["position"]["x"]
                pose_stamped.pose.position.y = pose_dict["pose"]["position"]["y"]
                pose_stamped.pose.position.z = pose_dict["pose"]["position"]["z"]

                pose_stamped.pose.orientation.x = pose_dict["pose"]["orientation"]["x"]
                pose_stamped.pose.orientation.y = pose_dict["pose"]["orientation"]["y"]
                pose_stamped.pose.orientation.z = pose_dict["pose"]["orientation"]["z"]
                pose_stamped.pose.orientation.w = pose_dict["pose"]["orientation"]["w"]

                path.poses.append(pose_stamped)

            return path
        # File not found or format is not compatible
        except Exception:
            return None