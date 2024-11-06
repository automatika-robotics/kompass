import numpy as np
from typing import Optional, List
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kompass_core.models import RobotState

from kompass_interfaces.action import ControlPath
from kompass_interfaces.msg import TwistArray


def ros_odometry_to_2d_pose(odom_msg: Odometry) -> RobotState:
    """
    Converts a ROS Odometry msg to 2D pose information

    :param odom_msg: ROS Odomerty msg
    :type odom_msg: nav_msgs.msd.Odometry

    :return: 2D pose (x, y, heading, speed)
    :rtype: tuple[float]
    """
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    ori = 2 * np.arctan2(
        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w
    )
    vx = odom_msg.twist.twist.linear.x
    vy = odom_msg.twist.twist.linear.y
    speed = np.sqrt(vx**2 + vy**2)
    omega = odom_msg.twist.twist.angular.z
    return RobotState(x=x, y=y, yaw=ori, speed=speed, vx=vx, vy=vy, omega=omega)


def twist_array_to_ros_twist(cmd_list: TwistArray, idx: int) -> Twist:
    """
    Converts a TwistArray msg to Twist standard msg at a given index

    :param cmd_list: List of Twist commands
    :type cmd_list: TwistArray
    :param idx: Converted value index
    :type idx: int

    :return: Ros Twist msg
    :rtype: Twist
    """
    cmd_vel = Twist()
    cmd_vel.linear.x = cmd_list.linear_velocities.x[idx]
    cmd_vel.linear.y = cmd_list.linear_velocities.y[idx]
    cmd_vel.linear.z = cmd_list.linear_velocities.z[idx]

    cmd_vel.angular.x = cmd_list.angular_velocities.x[idx]
    cmd_vel.angular.y = cmd_list.angular_velocities.y[idx]
    cmd_vel.angular.z = cmd_list.angular_velocities.z[idx]

    return cmd_vel


def __restrict_list_length(list_values: List[float], list_len: int):
    return (
        list_values[:list_len]
        if len(list_values) >= list_len
        else list_values.extend((list_len - len(list_values)) * [0.0])
    )


def init_twist_array_msg(
    number_of_cmds: int,
    linear_x: Optional[List[float]] = None,
    linear_y: Optional[List[float]] = None,
    angular: Optional[List[float]] = None,
) -> TwistArray:
    """Initializes the TwistArray ROS msg with a set of zero commands of a given length or with given values

    :param number_of_cmds: _description_
    :type number_of_cmds: int
    :param linear_x: linear_velocities.x, defaults to None
    :type linear_x: Optional[List[float]], optional
    :param linear_y: linear_velocities.y, defaults to None
    :type linear_y: Optional[List[float]], optional
    :param angular: angular_velocities.z, defaults to None
    :type angular: Optional[List[float]], optional

    :return: ROS message
    :rtype: TwistArray
    """
    if linear_x:
        linear_x = __restrict_list_length(linear_x, number_of_cmds)
    if linear_y:
        linear_y = __restrict_list_length(linear_y, number_of_cmds)
    if angular:
        angular = __restrict_list_length(angular, number_of_cmds)

    cmd_list = TwistArray()
    init_list = number_of_cmds * [0.0]
    cmd_list.linear_velocities.x = linear_x if linear_x else init_list
    cmd_list.linear_velocities.y = linear_y if linear_y else init_list
    cmd_list.linear_velocities.z = init_list
    cmd_list.angular_velocities.x = init_list
    cmd_list.angular_velocities.y = init_list
    cmd_list.angular_velocities.z = angular if angular else init_list
    return cmd_list
