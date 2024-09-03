import numpy as np
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


def send_control_feedback(
    feedback_msg: ControlPath.Feedback,
    vel_lin: float,
    vel_ang: float,
    N_horizon: int,
    t_compute: float,
    lat_dist_error: float,
    ori_error: float,
) -> ControlPath.Feedback:
    """
    Updates the controller feedback msg logs info

    :param feedback_msg: Control feedback msg
    :type feedback_msg: ControlPath.Feedback
    :param vel_lin: Linear velocity control (m/s)
    :type vel_lin: float
    :param vel_ang: Angular velocity control (rad/s)
    :type vel_ang: float
    :param N_horizon: Number of prediction time steps
    :type N_horizon: int
    :param t_compute: Control computation time (s)
    :type t_compute: float
    :param lat_dist_error: Lateral distance between the robot and the global path (m)
    :type lat_dist_error: float
    :param ori_error: Orientation difference between the robot and the global path (rad)
    :type ori_error: float
    :return: Updated control feedback msg
    :rtype: ControlPath.Feedback
    """
    feedback_msg.control_feedback.linear_velocity_control = vel_lin
    feedback_msg.control_feedback.angular_velocity_control = vel_ang
    feedback_msg.control_feedback.global_path_deviation.orientation_error = (
        lat_dist_error
    )
    feedback_msg.control_feedback.global_path_deviation.lateral_distance_error = (
        ori_error
    )
    feedback_msg.control_feedback.prediction_horizon = N_horizon
    feedback_msg.control_feedback.compute_time = t_compute

    return feedback_msg


def init_twist_array_msg(
    number_of_cmds: int,
    init_linear_x_value: float = 0.0,
    init_linear_y_value: float = 0.0,
    init_angular_value: float = 0.0,
) -> TwistArray:
    """
    Initilizes the TwistArray ROS msg with a set of zero commands of a given length or with given values

    :param number_of_cmds: Number of Twist commands (list length)
    :type number_of_cmds: int
    :param init_linear_x_value: Initial value for linear velocities (forward), defaults to 0.0
    :type init_linear_x_value: float, optional
    :param init_linear_y_value: Initial value for linear velocities (lateral), defaults to 0.0
    :type init_linear_y_value: float, optional
    :param init_angular_value: Initial value for angular velocities, defaults to 0.0
    :type init_angular_value: float, optional

    :return: Resulting ros message
    :rtype: TwistArray
    """
    cmd_list = TwistArray()
    init_list = number_of_cmds * [0.0]
    cmd_list.linear_velocities.x = number_of_cmds * [init_linear_x_value]
    cmd_list.linear_velocities.y = number_of_cmds * [init_linear_y_value]
    cmd_list.linear_velocities.z = init_list
    cmd_list.angular_velocities.x = init_list
    cmd_list.angular_velocities.y = init_list
    cmd_list.angular_velocities.z = number_of_cmds * [init_angular_value]
    return cmd_list
