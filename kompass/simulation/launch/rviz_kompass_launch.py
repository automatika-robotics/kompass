#!/usr/bin/env python

"""Launch RVIZ for point navigation"""

import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
)


def generate_launch_description():
    kompass_package_dir = get_package_share_directory("kompass")
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    rviz_config_dir = os.path.join(
        kompass_package_dir,
        "rviz",
        "webots.rviz",
    )

    # RVIZ
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-d", rviz_config_dir],
        output="screen",
    )

    return LaunchDescription([
        rviz_node,
    ])
