# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="turtlebot3_cartographer.lua"
    )
    turtlebot3_cartographer_prefix = os.path.dirname(os.path.abspath(__file__))

    cartographer_config_dir = LaunchConfiguration(
        "cartographer_config_dir",
        default=os.path.join(turtlebot3_cartographer_prefix, "../resources"),
    )
    rviz_config_dir = os.path.join(
        turtlebot3_cartographer_prefix,
        "../resources",
        "webots.rviz",
    )

    resolution = LaunchConfiguration("resolution", default="0.05")
    publish_period_sec = LaunchConfiguration("publish_period_sec", default="1.0")

    return LaunchDescription([
        DeclareLaunchArgument(
            "configuration_basename",
            default_value=configuration_basename,
            description="Name of lua file for cartographer",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory",
                cartographer_config_dir,
                "-configuration_basename",
                configuration_basename,
            ],
        ),
        DeclareLaunchArgument(
            "resolution",
            default_value=resolution,
            description="Resolution of a grid cell in the published occupancy grid",
        ),
        DeclareLaunchArgument(
            "publish_period_sec",
            default_value=publish_period_sec,
            description="OccupancyGrid publishing period",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                ThisLaunchFileDir(),
                "/occupancy_grid_launch.py",
            ]),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "resolution": resolution,
                "publish_period_sec": publish_period_sec,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", rviz_config_dir],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
