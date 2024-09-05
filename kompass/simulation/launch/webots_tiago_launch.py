#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Tiago Robot Simulation + Map Server + Robot Localization"""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
import launch
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch.actions import (
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():
    kompass_package_dir = get_package_share_directory("kompass")
    package_dir = get_package_share_directory("webots_ros2_tiago")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)
    run_rviz = LaunchConfiguration("run_rviz", default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "default.wbt"]),
        mode=mode,
        ros2_supervisor=True,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": '<robot name=""><link name=""/></robot>'}],
    )

    footprint_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
    )

    # ROS control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["diffdrive_controller"] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )
    ros_control_spawners = [
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    robot_description_path = os.path.join(package_dir, "resource", "tiago_webots.urdf")
    ros2_control_params = os.path.join(package_dir, "resource", "ros2_control.yml")
    use_twist_stamped = "ROS_DISTRO" in os.environ and (
        os.environ["ROS_DISTRO"] in ["rolling", "jazzy"]
    )
    if use_twist_stamped:
        mappings = [
            ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ]
    else:
        mappings = [
            ("/diffdrive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diffdrive_controller/odom", "/odom"),
        ]
    tiago_driver = WebotsController(
        robot_name="Tiago_Lite",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": True,
            },
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(
                kompass_package_dir,
                "params",
                "tiago_localization.yaml",
            )
        ],
        remappings=[("/set_pose", "/initialpose")],
    )

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=tiago_driver,
        nodes_to_start=ros_control_spawners + [localization_node],
    )

    # Map server
    map_server_config_path = os.path.join(
        kompass_package_dir, "maps", "tiago_office.yaml"
    )

    map_server_node = LifecycleNode(
        name="map_server",
        namespace="",
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_server_config_path}],
    )

    emit_event_to_configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state="inactive",
            entities=[
                LogInfo(
                    msg="node 'Map Server' reached the 'inactive' state, 'activating'."
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

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
        condition=IfCondition(run_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="turtlebot3_burger_example.wbt",
            description="Choose one of the world files from `/webots_ros2_turtlebot/world` directory",
        ),
        DeclareLaunchArgument(
            "mode", default_value="realtime", description="Webots startup mode"
        ),
        webots,
        webots._supervisor,
        robot_state_publisher,
        footprint_publisher,
        tiago_driver,
        waiting_nodes,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        map_server_node,
        emit_event_to_configure_map_server,
        activate_map_server,
        rviz_node,
    ])
