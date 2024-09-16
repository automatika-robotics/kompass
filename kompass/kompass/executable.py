#!/usr/bin/env python3
from ros_sugar import executable_main
from kompass.components import (
    Controller,
    ControllerConfig,
    DriveManager,
    DriveManagerConfig,
    MotionServer,
    MotionServerConfig,
    Planner,
    PlannerConfig,
)

_kompass_components = [Planner, Controller, DriveManager, MotionServer]

_kompass_configs = [
    PlannerConfig,
    ControllerConfig,
    DriveManagerConfig,
    MotionServerConfig,
]


def main(args=None):
    """
    Executable to run a component as a ros node.
    Used to start a node using Launcher

    # component is any KOMPASS component
    component = Planner()
    action = launch_ros.actions.LifecycleNode(
                                                package="kompass",
                                                name="node_name",
                                                executable="executable",
                                                arguments=component.launch_cmd_args,
                                            )

    :raises ValueError: If component cannot be started with provided arguments
    """
    executable_main(list_of_components=_kompass_components, list_of_configs=_kompass_configs)
