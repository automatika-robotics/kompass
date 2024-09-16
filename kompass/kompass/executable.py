#!/usr/bin/env python3
from ros_sugar import executable_main
from kompass import components_list, configs_list


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

    :param args: _description_, defaults to None
    :type args: _type_, optional

    :raises ValueError: If component cannot be started with provided arguments
    """
    executable_main(components_list, configs_list)
