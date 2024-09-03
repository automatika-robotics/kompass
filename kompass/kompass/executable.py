#!/usr/bin/env python3
import argparse
import logging
from typing import Optional

import rclpy
import setproctitle
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from kompass import components_dict, config_dict


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    """Parse arguments."""
    parser = argparse.ArgumentParser(description="Run KOMPASS ROS2 component")
    parser.add_argument(
        "--config_type", type=str, help="Component configuration class name"
    )
    parser.add_argument("--component_type", type=str, help="Component class name")
    parser.add_argument(
        "--node_name",
        type=str,
        help="Component ROS2 node name",
    )
    parser.add_argument("--config", type=str, help="Component configuration object")
    parser.add_argument(
        "--inputs",
        type=str,
        help="Component input topics",
    )
    parser.add_argument(
        "--outputs",
        type=str,
        help="Component output topics",
    )

    parser.add_argument(
        "--config_file", type=str, help="Path to configuration YAML file"
    )
    parser.add_argument(
        "--events", type=str, help="Events to be monitored by the component"
    )
    parser.add_argument(
        "--actions", type=str, help="Actions associated with the component Events"
    )
    return parser.parse_known_args()


def _parse_component_config(args: argparse.Namespace) -> Optional[object]:
    """Parse the component config object

    :param args: Command line arguments
    :type args: argparse.Namespace

    :return: Component config object
    :rtype: object
    """
    config_type = args.config_type or None

    if not config_type or config_type not in config_dict.keys():
        logging.warn(
            f"Unknown config_type '{config_type}'. Proceeding with default config for component"
        )
        config = None
    else:
        # Get config type and update from json arg
        config = config_dict[config_type]()

        config_json = args.config

        if config_json and config:
            config.from_json(config_json)

    return config


def _parse_ros_args(args_names: list[str]) -> list[str]:
    """Parse ROS arguments from command line arguments

    :param args_names: List of all parsed arguments
    :type args_names: list[str]

    :return: List ROS parsed arguments
    :rtype: list[str]
    """
    # Look for --ros-args in ros_args
    ros_args_start = None
    if "--ros-args" in args_names:
        ros_args_start = args_names.index("--ros-args")

    if ros_args_start is not None:
        ros_specific_args = args_names[ros_args_start:]
    else:
        ros_specific_args = []
    return ros_specific_args


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
    args, args_names = parse_args()

    # Initialize rclpy with the ros-specific arguments
    rclpy.init(args=_parse_ros_args(args_names))

    component_type = args.component_type or None

    if not component_type or component_type not in components_dict.keys():
        raise ValueError(
            f"Cannot launch unknown component type '{component_type}'. Known types are: '{list(components_dict.keys())}'"
        )

    # Get name
    node_name = args.node_name or None

    if not node_name:
        raise ValueError("Cannot launch component without specifying a node name")

    # SET PROCESS NAME
    setproctitle.setproctitle(node_name)

    config = _parse_component_config(args)

    # Get Yaml config file if provided
    config_file = args.config_file or None

    # Init the component
    component = components_dict[component_type](
        config=config, node_name=node_name, config_file=config_file
    )

    # Init the node with rclpy
    component.rclpy_init_node()

    # Set inputs/outputs
    inputs_json = args.inputs or None
    outputs_json = args.outputs or None

    try:
        if inputs_json:
            component.inputs_json = inputs_json

        if outputs_json:
            component.outputs_json = outputs_json
    except (ValueError, TypeError) as e:
        logging.warn(
            f"Passed Invalid inputs and/or outputs -> continue with component default values. Error: '{e}'"
        )

    # Set events/actions
    events_json = args.events or None
    actions_json = args.actions or None

    if events_json and actions_json:
        component.events_json = events_json
        component.actions_json = actions_json

    executor = MultiThreadedExecutor()

    executor.add_node(component)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.remove_node(component)
        try_shutdown()
