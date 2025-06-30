import argparse


def _kompass_import_error():
    """Prints kompass_core import error message"""
    print(
        "Welcome to Kompass! The fastest and most intuitive navigation stack known to man! Kompass uses the `kompass-core` python package, which implements highly parallelized navigation algorithms. Please install kompass-core using one of the following methods:\n - Install kompass-core with GPU support (Recommended):\n  `curl https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash`\n - Install kompass-core from pypi without GPU support:\n  `sudo apt-get install -y libompl-dev libfcl-dev libpcl-dev && pip install kompass-core`"
    )


def list_control_algorithms(*_):
    """Prints a list of available algorithms in the Controller component"""
    try:
        from kompass_core.control import ControllersID

        print("Available control algorithms:")
        for item in ControllersID:
            print(f"- {item.value}")
    except ImportError:
        _kompass_import_error()


def control_params(args):
    """Prints a given control algorithm config parameters

    :param args: Algorithm name in args.algorithm
    """
    try:
        from kompass_core.control import ControllersID, ControlConfigClasses

        config_cls = ControlConfigClasses[ControllersID(args.algorithm)]()
        print(f"Parameters for '{args.algorithm}' Controller in Kompass:")
        print("------------------------")
        print("Name: Default Value")
        print(f"{config_cls}")
    except ImportError:
        _kompass_import_error()
    except Exception:
        print(
            f"Unknown control algorithm: {args.algorithm}. To get available algorithms run: 'kompass_cli control list'"
        )


def list_planning_algorithms(*_):
    """Prints a list of available algorithms in the Planner component"""
    try:
        import omplpy as ompl
        from kompass_core.third_party.ompl.config import initializePlanners

        initializePlanners()
        planners = ompl.geometric.planners.getPlanners()
        print("Available planning algorithms from OMPL Geometric Planners:")
        for planner_name in planners.keys():
            reduced_name = planner_name.split(".")[-1]
            print(f"- {reduced_name}")
    except ImportError:
        _kompass_import_error()


def planner_params(args):
    """Prints a given planning algorithm config parameters

    :param args: Algorithm name in args.algorithm
    """
    try:
        import omplpy as ompl
        from kompass_core.third_party.ompl.config import initializePlanners

        initializePlanners()
        planners = ompl.geometric.planners.getPlanners()
        for planner_name, planner_params in planners.items():
            reduced_name = planner_name.split(".")[-1]
            if (
                reduced_name.lower() == args.algorithm.lower()
                or planner_name.lower() == args.algorithm.lower()
            ):
                print(f"'{planner_name}' Parameters:")
                print("------------------------")
                print("Name: Default Value")
                from kompass_core.third_party.ompl.config import create_config_class

                planner_config_class = create_config_class(reduced_name, planner_params)
                print(f"{planner_config_class()}")
                return
        print(
            f"Unknown planning algorithm: {args.algorithm}. To get available algorithms run: 'kompass_cli planning list'"
        )
    except ImportError:
        _kompass_import_error()


def gpu_support(*_):
    """Prints a list of available accelerators"""
    try:
        from kompass_cpp import get_available_accelerators
    except ImportError:
        _kompass_import_error()
        return

    if available_accelerators := get_available_accelerators():
        print("Available GPU accelerators:")
        for acc in available_accelerators.split("\n"):
            print(f"- {acc}")
    else:
        print("No GPU accelerators are available on this machine")
        return


def show_info(*_):
    """Shows CLI info"""
    print("""
    Kompass CLI - A command-line interface for inspection KOMPASS algorithms.

    Commands:
    • controller list
        List all available control algorithms.

    • controller params <algorithm>
        Display default parameters for the specified control algorithm.

    • planner list
        List all available planning algorithms.

    • planner params <algorithm>
        Display default parameters for the specified planning algorithm.

    • accelerators_support
        Get the SYCL-compatible accelerator devices available on the system.

    Usage Examples:
    ros2 run kompass kompass_cli controller list
    ros2 run kompass kompass_cli controller params <algorithm_name>
    ros2 run kompass kompass_cli planning list
    ros2 run kompass kompass_cli planning params <algorithm_name>
    ros2 run kompass kompass_cli accelerators_support
    ros2 run kompass kompass_cli info

    Kompass CLI is designed for Kompass users and developers
    to quickly query algorithm capabilities and configurations.

    Kompass Official Documentation: https://automatika-robotics.github.io/kompass/
    Kompass Source Code: https://github.com/automatika-robotics/kompass
    Author: Automatika Robotics
    """)


def main():
    """KOMPASS CLI MAIN"""
    parser = argparse.ArgumentParser(prog="kompass_cli", description="Kompass CLI")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Control group
    control_parser = subparsers.add_parser(
        "controller",
        help="Show control algorithms and parameters provided by the Controller component",
    )
    control_subparsers = control_parser.add_subparsers(
        dest="control_cmd", required=True
    )

    control_list = control_subparsers.add_parser(
        "list", help="List available control algorithms"
    )
    control_list.set_defaults(func=list_control_algorithms)

    control_params_cmd = control_subparsers.add_parser(
        "params", help="Get a control algorithm parameters and its default values"
    )
    control_params_cmd.add_argument("algorithm", help="Algorithm name")
    control_params_cmd.set_defaults(func=control_params)

    # Planning group
    planning_parser = subparsers.add_parser(
        "planner",
        help="Show planning algorithms and parameters provided by the Planner component",
    )
    planning_subparsers = planning_parser.add_subparsers(
        dest="planning_cmd", required=True
    )

    planning_list = planning_subparsers.add_parser(
        "list", help="List planning algorithms"
    )
    planning_list.set_defaults(func=list_planning_algorithms)

    planning_params_cmd = planning_subparsers.add_parser(
        "params", help="Get a planning algorithm parameters and its default values"
    )
    planning_params_cmd.add_argument("algorithm", help="Algorithm name")
    planning_params_cmd.set_defaults(func=planner_params)

    # GPU Support
    gpu_parser = subparsers.add_parser(
        "accelerators_support",
        help="Get the SYCL-compatible accelerator devices available on the system",
    )
    gpu_parser.set_defaults(func=gpu_support)

    # Info
    info_parser = subparsers.add_parser("info", help="Show CLI info and usage examples")
    info_parser.set_defaults(func=show_info)

    # Parse and dispatch
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
