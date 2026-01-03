import numpy as np
import os
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
from kompass.control import ControllersID

from automatika_ros_sugar.msg import ComponentStatus
from kompass_interfaces.action import PlanPath
from kompass_interfaces.msg import PathTrackingError
from geometry_msgs.msg import Pose, PointStamped

from kompass import event
from kompass.actions import Action

from kompass.components import (
    Controller,
    DriveManager,
    Planner,
    PlannerConfig,
    LocalMapper,
    MapServer,
    MapServerConfig,
)
from kompass.actions import ComponentActions, LogInfo
from kompass.launcher import Launcher
from kompass.ros import Topic

from ament_index_python.packages import (
    get_package_share_directory,
)


def kompass_bringup():
    package_dir = get_package_share_directory(package_name="kompass")
    kompass_sim_dir = get_package_share_directory(package_name="kompass_sim")
    config_file = os.path.join(package_dir, "params", "turtlebot3.toml")

    # Setup your robot configuration
    my_robot = RobotConfig(
        model_type=RobotType.DIFFERENTIAL_DRIVE,
        geometry_type=RobotGeometry.Type.CYLINDER,
        geometry_params=np.array([0.1, 0.3]),
        ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
        ctrl_omega_limits=AngularCtrlLimits(
            max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
        ),
    )

    config = PlannerConfig(robot=my_robot, loop_rate=1.0)
    planner = Planner(component_name="planner", config=config)

    controller = Controller(component_name="controller")
    driver = DriveManager(component_name="drive_manager")
    mapper = LocalMapper(component_name="mapper")

    # Map server
    map_file = os.path.join(kompass_sim_dir, "maps", "turtlebot3_webots.yaml")
    config = MapServerConfig(
        loop_rate=1.0,
        map_file_path=map_file,  # Path to a 2D map yaml file or a point cloud file
        grid_resolution=0.5,
        pc_publish_row=False,
    )
    map_server = MapServer(component_name="global_map_server", config=config)

    # Publish Twist or TwistStamped from the DriveManager based on the distribution
    if "ROS_DISTRO" in os.environ and (
        os.environ["ROS_DISTRO"] in ["rolling", "jazzy", "kilted"]
    ):
        cmd_msg_type: str = "TwistStamped"
    else:
        cmd_msg_type = "Twist"

    driver.outputs(command=Topic(name="/cmd_vel", msg_type=cmd_msg_type))

    # Configure Controller options
    controller.algorithm = ControllersID.DWA
    controller.direct_sensor = False
    controller.run_type = "Timed"

    # Run the Planner as an action server
    planner.run_type = "ActionServer"

    # Add on Any fail policy to the DriveManager
    driver.on_fail(action=Action(driver.restart))

    # DEFINE EVENTS - Uncomment this code to add reactive behavior in case of emergency stopping
    event_emergency_stop = event.OnEqual(
        "emergency_stop",
        Topic(name="emergency_stop", msg_type="Bool"),
        True,
        "data",
    )
    event_controller_fail = event.OnEqual(
        "controller_fail",
        Topic(name="controller_status", msg_type="ComponentStatus"),
        ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
        "status",
    )
    unblock_action = Action(method=driver.move_to_unblock)

    # On any clicked point
    event_clicked_point = event.OnGreater(
        "agents_goal",
        Topic(name="/clicked_point", msg_type="PointStamped"),
        0,
        ["header", "stamp", "sec"],
        or_equal=True,
    )

    # Define an Action to send a goal to the planner ActionServer
    send_goal: Action = ComponentActions.send_action_goal(
        action_name="/planner/plan_path",
        action_type=PlanPath,
        action_request_msg=PlanPath.Goal(),
    )

    # Define a method to parse a message of type PointStamped to the planner PlanPath Goal
    def goal_point_parser(*, msg: PointStamped, **_):
        action_request = PlanPath.Goal()
        goal = Pose()
        goal.position.x = msg.point.x
        goal.position.y = msg.point.y
        action_request.goal = goal
        end_tolerance = PathTrackingError()
        end_tolerance.orientation_error = 0.2
        end_tolerance.lateral_distance_error = 0.05
        action_request.end_tolerance = end_tolerance
        return action_request

    # Adds the parser method as an Event parser of the send_goal action
    send_goal.event_parser(goal_point_parser, output_mapping="action_request_msg")

    # Define Events/Actions dictionary
    events_actions = {
        event_clicked_point: [LogInfo(msg="Got new goal point"), send_goal],
        event_controller_fail: unblock_action,
        # Add the event action - Uncomment this code to add reactive behavior in case of emergency stopping
        event_emergency_stop: [
            ComponentActions.restart(component=planner),
            unblock_action,
        ],
    }

    # Setup the launcher
    launcher = Launcher(config_file=config_file)

    # Add Kompass components
    launcher.kompass(
        components=[driver, mapper, map_server, controller, planner],
        events_actions=events_actions,
        multiprocessing=True,
    )

    # Get odom from localizer filtered odom for all components
    odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
    launcher.inputs(location=odom_topic)

    # Set the robot config for all components
    launcher.robot = my_robot

    launcher.bringup()
