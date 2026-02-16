import numpy as np
import os
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
    RobotFrames,
)
from kompass.control import ControllersID, MapConfig

from automatika_ros_sugar.msg import ComponentStatus

from kompass.components import (
    Controller,
    DriveManager,
    DriveManagerConfig,
    Planner,
    PlannerConfig,
    LocalMapper,
    LocalMapperConfig,
    MapServer,
    MapServerConfig,
    TopicsKeys,
)
from kompass.ros import Topic, Launcher, Event, Action, actions

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

    # Configure the Global Planner
    planner_config = PlannerConfig(loop_rate=1.0)
    planner = Planner(component_name="planner", config=planner_config)
    planner.run_type = "Timed"

    # Configure the motion controller
    controller = Controller(component_name="controller")
    controller.algorithm = ControllersID.PURE_PURSUIT
    controller.direct_sensor = (
        False  # Get local perception from a "map" instead (from the local mapper)
    )

    # Configure the Drive Manager (Direct commands sending to robot)
    driver_config = DriveManagerConfig(
        critical_zone_distance=0.05,
        critical_zone_angle=90.0,
        slowdown_zone_distance=0.3,
    )
    driver = DriveManager(component_name="drive_manager", config=driver_config)
    # Publish Twist or TwistStamped from the DriveManager based on the distribution
    if "ROS_DISTRO" in os.environ and (
        os.environ["ROS_DISTRO"] in ["rolling", "jazzy", "kilted"]
    ):
        cmd_msg_type: str = "TwistStamped"
    else:
        cmd_msg_type = "Twist"

    driver.outputs(robot_command=Topic(name="/cmd_vel", msg_type=cmd_msg_type))

    # Configure a Local Mapper
    local_mapper_config = LocalMapperConfig(
        map_params=MapConfig(width=3.0, height=3.0, resolution=0.05)
    )
    local_mapper = LocalMapper(component_name="mapper", config=local_mapper_config)

    # Configure the global Map Server
    map_file = os.path.join(kompass_sim_dir, "maps", "turtlebot3_webots.yaml")
    map_server_config = MapServerConfig(
        loop_rate=1.0,
        map_file_path=map_file,  # Path to a 2D map yaml file or a point cloud file
        grid_resolution=0.5,
        pc_publish_row=False,
    )
    map_server = MapServer(component_name="global_map_server", config=map_server_config)

    # Get the emergency stop topic directly from the DriveManager outputs
    emergency_stop_topic = driver.get_out_topic(TopicsKeys.EMERGENCY)

    # The robot is considered blocked if:
    # 1. the emergency stop is active
    # 2. OR the controller algorithm failed to find a solution
    event_robot_blocked = Event(
        emergency_stop_topic.msg.data.is_true()
        | (
            controller.status_topic.msg.status
            == ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL
        ),
    )
    unblock_action = Action(method=driver.move_to_unblock)

    # On any clicked point
    clicked_point_topic = Topic(name="/clicked_point", msg_type="PointStamped")
    event_clicked_point = Event(
        clicked_point_topic,
    )

    # Define an Action to send a goal to the planner ActionServer based on the coming clicked point topic data
    send_goal: Action = Action(
        method=planner.trigger_main_action_server,
        args=(
            clicked_point_topic.msg.point.x,
            clicked_point_topic.msg.point.y,
            0.0,
            0.05,
            0.2,
        ),
    )

    # Define Events/Actions dictionary
    events_actions = {
        event_clicked_point: [actions.log(msg="Got new goal point"), send_goal],
        # Add the event action - Uncomment this code to add reactive behavior in case of emergency stopping
        event_robot_blocked: [
            unblock_action,
        ],
    }

    # Setup the launcher
    launcher = Launcher(config_file=config_file)

    # Add Kompass components
    launcher.kompass(
        components=[map_server, controller, planner, driver, local_mapper],
        events_actions=events_actions,
        multiprocessing=True,
    )

    # Get odom from localizer filtered odom for all components
    odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
    launcher.inputs(location=odom_topic)

    # Set the robot config for all components
    launcher.robot = my_robot
    launcher.frames = RobotFrames(world="map", odom="map", scan="LDS-01")

    # Enable the UI
    # Inputs: Planner action server
    # Outputs: Static Map, Global Plan, Robot Odometry
    launcher.enable_ui(
        inputs=[planner.ui_main_action_input],
        outputs=[
            map_server.get_out_topic(TopicsKeys.GLOBAL_MAP),
            odom_topic,
            planner.get_out_topic(TopicsKeys.GLOBAL_PLAN),
        ],
    )

    # Run the Recipe
    launcher.bringup()
