import numpy as np
import os

from kompass_core.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)
from kompass_core.control import LocalPlannersID

from ros_sugar_interfaces.msg import ComponentStatus

from kompass import event
from kompass.actions import Action

from kompass.components import Controller, DriveManager, Planner, PlannerConfig
from kompass.config import RobotConfig
from kompass.launcher import Launcher
from kompass.topic import Topic

from ament_index_python.packages import (
    get_package_share_directory,
)


def kompass_bringup():
    package_dir = get_package_share_directory(package_name="kompass")
    config_file = os.path.join(package_dir, "params", "turtlebot3.yaml")

    # DEFINE COMPONENTS
    my_robot = RobotConfig(
        model_type=RobotType.DIFFERENTIAL_DRIVE,
        geometry_type=RobotGeometry.Type.CYLINDER,
        geometry_params=np.array([0.1, 0.3]),
        ctrl_vx_limits=LinearCtrlLimits(max_vel=0.2, max_acc=1.5, max_decel=2.5),
        ctrl_omega_limits=AngularCtrlLimits(
            max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
        ),
    )

    config = PlannerConfig(robot=my_robot, loop_rate=1.0)
    planner = Planner(node_name="planner", config=config, config_file=config_file)

    controller = Controller(node_name="controller")
    driver = DriveManager(node_name="drive_manager")

    # Configure Controller options
    controller.algorithm = LocalPlannersID.STANLEY
    controller.outputs(command=Topic(name="/cmd_vel", msg_type="Twist"))
    controller.direct_sensor = True

    planner.run_type = "Timed"
    goal_topic = Topic(name="/clicked_point", msg_type="PointStamped")
    planner.inputs(goal_point=goal_topic)

    # DEFINE EVENTS
    event_control_fail = event.OnDifferent(
        "control_fail",
        Topic(name="controller_status", msg_type="ComponentStatus"),
        ComponentStatus.STATUS_HEALTHY,
        ("status"),
    )
    event_emergency_stop = event.OnEqual(
        "emergency_stop",
        Topic(name="emergency_stop", msg_type="Bool"),
        True,
        ("data"),
    )
    unblock_action = Action(method=driver.move_to_unblock)

    driver.events_actions = {
        event_control_fail: unblock_action,
        event_emergency_stop: unblock_action,
    }

    driver.on_fail(action=Action(driver.restart))

    # Setup the launcher
    launcher = Launcher(
        components=[planner, controller, driver],
        config_file=config_file,
        activate_all_components_on_start=True,
        multi_processing=True,
    )

    # Get odom from localizer filtered odom
    odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
    launcher.inputs(location=odom_topic)

    launcher.robot = my_robot

    launcher.bringup(ros_log_level="info", introspect=True)
