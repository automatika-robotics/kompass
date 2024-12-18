from typing import Dict
from ros_sugar.supported_types import add_additional_datatypes
from .. import data_types
from ros_sugar.io import AllowedTopics, get_all_msg_types
from ros_sugar.config import QoSConfig
from rclpy import qos
from ..ros import Topic
from ..utils import StrEnum

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


class TopicsKeys(StrEnum):
    # INPUT
    GOAL_POINT = "goal_point"
    # Global
    GLOBAL_PLAN = "plan"
    GLOBAL_MAP = "map"
    # Sensory information
    ROBOT_LOCATION = "location"
    SPATIAL_SENSOR = "sensor_data"
    VISION_TRACKINGS = "vision_tracking"
    # Calculated
    LOCAL_PLAN = "local_plan"
    INTERMEDIATE_CMD = "command"
    INTERMEDIATE_CMD_LIST = "multi_command"
    LOCAL_MAP = "local_map"
    LOCAL_MAP_OCC = "local_map"
    INTERPOLATED_PATH = "interpolation"
    TRACKED_POINT = "tracked_point"
    # Result
    FINAL_COMMAND = "robot_command"
    EMERGENCY = "emergency_stop"
    REACHED_END = "reached_end"
    RUN_TESTS = "run_tests"


controller_allowed_inputs: Dict[str, AllowedTopics] = {
    TopicsKeys.GLOBAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["LaserScan", "PointCloud2"]),
    TopicsKeys.LOCAL_MAP: AllowedTopics(types=["OccupancyGrid"]),
    TopicsKeys.VISION_TRACKINGS: AllowedTopics(types=["Trackings", "Detections"]),
}

controller_allowed_outputs: Dict[str, AllowedTopics] = {
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.INTERMEDIATE_CMD_LIST: AllowedTopics(types=["TwistArray"]),
    TopicsKeys.INTERPOLATED_PATH: AllowedTopics(types=["Path"]),
    TopicsKeys.LOCAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.TRACKED_POINT: AllowedTopics(
        types=["Odometry", "PoseStamped", "Pose", "Detection"]
    ),
}

# Create default inputs - Used if no inputs config is provided to the controller
controller_default_inputs: Dict[str, Topic] = {
    TopicsKeys.GLOBAL_PLAN: Topic(name="/plan", msg_type="Path"),
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.LOCAL_MAP: Topic(
        name="/local_map/occupancy_layer", msg_type="OccupancyGrid"
    ),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
    TopicsKeys.VISION_TRACKINGS: Topic(name="/trackings", msg_type="Trackings"),
}

# Create default outputs - Used if no outputs config is provided to the controller
controller_default_outputs: Dict[str, Topic] = {
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/control", msg_type="Twist"),
    TopicsKeys.INTERMEDIATE_CMD_LIST: Topic(
        name="/control_list", msg_type="TwistArray"
    ),
    TopicsKeys.INTERPOLATED_PATH: Topic(name="/interpolated_path", msg_type="Path"),
    TopicsKeys.TRACKED_POINT: Topic(name="/tracked_point", msg_type="PoseStamped"),
    TopicsKeys.LOCAL_PLAN: Topic(name="/local_path", msg_type="Path"),
}

# DRIVE MANAGER
driver_allowed_inputs: Dict[str, AllowedTopics] = {
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.INTERMEDIATE_CMD_LIST: AllowedTopics(types=["TwistArray"]),
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(
        types=["LaserScan", "Float64", "Float32"],
        number_required=1,
        number_optional=10,
    ),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(
        types=["Odometry", "PoseStamped", "PointStamped"]
    ),
}

driver_allowed_outputs: Dict[str, AllowedTopics] = {
    TopicsKeys.FINAL_COMMAND: AllowedTopics(types=["Twist"]),
    TopicsKeys.EMERGENCY: AllowedTopics(types=["Bool"]),
}


driver_default_inputs: Dict[str, Topic] = {
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/control", msg_type="Twist"),
    TopicsKeys.INTERMEDIATE_CMD_LIST: Topic(
        name="/control_list", msg_type="TwistArray"
    ),
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

driver_default_outputs: Dict[str, Topic] = {
    TopicsKeys.FINAL_COMMAND: Topic(name="/cmd_vel", msg_type="Twist"),
    TopicsKeys.EMERGENCY: Topic(name="/emergency_stop", msg_type="Bool"),
}

# LOCAL MAPPER
mapper_allowed_inputs: Dict[str, AllowedTopics] = {
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["LaserScan"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
}

mapper_allowed_outputs: Dict[str, AllowedTopics] = {
    TopicsKeys.LOCAL_MAP_OCC: AllowedTopics(types=["OccupancyGrid"]),
}


# Create default inputs - Used if no inputs config is provided to the controller
mapper_default_inputs: Dict[str, Topic] = {
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

# Create default outputs - Used if no outputs config is provided to the controller
mapper_default_outputs: Dict[str, Topic] = {
    TopicsKeys.LOCAL_MAP_OCC: Topic(
        name="/local_map/occupancy_layer", msg_type="OccupancyGrid"
    ),
}

# MOTION SERVER
motion_server_allowed_inputs: Dict[str, AllowedTopics] = {
    TopicsKeys.RUN_TESTS: AllowedTopics(types=["Bool"]),
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
}

motion_server_allowed_outputs: Dict[str, AllowedTopics] = {
    TopicsKeys.FINAL_COMMAND: AllowedTopics(types=["Twist"]),
}

motion_server_default_outputs: Dict[str, Topic] = {
    TopicsKeys.FINAL_COMMAND: Topic(name="/control", msg_type="Twist"),
}

motion_server_default_inputs: Dict[str, Topic] = {
    TopicsKeys.RUN_TESTS: Topic(name="/run_tests", msg_type="Bool"),
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/cmd_vel", msg_type="Twist"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

# PLANNER
planner_allowed_inputs: Dict[str, AllowedTopics] = {
    TopicsKeys.GLOBAL_MAP: AllowedTopics(types=["OccupancyGrid"]),
    TopicsKeys.GOAL_POINT: AllowedTopics(
        types=["Odometry", "PoseStamped", "PointStamped"]
    ),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
}

planner_allowed_outputs: Dict[str, AllowedTopics] = {
    TopicsKeys.GLOBAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.REACHED_END: AllowedTopics(types=["Bool"]),
}


# Default values for inputs / outputs
planner_default_inputs: Dict[str, Topic] = {
    TopicsKeys.GLOBAL_MAP: Topic(
        name="/map",
        msg_type="OccupancyGrid",
        qos_profile=QoSConfig(durability=qos.DurabilityPolicy.TRANSIENT_LOCAL),
    ),
    TopicsKeys.GOAL_POINT: Topic(name="/goal", msg_type="PointStamped"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

planner_default_outputs: Dict[str, Topic] = {
    TopicsKeys.GLOBAL_PLAN: Topic(name="/plan", msg_type="Path"),
    TopicsKeys.REACHED_END: Topic(name="/reached_end", msg_type="Bool"),
}
