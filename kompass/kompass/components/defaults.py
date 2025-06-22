"""
Kompass stack default inputs/outputs values and allowed topic types for each component
"""

from typing import Dict, Optional
from ros_sugar.supported_types import add_additional_datatypes
from .. import data_types
from ros_sugar.io import AllowedTopics, get_all_msg_types
from ros_sugar.config import QoSConfig
from ros_sugar.io import Topic
from rclpy import qos
from ..utils import StrEnum

# Get Kompass types to pass to the base component as additional supported types
add_additional_datatypes(get_all_msg_types(data_types))


class TopicsKeys(StrEnum):
    """Unique stack keys associated with inputs/outputs

    ```{list-table}
    :widths: 20 20 60
    :header-rows: 1

    * - Key
      - Name
      - Description
    * - GOAL_POINT
      - goal_point
      - Target destination point on the map for the robot point navigation
    * - GLOBAL_PLAN
      - plan
      - Global navigation plan (path) from start to goal
    * - GLOBAL_MAP
      - map
      - Global (reference) map used for navigation
    * - ROBOT_LOCATION
      - location
      - Current position and orientation of the robot
    * - SPATIAL_SENSOR
      - sensor_data
      - Raw data from robot's spatial sensors (e.g., LIDAR, depth sensors)
    * - VISION_TRACKINGS
      - vision_tracking
      - Visual tracking data from robot's cameras or vision systems
    * - DEPTH_CAM_INFO
      - depth_camera_info
      - Depth camera information which includes camera intrinsics parameters
    * - LOCAL_PLAN
      - local_plan
      - Short-term path plan considering immediate surroundings
    * - INTERMEDIATE_CMD
      - command
      - Robot velocity command produced by the control system
    * - INTERMEDIATE_CMD_LIST
      - multi_command
      - List of intermediate velocity commands
    * - LOCAL_MAP
      - local_map
      - Map of the immediate surroundings for local navigation (control)
    * - LOCAL_MAP_OCC
      - local_map
      - Occupancy grid representation of the local environment
    * - INTERPOLATED_PATH
      - interpolation
      - Interpolated global path
    * - TRACKED_POINT
      - tracked_point
      - Specific point being tracked by the robot's systems on the reference path of reference vision target
    * - FINAL_COMMAND
      - robot_command
      - Final control command sent to robot's driver
    * - EMERGENCY
      - emergency_stop
      - Emergency stop signal for immediate robot halt
    * - REACHED_END
      - reached_end
      - Flag indicating whether the goal point has been reached
    * - RUN_TESTS
      - run_tests
      - Flag to initiate system test procedures
    ```

    """

    # INPUT
    GOAL_POINT = "goal_point"
    # Global
    GLOBAL_PLAN = "plan"
    GLOBAL_MAP = "map"
    # Sensory information
    ROBOT_LOCATION = "location"
    SPATIAL_SENSOR = "sensor_data"
    VISION_DETECTIONS = "vision_detections"
    DEPTH_CAM_INFO = "depth_camera_info"
    # Calculated
    LOCAL_PLAN = "local_plan"
    PATH_SAMPLES = "path_samples"
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


controller_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.GLOBAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry", "PoseStamped", "Pose"]),
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["LaserScan", "PointCloud2"]),
    TopicsKeys.LOCAL_MAP: AllowedTopics(types=["OccupancyGrid"]),
    TopicsKeys.VISION_DETECTIONS: AllowedTopics(types=["Detections"]),
    TopicsKeys.DEPTH_CAM_INFO: AllowedTopics(types=["CameraInfo"]),
}

controller_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.INTERMEDIATE_CMD_LIST: AllowedTopics(types=["TwistArray"]),
    TopicsKeys.INTERPOLATED_PATH: AllowedTopics(types=["Path"]),
    TopicsKeys.LOCAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.PATH_SAMPLES: AllowedTopics(types=["Path"]),
    TopicsKeys.TRACKED_POINT: AllowedTopics(types=["Odometry", "PoseStamped", "Pose"]),
}

# Create default inputs - Used if no inputs config is provided to the controller
controller_default_inputs: Dict[TopicsKeys, Optional[Topic]] = {
    TopicsKeys.GLOBAL_PLAN: Topic(name="/plan", msg_type="Path"),
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.LOCAL_MAP: Topic(
        name="/local_map/occupancy_layer", msg_type="OccupancyGrid"
    ),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
    TopicsKeys.VISION_DETECTIONS: None,  # No default topic is assigned. Should be provided by the user to use the vision tracking action
    TopicsKeys.DEPTH_IMG: None,  # No default topic is assigned. Should be provided by the user to use the vision tracking action
    TopicsKeys.DEPTH_CAM_INFO: None,  # No default topic is assigned. Should be provided by the user to use the vision tracking action
}

# Create default outputs - Used if no outputs config is provided to the controller
controller_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/control", msg_type="Twist"),
    TopicsKeys.INTERMEDIATE_CMD_LIST: Topic(
        name="/control_list", msg_type="TwistArray"
    ),
    TopicsKeys.INTERPOLATED_PATH: Topic(name="/interpolated_path", msg_type="Path"),
    TopicsKeys.TRACKED_POINT: Topic(name="/tracked_point", msg_type="PoseStamped"),
    TopicsKeys.LOCAL_PLAN: Topic(name="/local_path", msg_type="Path"),
    TopicsKeys.PATH_SAMPLES: Topic(name="/debug_samples", msg_type="Path"),
}

# DRIVE MANAGER
driver_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.INTERMEDIATE_CMD_LIST: AllowedTopics(types=["TwistArray"]),
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(
        types=["LaserScan", "Float64", "Float32"],
        number_required=1,
        number_optional=10,
    ),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry", "PoseStamped", "Pose"]),
}

driver_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.FINAL_COMMAND: AllowedTopics(types=["Twist", "TwistStamped"]),
    TopicsKeys.EMERGENCY: AllowedTopics(types=["Bool"]),
}


driver_default_inputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/control", msg_type="Twist"),
    TopicsKeys.INTERMEDIATE_CMD_LIST: Topic(
        name="/control_list", msg_type="TwistArray"
    ),
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

driver_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.FINAL_COMMAND: Topic(name="/cmd_vel", msg_type="Twist"),
    TopicsKeys.EMERGENCY: Topic(name="/emergency_stop", msg_type="Bool"),
}

# LOCAL MAPPER
mapper_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.SPATIAL_SENSOR: AllowedTopics(types=["LaserScan"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry", "PoseStamped", "Pose"]),
}

mapper_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.LOCAL_MAP_OCC: AllowedTopics(types=["OccupancyGrid"]),
}


# Create default inputs - Used if no inputs config is provided to the controller
mapper_default_inputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.SPATIAL_SENSOR: Topic(name="/scan", msg_type="LaserScan"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

# Create default outputs - Used if no outputs config is provided to the controller
mapper_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.LOCAL_MAP_OCC: Topic(
        name="/local_map/occupancy_layer", msg_type="OccupancyGrid"
    ),
}

# MOTION SERVER
motion_server_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.RUN_TESTS: AllowedTopics(types=["Bool"]),
    TopicsKeys.INTERMEDIATE_CMD: AllowedTopics(types=["Twist"]),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
}

motion_server_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.FINAL_COMMAND: AllowedTopics(types=["Twist"]),
}

motion_server_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.FINAL_COMMAND: Topic(name="/control", msg_type="Twist"),
}

motion_server_default_inputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.RUN_TESTS: Topic(name="/run_tests", msg_type="Bool"),
    TopicsKeys.INTERMEDIATE_CMD: Topic(name="/cmd_vel", msg_type="Twist"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

# PLANNER
planner_allowed_inputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.GLOBAL_MAP: AllowedTopics(types=["OccupancyGrid"]),
    TopicsKeys.GOAL_POINT: AllowedTopics(
        types=["Odometry", "PoseStamped", "PointStamped"]
    ),
    TopicsKeys.ROBOT_LOCATION: AllowedTopics(types=["Odometry"]),
}

planner_allowed_outputs: Dict[TopicsKeys, AllowedTopics] = {
    TopicsKeys.GLOBAL_PLAN: AllowedTopics(types=["Path"]),
    TopicsKeys.REACHED_END: AllowedTopics(types=["Bool"]),
}


# Default values for inputs / outputs
planner_default_inputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.GLOBAL_MAP: Topic(
        name="/map",
        msg_type="OccupancyGrid",
        qos_profile=QoSConfig(durability=qos.DurabilityPolicy.TRANSIENT_LOCAL),
    ),
    TopicsKeys.GOAL_POINT: Topic(name="/goal", msg_type="PointStamped"),
    TopicsKeys.ROBOT_LOCATION: Topic(name="/odom", msg_type="Odometry"),
}

planner_default_outputs: Dict[TopicsKeys, Topic] = {
    TopicsKeys.GLOBAL_PLAN: Topic(name="/plan", msg_type="Path"),
    TopicsKeys.REACHED_END: Topic(name="/reached_end", msg_type="Bool"),
}
