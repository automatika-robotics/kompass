from typing import Optional, Union
from attrs import define, field, Factory
import numpy as np

# ROS MSGS
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose

# KOMPASS
from kompass_core.mapping.local_mapper import LocalMapperConfig as LocalMapperHandlerConfig
from kompass_core.mapping.local_mapper import LocalMapper as LocalMapperHandler
from kompass_core.mapping.laserscan_model import LaserScanModelConfig
from kompass_core.datatypes.pose import PoseData
from kompass_core.models import Robot, RobotState
from kompass_core.datatypes.laserscan import LaserScanData
from kompass_core.datatypes.pointcloud import PointCloudData

# KOMPASS ROS
from ..config import ComponentConfig
from ..topic import (
    AllowedTopic,
    RestrictedTopicsConfig,
    Topic,
    create_topics_config,
    update_topics_config,
)
from .component import Component, TFListener


class LocalMapperInputs(RestrictedTopicsConfig):
    # Restricted Topics Config for LocalMapper component authorized input topics

    SENSOR_DATA: AllowedTopic = AllowedTopic(
        key="sensor_data", types=["LaserScan", "PointCloud2"]
    )
    LOCATION: AllowedTopic = AllowedTopic(key="location", types=["Odometry"])


class LocalMapperOutputs(RestrictedTopicsConfig):
    # Restricted Topics Config for LocalMapper component authorized output topics

    MAP_OCC = AllowedTopic(key="occupancy_layer", types=["OccupancyGrid"])
    # MAP_PROP = AllowedTopic(key="probability_layer", types=["OccupancyGrid"])


# Create default inputs - Used if no inputs config is provided to the controller
_mapper_default_inputs = create_topics_config(
    "LocalMapperInputs",
    sensor_data=Topic(name="/scan", msg_type="LaserScan"),
    location=Topic(name="/odom", msg_type="Odometry"),
)

# Create default outputs - Used if no outputs config is provided to the controller
_mapper_default_outputs = create_topics_config(
    "LocalMapperOutputs", occupancy_layer=Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid")
    # , probability_layer=Topic(name="/local_map/probability_layer", msg_type="OccupancyGrid")
)


@define
class LocalMapperConfig(ComponentConfig):
    """
    LocalMapperConfig parameters
    """
    map_params: LocalMapperHandlerConfig = field(default=Factory(LocalMapperHandlerConfig))
    laserscan_model : LaserScanModelConfig = field(default=Factory(LaserScanModelConfig))


class LocalMapper(Component):
    def __init__(
        self,
        *,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[LocalMapperConfig] = None,
        inputs=None,
        outputs=None,
        **kwargs,
    ) -> None:
        self.config: LocalMapperConfig = config or LocalMapperConfig()

        # Get default component inputs/outputs
        in_topics = _mapper_default_inputs()
        out_topics = _mapper_default_outputs()

        if inputs:
            in_topics = update_topics_config(in_topics, **inputs)

        if outputs:
            out_topics = update_topics_config(out_topics, **outputs)

        super().__init__(
            config=self.config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=LocalMapperInputs,
            allowed_outputs=LocalMapperOutputs,
            component_name=component_name,
            **kwargs,
        )
        self.config.map_params.resolution = 0.05

    def attach_callbacks(self) -> None:
        """
        Attaches method to update local map from scan
        """
        # Adds callback to set the path in the controller when a new plan is received
        self.callbacks[LocalMapperInputs.SENSOR_DATA.key].on_callback_execute(
            self._update_map_from_scan
        )

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.robot_state: Optional[RobotState] = (
            None  # robot current state - to be updated from odom
        )

        self.__robot = Robot(
            robot_type=self.config.robot.model_type,
            geometry_type=self.config.robot.geometry_type,
            geometry_params=self.config.robot.geometry_params,
            state=self.robot_state,
        )

        self.__sensor_tf_listener: TFListener = (
            self.depth_tf_listener
            if self._input_topics.sensor_data.msg_type._ros_type == PointCloud2
            else self.scan_tf_listener
        )

        self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = None

        self._local_map_builder = LocalMapperHandler(
            config=self.config.map_params,
            scan_model_config=self.config.laserscan_model
        )

    def _update_state(self) -> None:
        """
        Updates node inputs from associated callbacks
        """

        self.robot_state: Optional[RobotState] = self.callbacks[
            LocalMapperInputs.LOCATION.key
        ].get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

        self.sensor_data: Optional[Union[LaserScanData, PointCloudData]] = (
            self.callbacks[
                LocalMapperInputs.SENSOR_DATA.key
            ].get_output(
                transformation=self.__sensor_tf_listener.transform
                if self.__sensor_tf_listener
                else None
            )
        )

    def publish_data(self):
        """
        Publish layers and obstacles mapped from LocalMapper
        """
        if not self._local_map_builder.processed:
            return

        # Get map origin
        origin_pose_msg = Pose()
        origin_pose_msg.position.x = self._local_map_builder.lower_right_corner_pose.x
        origin_pose_msg.position.y = self._local_map_builder.lower_right_corner_pose.y
        origin_pose_msg.position.z = self._local_map_builder.lower_right_corner_pose.z
        origin_pose_msg.orientation.x = self._local_map_builder.lower_right_corner_pose.qx
        origin_pose_msg.orientation.y = self._local_map_builder.lower_right_corner_pose.qy
        origin_pose_msg.orientation.z = self._local_map_builder.lower_right_corner_pose.qz
        origin_pose_msg.orientation.w = self._local_map_builder.lower_right_corner_pose.qw

        msg_header = Header()
        msg_header.stamp = self.get_ros_time()
        msg_header.frame_id = self.config.frames.world

        # Publish occupancy grid data to ROS
        self.publishers_dict[LocalMapperOutputs.MAP_OCC.key].publish(
            self._local_map_builder.occupancy,
            msg_header=msg_header,
            origin=origin_pose_msg,
            width=self._local_map_builder.grid_width,
            height=self._local_map_builder.grid_height,
            resolution=self.config.map_params.resolution
        )

    def _update_map_from_scan(self, **_):
        """Update local map from scan
        """
        if not self.sensor_data:
            return

        pose_robot_in_world = PoseData()
        pose_robot_in_world.x = self.robot_state.x
        pose_robot_in_world.y = self.robot_state.y
        pose_robot_in_world.qz = np.sin(self.robot_state.yaw / 2)
        pose_robot_in_world.qw = np.cos(self.robot_state.yaw / 2)

        self._local_map_builder.scan_update_model.range_max = self.sensor_data.range_max

        self._local_map_builder.update_from_scan(
            pose_robot_in_world, self.sensor_data
        )

    def _execution_step(self):
        """
        LocalMapper main execution step
        """
        super()._execution_step()
        # Get inputs from callbacks
        self._update_state()

        # Check if all inputs are available
        if self.got_all_inputs():
            self.publish_data()
