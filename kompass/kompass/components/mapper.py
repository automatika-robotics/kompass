from typing import Optional
from attrs import define, field, Factory
import numpy as np

# ROS MSGS
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

# KOMPASS
from kompass_core.mapping import MapConfig
from kompass_core.mapping import LocalMapper as LocalMapperHandler
from kompass_core.mapping.laserscan_model import LaserScanModelConfig
from kompass_core.datatypes.pose import PoseData
from kompass_core.models import RobotState
from kompass_core.datatypes.laserscan import LaserScanData

# KOMPASS ROS
from ..config import ComponentConfig
from ..topic import (
    AllowedTopic,
    RestrictedTopicsConfig,
    Topic,
    create_topics_config,
    update_topics_config,
)
from .component import Component


class LocalMapperInputs(RestrictedTopicsConfig):
    # Restricted Topics Config for LocalMapper component authorized input topics

    SENSOR_DATA: AllowedTopic = AllowedTopic(key="sensor_data", types=["LaserScan"])
    LOCATION: AllowedTopic = AllowedTopic(key="location", types=["Odometry"])


class LocalMapperOutputs(RestrictedTopicsConfig):
    # Restricted Topics Config for LocalMapper component authorized output topics

    MAP_OCC = AllowedTopic(key="occupancy_layer", types=["OccupancyGrid"])


# Create default inputs - Used if no inputs config is provided to the controller
_mapper_default_inputs = create_topics_config(
    "LocalMapperInputs",
    sensor_data=Topic(name="/scan", msg_type="LaserScan"),
    location=Topic(name="/odom", msg_type="Odometry"),
)

# Create default outputs - Used if no outputs config is provided to the controller
_mapper_default_outputs = create_topics_config(
    "LocalMapperOutputs",
    occupancy_layer=Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid"),
)


@define
class LocalMapperConfig(ComponentConfig):
    """
    LocalMapperConfig parameters
    """

    map_params: MapConfig = field(default=Factory(MapConfig))
    laserscan_model: LaserScanModelConfig = field(default=Factory(LaserScanModelConfig))


class LocalMapper(Component):
    """
    This component is responsible for generating this local map during the navigation.


    ```{note}
    Current implementation supports LaserScan sensor data to create an Occupancy Grid local map. PointCloud and semantic information will be supported in an upcoming release
    ```


    ## Available Run Types
    Set from ControllerConfig class or directly from Controller 'run_type' property.

    ```{list-table}
    :widths: 10 80
    * - **Timed**
      - Produces a local map periodically if all inputs are available
    ```

    ## Inputs

    ```{list-table}
    :widths: 10 30 15 20 20
    :header-rows: 1
    * - Key
      - Description
      - Accepted Types
      - Number of Topics
      - Default Value

    * - **location**
      - Robot current location
      - `Odometry, PoseWithCovariance, Pose`
      - 1
      - `Topic(name="/odom", msg_type="Odometry")`

    * - **sensor_data**
      - Direct sensor input
      - `LaserScan`
      - 1
      - `Topic(name="/scan", msg_type="LaserScan")`

    ```

    ## Outputs

    ```{list-table}
    :widths: 10 30 15 20
    :header-rows: 1
    * - Key
      - Description
      - Accepted Types
      - Default Value

    * - **local_map**
      - Local occupancy map
      - `OccupancyGrid`
      - `Topic(name="/scan", msg_type="LaserScan")`
    ```


    ## Usage Example:
    ```python
        from kompass_core.mapping import LocalMapperConfig
        from kompass.components import LocalMapper, MapperConfig

        # Select map parameters
        map_params = MapperConfig(width=5.0, height=5.0, resolution=0.2) # 5mX5m map with 0.2m/cell resolution

        # Setup custom component configuration
        my_config = LocalMapperConfig(loop_rate=10.0, map_params=map_params)

        # Init a mapper
        my_mapper = LocalMapper(component_name="mapper", config=my_config)
    ```
    """

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

        self.sensor_data: Optional[LaserScanData] = None

        self._local_map_builder = LocalMapperHandler(
            config=self.config.map_params, scan_model_config=self.config.laserscan_model
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

        self.sensor_data: Optional[LaserScanData] = self.callbacks[
            LocalMapperInputs.SENSOR_DATA.key
        ].get_output(
            transformation=self.scan_tf_listener.transform
            if self.scan_tf_listener
            else None
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
        origin_pose_msg.orientation.x = (
            self._local_map_builder.lower_right_corner_pose.qx
        )
        origin_pose_msg.orientation.y = (
            self._local_map_builder.lower_right_corner_pose.qy
        )
        origin_pose_msg.orientation.z = (
            self._local_map_builder.lower_right_corner_pose.qz
        )
        origin_pose_msg.orientation.w = (
            self._local_map_builder.lower_right_corner_pose.qw
        )

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
            resolution=self.config.map_params.resolution,
        )

    def _update_map_from_scan(self, **_):
        """Update local map from scan"""
        if not self.sensor_data or not self.robot_state:
            return

        pose_robot_in_world = PoseData()
        pose_robot_in_world.x = self.robot_state.x
        pose_robot_in_world.y = self.robot_state.y
        pose_robot_in_world.qz = np.sin(self.robot_state.yaw / 2)
        pose_robot_in_world.qw = np.cos(self.robot_state.yaw / 2)

        self._local_map_builder.scan_update_model.range_max = self.sensor_data.range_max

        self._local_map_builder.update_from_scan(pose_robot_in_world, self.sensor_data)

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
