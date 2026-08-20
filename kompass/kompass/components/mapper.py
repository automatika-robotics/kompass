from typing import Dict, List, Optional, Tuple
import time
from functools import partial
from attrs import define, field, Factory
import numpy as np

# ROS MSGS
from geometry_msgs.msg import Pose

# KOMPASS CORE
from kompass_core.mapping import MapConfig
from kompass_core.mapping import LocalMapper as LocalMapperHandler
from kompass_core.datatypes.scan_model import ScanModelConfig
from kompass_core.datatypes.pose import PoseData
from kompass_core.models import RobotState
from ros_sugar.io import LaserScanData
from kompass_core.models import RobotGeometry
from kompass_cpp.types import PointFieldType, SensorConfig

# KOMPASS ROS
from ..config import BaseValidators, ComponentConfig
from ..callbacks import LaserScanCallback, PointCloudCallback
from .ros import Topic, update_topics
from .component import Component

from .defaults import (
    TopicsKeys,
    mapper_allowed_inputs,
    mapper_allowed_outputs,
    mapper_default_inputs,
    mapper_default_outputs,
)


@define
class LocalMapperConfig(ComponentConfig):
    """
    LocalMapperConfig parameters
    """

    map_params: MapConfig = field(default=Factory(MapConfig))
    scan_model: ScanModelConfig = field(default=Factory(ScanModelConfig))
    sensor_data_timeout: float = field(
        default=0.2, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e3)
    )  # Maximum sensor message age (seconds) before its cloud is skipped in the fused update


class LocalMapper(Component):
    """
    This component is responsible for generating this local map during the navigation.


    ```{note}
    Supported sensor input is either ONE LaserScan topic or up to 11 PointCloud2 topics. Multiple point clouds (e.g. front + back 3D lidars) are fused into a single occupancy grid: each sensor's mount transform is applied inside the core and free space is carved from each sensor's own origin
    ```


    ## Available Run Types
    Set directly from LocalMapper 'run_type' property.

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
      - `LaserScan, PointCloud2`
      - 1 to 11
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
        from kompass.components import LocalMapper, LocalMapperConfig
        from kompass_core.mapping import MapConfig as MapperConfig

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
        inputs: Optional[Dict[str, Topic]] = None,
        outputs: Optional[Dict[str, Topic]] = None,
        **kwargs,
    ) -> None:
        self.config: LocalMapperConfig = config or LocalMapperConfig()

        # Update defaults from custom topics if provided
        in_topics = (
            update_topics(mapper_default_inputs, **inputs)
            if inputs
            else mapper_default_inputs
        )
        out_topics = (
            update_topics(mapper_default_outputs, **outputs)
            if outputs
            else mapper_default_outputs
        )

        super().__init__(
            config=self.config,
            config_file=config_file,
            inputs=in_topics,
            outputs=out_topics,
            allowed_inputs=mapper_allowed_inputs,
            allowed_outputs=mapper_allowed_outputs,
            component_name=component_name,
            **kwargs,
        )

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self.robot_state: Optional[RobotState] = (
            None  # robot current state - to be updated from odom
        )

        self._local_map_frame: str = self.config.frames.world  # Default local map frame is world frame, will be updated to odom frame if the transform from odom to world is not available

        self.robot_height = RobotGeometry.get_height(
            self.robot_geometry_type, self.robot.geometry_params
        )

        self.sensor_data: Optional[LaserScanData] = None

        # Classify the spatial sensor topics
        pc_callbacks: List[PointCloudCallback] = []
        pc_indices: List[int] = []
        scan_callbacks: List[LaserScanCallback] = []
        num_sensors = self._inputs_keys.count(TopicsKeys.SPATIAL_SENSOR)
        for idx in range(num_sensors):
            callback = self.get_callback(TopicsKeys.SPATIAL_SENSOR, idx)
            # N PointCloud2 topics -> multi-sensor pointcloud mode
            # mount transforms applied inside
            if isinstance(callback, PointCloudCallback):
                pc_callbacks.append(callback)
                pc_indices.append(idx)
            # laserscan mode (data pre-transformed into the body frame by the callback)
            elif isinstance(callback, LaserScanCallback):
                scan_callbacks.append(callback)

        self._pc_callbacks: Tuple[PointCloudCallback, ...] = tuple(pc_callbacks)
        self._pc_indices: List[int] = pc_indices
        self._pc_last_msg: List[float] = []
        self._sensor_timeout: float = self.config.sensor_data_timeout
        self._scan_callback: Optional[LaserScanCallback] = None
        # one laserscan XOR N pointclouds
        self._invalid_sensor_setup: bool = not (
            (pc_callbacks and not scan_callbacks)
            or (len(scan_callbacks) == 1 and not pc_callbacks)
        )
        self._local_map_builder: Optional[LocalMapperHandler] = None

        if self._invalid_sensor_setup or pc_callbacks:
            # Pointcloud mode. The handler is built at activation
            return

        # Laserscan mode (single sensor)
        self._scan_callback = scan_callbacks[0]
        self._local_map_builder = LocalMapperHandler(
            config=self.config.map_params, scan_model_config=self.config.scan_model
        )

        # Sensor data is mapped in the robot body frame. The sensor's own frame
        # comes from the incoming messages, so nothing has to be configured
        self.transform_inputs_to(
            TopicsKeys.SPATIAL_SENSOR, self.config.frames.robot_base, static_tf=True
        )

        self._scan_callback.on_callback_execute(self._update_map_from_scan)

    def _update_state(self) -> None:
        """
        Updates node inputs from associated callbacks
        """

        location_callback = self.get_callback(TopicsKeys.ROBOT_LOCATION)
        self.robot_state: Optional[RobotState] = location_callback.get_output(
            transformation=self.odom_tf_listener.transform
            if self.odom_tf_listener
            else None
        )

        if self.odom_tf_listener and self.odom_tf_listener.got_transform:
            # The transform to the world frame is available, so robot_state has
            # been transformed and the local map is built in the world frame
            self._local_map_frame = self.config.frames.world
        else:
            # No transform yet, so robot_state is still in whatever frame the
            # location messages arrived in, and so is the local map
            self._local_map_frame = (
                location_callback.frame_id or self.config.frames.world
            )

        # In laserscan mode the sensor->body transform is applied by the callback
        if self._scan_callback is not None:
            self.sensor_data = self._scan_callback.get_output()

    def publish_data(self):
        """
        Publish layers and obstacles mapped from LocalMapper
        """
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

        # Publish occupancy grid data to ROS
        self.get_publisher(TopicsKeys.LOCAL_MAP_OCC).publish(
            self._local_map_builder.occupancy,
            frame_id=self._local_map_frame,
            origin=origin_pose_msg,
            width=self._local_map_builder.grid_width,
            height=self._local_map_builder.grid_height,
            resolution=self.config.map_params.resolution,
        )

    def _robot_pose_in_world(self) -> PoseData:
        """Builds the planar robot pose used to center the local map"""
        pose_robot_in_world = PoseData()
        pose_robot_in_world.x = self.robot_state.x
        pose_robot_in_world.y = self.robot_state.y
        pose_robot_in_world.qz = np.sin(self.robot_state.yaw / 2)
        pose_robot_in_world.qw = np.cos(self.robot_state.yaw / 2)
        return pose_robot_in_world

    def _update_map_from_scan(self, *_, **__):
        """Update local map from laserscan data (callback-driven)"""
        if self.sensor_data is None or self.robot_state is None:
            return

        self._local_map_builder.update_from_laserscan(
            self._robot_pose_in_world(),
            ranges=self.sensor_data.ranges,
            angles=self.sensor_data.angles,
        )

    def _update_map_from_clouds(self):
        """Fuses the current point clouds into the local map (tick-driven).

        A sensor with no fresh data (older than `sensor_data_timeout`)
        contributes a None slot. If NO sensor has fresh data the update is skipped
        entirely, keeping the last grid instead of wiping it.
        """
        if self.robot_state is None:
            self.get_logger().warn(
                "Robot state is not available, skipping local map update!"
            )
            self.health_status.set_fail_system(
                topic_names=[self.get_in_topic(TopicsKeys.ROBOT_LOCATION).name]
            )
            return
        now = time.monotonic()
        clouds: List[Optional[dict]] = []
        fresh = 0
        for i, callback in enumerate(self._pc_callbacks):
            pc = (
                callback.get_output()
                if now - self._pc_last_msg[i] <= self._sensor_timeout
                else None
            )
            if pc is None:
                clouds.append(None)
                continue
            # Metadata-only view of the raw buffer (zero-copy contract)
            clouds.append({
                "data": pc.data,
                "point_step": pc.point_step,
                "row_step": pc.row_step,
                "height": pc.height,
                "width": pc.width,
                "x_offset": pc.x_offset,
                "y_offset": pc.y_offset,
                "z_offset": pc.z_offset,
            })
            fresh += 1
        if not fresh:
            self.get_logger().warn(
                "New sensor data is not available, skipping local map update!"
            )
            return
        self._local_map_builder.update_from_pointclouds(
            self._robot_pose_in_world(), clouds=clouds
        )

    def _wait_first_sensor_output_and_tf(self, callback) -> tuple:
        """Blocks until the given sensor callback delivers its first output

        :param callback: Sensor topic callback
        """
        output = callback.get_output()
        _timeout = 0.0
        while output is None and _timeout <= self.config.topic_subscription_timeout:
            self.get_logger().info(
                "Waiting for sensor data to initialize the local mapper..",
                once=True,
            )
            time.sleep(self.config.topic_try_wait_timeout)
            _timeout += self.config.topic_try_wait_timeout
            output = callback.get_output()
        # Get TF
        if not output or not callback.frame_id:
            static_tf = None
        else:
            static_tf = self.get_transform_listener(
                callback.frame_id, self.config.frames.robot_base, static_tf=True
            )
        return (output, static_tf)

    def _stamp_pc_arrival(self, sensor_idx: int, **_):
        """Record a pointcloud sensor message arrival (staleness gating)"""
        self._pc_last_msg[sensor_idx] = time.monotonic()

    def _execute_once(self):
        """Actions to be executed once at the start of the component execution"""
        super()._execute_once()

        if self._invalid_sensor_setup:
            self.get_logger().error(
                "LocalMapper supports either ONE LaserScan sensor or N PointCloud2 "
                "sensors, not a mix or multiple laserscans -> Mapping is disabled!"
            )
            sensor_names = self.in_topic_name(TopicsKeys.SPATIAL_SENSOR) or []
            self.health_status.set_fail_system(
                topic_names=sensor_names
                if isinstance(sensor_names, list)
                else [sensor_names]
            )
            self.health_status_publisher.publish(self.health_status())
            return

        if not self._pc_callbacks:
            # Laserscan mode. Handler already built at init
            return

        # Multi-sensor pointcloud mode. One SensorConfig per sensor (mount
        # pose from TF, point field encoding from the sensor's first message)
        sensor_configs = []
        for callback, _ in zip(self._pc_callbacks, self._pc_indices, strict=True):
            cloud, sensor_tf = self._wait_first_sensor_output_and_tf(callback)
            if not cloud:
                self.get_logger().error(
                    f"Failed to initialize LocalMapper, PointCloud2 sensor {callback.input_topic.name} did not deliver any data -> Declaring fail and attempting again in {1 / self.config.loop_rate:.1f} seconds!"
                )
                self.health_status.set_fail_system(
                    topic_names=[callback.input_topic.name]
                )
                self.health_status_publisher.publish(self.health_status())
                time.sleep(1 / self.config.loop_rate)
                return self._execute_once()  # Try again next tick

            if not sensor_tf or not sensor_tf.transform:
                self.get_logger().error(
                    f"Sensor TF for {callback.input_topic.name} is not available -> Declaring fail and attempting again in {1 / self.config.loop_rate:.1f} seconds!"
                )
                self.health_status.set_fail_system(
                    topic_names=[callback.input_topic.name]
                )
                self.health_status_publisher.publish(self.health_status())
                time.sleep(1 / self.config.loop_rate)
                return self._execute_once()  # Try again next tick

            sensor_configs.append(
                SensorConfig(
                    position=sensor_tf.translation,
                    rotation=sensor_tf.rotation,
                    cloud_field_type=PointFieldType.from_int(cloud.x_field_datatype),
                )
            )
        self._local_map_builder = LocalMapperHandler(
            config=self.config.map_params,
            scan_model_config=self.config.scan_model,
            sensors=sensor_configs,
        )

        # Arrival stamps for staleness gating
        now = time.monotonic()
        self._pc_last_msg = [now] * len(self._pc_callbacks)
        for i, callback in enumerate(self._pc_callbacks):
            callback.on_callback_execute(
                partial(self._stamp_pc_arrival, sensor_idx=i), get_processed=False
            )
        self.get_logger().info(
            f"LocalMapper initialized with {len(self._pc_callbacks)} pointcloud sensor(s)"
        )

    def _execution_step(self):
        """
        LocalMapper main execution step
        """
        if self._local_map_builder is None:
            # Pointcloud mode before activation completes (or invalid setup)
            return

        # Get inputs from callbacks
        self._update_state()

        # Per tick for (Multi) PointCloud case
        if self._pc_callbacks:
            self._update_map_from_clouds()

        # Check if all inputs are available
        if self.got_all_inputs():
            self.publish_data()
