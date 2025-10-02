from typing import Optional, Dict, List
import threading
import os
from pathlib import Path
import numpy as np
import yaml
import cv2
from attrs import define, field
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2
from kompass_core.models import RobotGeometry
from kompass_core.datatypes import get_occupancy_grid_from_pcd, get_points_from_pcd
# KOMPASS ROS
from rclpy.topic_endpoint_info import TopicEndpointInfo
from rclpy.callback_groups import ReentrantCallbackGroup
from ..config import BaseValidators, ComponentConfig, ComponentRunType
from ..utils import IntEnum
from .ros import Topic, update_topics
from .component import Component
from .defaults import TopicsKeys, map_server_allowed_outputs, map_server_default_outputs
from kompass_interfaces.srv import Save3dMapToFile, Save2dMapToFile


class MapColors(IntEnum):
    WHITE = 254
    GRAY = 205
    BLACK = 0
    MAX = 255
    MIN = 0


@define(kw_only=True)
class MapServerConfig(ComponentConfig):
    """
    MapServer component configuration parameters

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **map_file_read_rate**
      - `float`, `0.0`
      - Rate to read and convert the map data from file. If zero, converts once.

    * - **pc_publish_row**
      - `bool`, `True`
      - Publish the row point cloud data read from the file. Only applicable if input is `.pcd` file.

    * - **pc_frame**
      - `Optional[str]`, `None`
      - Frame ID to use when publishing the point cloud data. If `None`, uses the world frame.

    * - **map_file_path**
      - `Optional[str]`, `None`
      - Path to the map file.

    * - **custom_map_frame**
      - `Optional[str]`, `None`
      - Custom frame ID for the map.

    * - **grid_resolution**
      - `float`, `0.05`
      - Resolution of the occupancy grid if generating from point cloud.

    * - **z_ground_limit**
      - `float`, `0.01`
      - Points below this height value are considered ground. Used only for point cloud input.

    ```
    """

    map_file_read_rate: float = field(
        default=0.0, validator=BaseValidators.in_range(min_value=0.0, max_value=1e6)
    )  # Rate to read and convert the map data from file, if zero -> converts once
    pc_publish_row: bool = field(
        default=True
    )  # Publish the row point cloud data read from the file. Only applicable if input is .pcd file
    pc_frame: Optional[str] = field(
        default=None
    )  # Frame id to use when publishing the point cloud data. If None, uses the world frame
    map_file_path: Optional[str] = field(default=None)
    custom_map_frame: Optional[str] = field(default=None)
    grid_resolution: float = field(
        default=0.05, validator=BaseValidators.in_range(min_value=1e-6, max_value=1e6)
    )  # Resolution of the occupancy grid if generating from point cloud
    z_ground_limit: float = field(
        default=0.01, validator=BaseValidators.in_range(min_value=-1e6, max_value=1e6)
    )  # Points below this height value are considered ground, used only for point cloud input


class MapServer(Component):
    """
        MapServer component used for reading and converting map data from file to serve a global map.
        Supports both 2D maps with a YAML file and 3D maps with PCD files.

        ## Outputs:

        ```{list-table}
        :widths: 10 40 10 40
        :header-rows: 1
        * - Key Name
          - Allowed Types
          - Number
          - Default

        * - **global_map**
          - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
          - 1
          - `Topic(name="/map", msg_type="OccupancyGrid")` - Global map generated from input data.

        * - **spatial_sensor**
          - [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
          - 1, optional
          - `Topic(name="/row_point_cloud", msg_type="PointCloud2")` - Row point cloud data for visualization or further processing.

        ```

    ## Available Services:

    - Save2dMapToFile: Saves the 2D map to a file (YAML format).
    - Save3dMapToFile: Saves the 3D map to a file (PCD format).

    ## Usage Example:
    ```python
    from kompass.components import MapServerConfig, MapServer
    from kompass.ros import Topic

    # Setup custom configuration
    my_config = MapServerConfig(
            map_file_read_rate=5.0,
            map_file_path="/path/to/your/map.pcd",
            grid_resolution=0.1,
            pc_publish_row=False,
        )

    # Init a MapServer object
    my_map_server = MapServer(component_name="map_server", config=my_config)
    ```
    """

    def __init__(
        self,
        component_name: str,
        config_file: Optional[str] = None,
        config: Optional[MapServerConfig] = None,
        outputs: Optional[Dict[str, Topic]] = None,
        **kwargs,
    ) -> None:
        """__init__.

        :param component_name:
        :type component_name: str
        :param config_file:
        :type config_file: Optional[str]
        :param config:
        :type config: Optional[DriveManagerConfig]
        :param inputs:
        :param outputs:
        :param kwargs:
        :rtype: None
        """

        if not config:
            config = MapServerConfig()

        out_topics = (
            update_topics(map_server_default_outputs, **outputs)
            if outputs
            else map_server_default_outputs
        )

        super().__init__(
            config=config,
            config_file=config_file,
            outputs=out_topics,
            allowed_outputs=map_server_allowed_outputs,
            component_name=component_name,
            allowed_run_types=[ComponentRunType.TIMED],
            **kwargs,
        )
        self.config: MapServerConfig = config

    def init_variables(self):
        """
        Overwrites the init variables method called at Node init
        """
        self._pc_msg: Optional[PointCloud2] = None
        self._grid_data : Optional[np.ndarray] = None
        self._grid_res : Optional[float] = None
        self._grid_origin : Optional[Pose] = None
        self.robot_height = RobotGeometry.get_height(
            self.config.robot.geometry_type, self.config.robot.geometry_params
        )

    def create_all_timers(self):
        """Create all node timers
        """
        super().create_all_timers()
        if self.config.map_file_read_rate > 0.0:
            if self.config.map_file_read_rate < 1.0:
                self.get_logger().warn("map_file_read_rate is too low, consider increasing it to at least 1 Hz!")
            self.__conversion_timer = self.create_timer(1.0 / self.config.map_file_read_rate, self.convert_map_from_file)

    def create_all_services(self):
        """
        Creates all node services
        """
        self.__save_2d_map_srv = self.create_service(
            Save2dMapToFile,
            f"{self.node_name}/save_2d_map_to_file",
            self._save_2d_map_to_file_srv_callback,
        )
        self.__save_3d_map_srv = self.create_service(
            Save3dMapToFile,
            f"{self.node_name}/save_3d_map_to_file",
            self._save_3d_map_to_file_srv_callback,
        )
        super().create_all_services()

    def destroy_all_timers(self):
        """Destroys all node timers
        """
        super().destroy_all_timers()
        if self.config.map_file_read_rate > 0.0:
            self.destroy_timer(self.__conversion_timer)

    def destroy_all_services(self):
        """
        Destroys all node services
        """
        self.destroy_service(self.__save_2d_map_srv)
        self.destroy_service(self.__save_3d_map_srv)
        super().destroy_all_services()

    def convert_map_from_file(self) -> bool:
        """
        Convert the map from file to OccupancyGrid message
        """
        # Check if the file exists
        if not self.config.map_file_path or not os.path.isfile(self.config.map_file_path):
            self.get_logger().error(f"Map file {self.config.map_file_path} does not exist!")
            return False

        # Check if the file is of supported type
        supported_extensions = [".yaml", ".pcd"]  # TODO check ".las", ".laz"]
        if not any(self.config.map_file_path.endswith(ext) for ext in supported_extensions):
            self.get_logger().error(
                f"Map file {self.config.map_file_path} has unsupported extension. Supported extensions are: {supported_extensions}"
            )
            return False

        # Handle PointCloud
        if self.config.map_file_path.endswith(".pcd"):
            (self._grid_data, grid_origin) = get_occupancy_grid_from_pcd(
                self.config.map_file_path,
                grid_resolution=self.config.grid_resolution,
                z_ground_limit=self.config.z_ground_limit,
                robot_height=self.robot_height
            )
            self._grid_res = self.config.grid_resolution
            self._grid_origin = Pose()
            self._grid_origin.position.x = grid_origin[0]
            self._grid_origin.position.y = grid_origin[1]
            self._grid_origin.position.z = grid_origin[2]

            if self.config.pc_publish_row:
                self._create_pointcloud2()

        elif self.config.map_file_path.endswith(".yaml"):
            # READ 2D MAP
            self._read_map_from_yaml(self.config.map_file_path)
            self._pc_msg = None

        self.get_logger().info(
            f"Loaded 2D map from {self.config.map_file_path} with size {self._grid_data.shape}.")
        return True

    def _read_map_from_yaml(self, yaml_path: str):
        """ Read a 2D map from a YAML file and convert it to an OccupancyGrid message

        :param yaml_path: Path to the YAML file
        :type yaml_path: str
        :raises FileNotFoundError: If the image file is not found
        """
        file_root = Path(yaml_path).parent.resolve()
        # Load YAML metadata
        with open(yaml_path, "r") as f:
            map_metadata = yaml.safe_load(f)

        image_path = map_metadata["image"]
        resolution = float(map_metadata["resolution"])
        origin = map_metadata["origin"]
        negate = bool(map_metadata["negate"])
        occupied_thresh = float(map_metadata["occupied_thresh"])
        free_thresh = float(map_metadata["free_thresh"])

        img = cv2.imread(file_root / Path(image_path), -1)

        if img is None:
            raise FileNotFoundError(f"No valid map is found at: {image_path}")

        img_normalized = img.astype(np.float32) / float(MapColors.MAX)

        # Negate if needed (invert black/white meaning)
        if not negate:
            img_normalized = 1.0 - img_normalized

        # Convert to occupancy values
        data = np.full(img_normalized.shape, -1, dtype=np.int8)

        data[img_normalized >= occupied_thresh] = 100
        data[img_normalized <= free_thresh] = 0

        # OccupancyGrid items
        self._grid_data = data
        self._grid_res = resolution
        self._grid_origin = Pose()
        self._grid_origin.position.x = origin[0]
        self._grid_origin.position.y = origin[1]

    def _save_map_to_yaml(
        self, msg: OccupancyGrid, yaml_filename: str, image_filename: str, occupied_thresh: float, free_thresh: float
    ) -> bool:
        """ Save an OccupancyGrid message to a YAML file and a PGM image

        :param msg: OccupancyGrid message
        :type msg: OccupancyGrid
        :param yaml_filename: Path to the output YAML file
        :type yaml_filename: str
        :param image_filename: Path to the output PGM image file
        :type image_filename: str
        :param occupied_thresh: Threshold above which cells are considered occupied (0-1)
        :type occupied_thresh: float
        :param free_thresh: Threshold below which cells are considered free (0-1)
        :type free_thresh: float

        :return: True if successful, False otherwise
        :rtype: bool
        """
        try:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = [
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                0.0,
            ]

            # Convert occupancy grid data to image
            data = np.array(msg.data, dtype=np.int8).reshape((
                height,
                width,
            ))
            img = np.zeros_like(data, dtype=np.uint8)

            # Map occupancy grid values to grayscale
            img[data == -1] = MapColors.GRAY  # Unknown -> gray
            img[data >= occupied_thresh * 100] = MapColors.BLACK    # Occupied -> black
            img[data <= free_thresh * 100] = MapColors.WHITE  # Free -> white

            # Flip vertically (ROS uses bottom-left origin)
            img = cv2.flip(img, 0)

            # Save as PGM (binary, P5 format)
            cv2.imwrite(image_filename, img)

            # Write YAML file
            yaml_data = {
                "image": os.path.basename(image_filename),
                "mode": "trinary",
                "resolution": resolution,
                "origin": origin,
                "negate": 0,
                "occupied_thresh": occupied_thresh,
                "free_thresh": free_thresh,
            }

            with open(yaml_filename, "w") as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")
            return False

    def _create_pointcloud2(self) -> None:
        """ Create a PointCloud2 message from a Nx3 numpy array of points"""
        pc_points = get_points_from_pcd(self.config.map_file_path)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.config.pc_frame or self.config.frames.world
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        self._pc_msg = pc2.create_cloud(header=header, fields=fields, points=pc_points)

    # SERVICES CALLBACKS
    def _save_map_srv_prep(self, msg_type, request) -> bool:
        """Prepare the save map service by subscribing to the topic and waiting for a message

        :param msg_type: Message type to subscribe to (OccupancyGrid or PointCloud2)
        :type msg_type: type
        :param request: Service request
        :type request: Save2dMapToFile.Request or Save3dMapToFile.Request

        :return: True if successful, False otherwise
        :rtype: bool
        """
        self.get_logger().info(f"RECEIVED SAVE {msg_type} MAP SERVICE REQUEST")
        self._save_map_data: Optional[PointCloud2] = None
        # Confirm the topic exists and is being published
        topic_end_point_info: List[TopicEndpointInfo] = (
            self.get_publishers_info_by_topic(request.topic_name)
        )
        if not topic_end_point_info:
            self.get_logger().error(
                f"Topic {request.topic_name} is not being published, cannot execute save map request"
            )
            return False

        self._map_msg_future = threading.Event()

        # create subscriber to get the current map
        self._save_map_subscriber = self.create_subscription(
            msg_type=msg_type,
            topic=request.topic_name,
            callback=self._save_map_callback,
            callback_group=ReentrantCallbackGroup(),
            qos_profile=10,
        )

        while not self._map_msg_future.is_set() and (
            request.timeout <= 0.0 or self._map_msg_future.wait(timeout=request.timeout)
        ):
            self.get_logger().debug("Waiting for map data to be received...")

        if not self._map_msg_future.is_set():
            self.get_logger().error(
                "Timeout waiting for map data, aborting save map request"
            )
            return False
        return True

    def _save_3d_map_to_file_srv_callback(
        self, request: Save3dMapToFile.Request, response: Save3dMapToFile.Response
    ) -> Save3dMapToFile.Response:
        """
        Save 3D Map to File service callback

        :param request: ROS service request
        :type request: Save3dMapToFile.request
        :param response: ROS service response
        :type response: Save3dMapToFile.response
        :return: Response
        :rtype: Save3dMapToFile.response
        """
        if not self._save_map_srv_prep(PointCloud2, request):
            response.success = False
            if hasattr(self, '_save_map_subscriber'):
                self.destroy_subscription(self._save_map_subscriber)
            return response

        save_path = os.path.join(
            request.save_file_location + request.save_file_name + ".pcd"
        )
        points = pc2.read_points(
                self._save_map_data, field_names=("x", "y", "z"), skip_nans=True
            )
        # Convert structured array to plain Nx3 float array
        points_array = np.vstack((
            points["x"],
            points["y"],
            points["z"],
        )).T
        # Open3D expects float64
        points_array = points_array.astype(np.float64)
        try:
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_array)
            o3d.io.write_point_cloud(save_path, pcd)
            self.get_logger().info(f"Saved point cloud map to {save_path}")
            response.success = True
        except ImportError:
            self.get_logger().error("open3d is not installed => Cannot save point cloud. Please run 'pip install open3d' it to handle '.pcd' files.")
            response.success = False

        self.destroy_subscription(self._save_map_subscriber)

        return response

    def _save_2d_map_to_file_srv_callback(
        self, request: Save2dMapToFile.Request, response: Save2dMapToFile.Response
    ) -> Save2dMapToFile.Response:
        """
        Save 2D Map to File service callback

        :param request: ROS service request
        :type request: Save2dMapToFile.request
        :param response: ROS service response
        :type response: Save2dMapToFile.response
        :return: Response
        :rtype: Save2dMapToFile.response
        """
        if not self._save_map_srv_prep(OccupancyGrid, request):
            response.success = False
            if hasattr(self, "_save_map_subscriber"):
                self.destroy_subscription(self._save_map_subscriber)
            return response

        response.success = self._save_map_to_yaml(
            msg=self._save_map_data,
            yaml_filename=os.path.join(
                request.save_file_location, request.save_file_name + ".yaml"
            ),
            image_filename=os.path.join(
                request.save_file_location, request.save_file_name + ".pgm"
            ),
            occupied_thresh=request.occupied_thresh,
            free_thresh=request.free_thresh
        )

        self.destroy_subscription(self._save_map_subscriber)

        return response

    def _save_map_callback(self, msg) -> None:
        """
        Callback to save the map data received from the topic

        :param msg: Map message
        :type msg: OccupancyGrid or PointCloud2
        """
        self.get_logger().info("Map received...")

        self._save_map_data = msg
        self._map_msg_future.set()

    def _execution_step(self):
        """
        Main execution of the component, executed at ech timer tick with rate self.config.loop_rate
        """
        if self._pc_msg is not None and self.config.pc_publish_row:
            self.get_publisher(TopicsKeys.SPATIAL_SENSOR).publish(
                self._pc_msg
            )
        if self._grid_data is None:
            self.get_logger().debug("No map data to publish")
            return
        self.get_publisher(TopicsKeys.GLOBAL_MAP).publish(
            self._grid_data, resolution=self._grid_res, origin=self._grid_origin, frame_id=self.config.custom_map_frame or self.config.frames.world
        )

    def _execute_once(self):
        if self.config.map_file_read_rate == 0.0 and self.config.map_file_path:
            self.convert_map_from_file()
