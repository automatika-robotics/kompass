from typing import Optional, List, Tuple, Union, Any, Dict
import numpy as np
from rclpy.logging import get_logger

from kompass_core.datatypes import Bbox2D, PointsOfInterest
from kompass_core.vision import DepthDetector
from kompass_core.models import RobotState
from .ros import GenericCallback
from .utils import depth_image_metadata, image_pre_processing

# Conditional import to get EmbodiedAgents vision types callbacks
try:
    from agents import callbacks as EmbodiedAgentsCallbacks
except ImportError:
    EmbodiedAgentsCallbacks = None


__all__ = [
    "TrackingsCallback",
    "DetectionsCallback",
    "PointsOfInterestCallback",
]


def _depth_view_and_meta(depth_msg, meta):
    """Zero-copy depth view + cached (dtype, scale) metadata.

    The raw camera buffer crosses to kompass-core untouched (uint16
    millimeters or float32 meters). The scale maps raw units to meters and reaches
    the core through the follower/detector's ``depth_conversion_factor`` config.

    :param depth_msg: sensor_msgs/Image depth message
    :param meta: Previously cached (dtype, scale) or None on first message
    :return: (depth view, (dtype, scale))
    """
    if meta is None:
        meta = depth_image_metadata(depth_msg.encoding)
    return image_pre_processing(depth_msg, meta[0], 1), meta


def _stamp_seconds(stamp) -> float:
    """ROS time stamp as seconds"""
    return stamp.sec + 1e-9 * stamp.nanosec


def _lift_to_state(
    detector: DepthDetector,
    depth: Union[np.ndarray, Dict[str, Any]],
    target: Union[List[Bbox2D], PointsOfInterest],
    robot_state: RobotState,
) -> Optional[RobotState]:
    """Lifts 2D boxes or points of interest to 3D through the given depth
    source and returns the first result as a robot state (its x, y in the
    world frame).

    :param detector: Depth detector configured with the camera
    :type detector: DepthDetector
    :param depth: Aligned depth image, or a point cloud as its layout keywords
    :type depth: Union[np.ndarray, Dict[str, Any]]
    :param target: 2D boxes or points of interest in the camera image
    :type target: Union[List[Bbox2D], PointsOfInterest]
    :param robot_state: Robot pose in the world frame
    :type robot_state: RobotState
    :return: Position of the first lifted target, None when nothing lifted
    :rtype: Optional[RobotState]
    """
    source = {"depth_img": depth} if isinstance(depth, np.ndarray) else depth
    boxes_3d = detector.compute_3d_detections(
        **source,
        input=target,
        robot_x=robot_state.x,
        robot_y=robot_state.y,
        robot_yaw=robot_state.yaw,
        robot_speed=robot_state.speed,
    )
    if not boxes_3d:
        return None
    center = boxes_3d[0].center
    return RobotState(x=center[0], y=center[1])


class _ExternalDepthMixin:
    """Optional depth source subscribed separately from the detections.

    When the component is given a ``VISION_DEPTH`` input, the depth comes from
    that topic. It can be a depth image aligned with the detection camera, or a
    point cloud handed to the core as its PointCloud2 layout. The external source
    takes precedence over the embedded depth and is paired with the
    detections by message stamp.
    """

    _depth_source: Optional[GenericCallback] = None
    _depth_max_age: float = 0.0
    _depth_camera_frame: Optional[str] = None

    def set_depth_source(
        self,
        callback: GenericCallback,
        max_age: float,
        img_size: Optional[np.ndarray] = None,
        camera_frame: Optional[str] = None,
    ) -> None:
        """Use a separately subscribed depth input to lift the detections.

        :param callback: Callback of the depth input: an Image callback
            (aligned depth image) or a PointCloud callback
        :type callback: GenericCallback
        :param max_age: Largest stamp difference (s) between the detections
            and the depth for the two to be paired
        :type max_age: float
        :param img_size: Size (width, height) of the detection image, from
            the camera intrinsics. Seeds the size the boxes need when the
            detections message carries no image of its own
        :type img_size: Optional[np.ndarray]
        :param camera_frame: Frame the camera intrinsics describe. A depth
            image in any other frame is not registered to the detection camera
            and is rejected as depth (reported at error level)
        :type camera_frame: Optional[str]
        """
        self._depth_source = callback
        self._depth_max_age = max_age
        self._depth_camera_frame = camera_frame
        if img_size is not None and self._img_size is None:
            self._img_size = np.asarray(img_size, dtype=np.int32)

    @property
    def depth(self) -> Optional[Union[np.ndarray, Dict[str, Any]]]:
        """Depth to lift this callback's detections with. The separate depth
        input when one is set, else the depth embedded in the message.

        The separate input is used only when its not stale and its stamp is within
        ``max_age`` of the detections' stamp.

        :return: The aligned depth image view, a point cloud as its
            PointCloud2 layout keywords, or None when no depth is usable
        :rtype: Optional[Union[np.ndarray, Dict[str, Any]]]
        """
        source = self._depth_source
        if source is None:
            return self._depth_image
        if self.msg is None or source.msg is None:
            return None
        # get age between msgs
        age = abs(
            _stamp_seconds(self.msg.header.stamp)
            - _stamp_seconds(source.msg.header.stamp)
        )
        if age > self._depth_max_age:
            get_logger(self.node_name or "VisionDepthSource").warning(
                f"Depth on '{source.input_topic.name}' is {age:.3f}s away from "
                f"the detections, beyond vision_depth_max_age={self._depth_max_age}s "
                "-> no depth this tick",
                throttle_duration_sec=1.0,
            )
            return None
        output = source.get_output()
        if output is None:
            return None
        # for aligned depth image
        if isinstance(output, np.ndarray):
            # discard any depth image in a mismatched frame
            if not self._depth_frame_is_registered(source.frame_id):
                return None
            return output
        # TODO: forward height/width as the image size check once the core's
        # organized-cloud path exists
        # for pointclouds
        return output.buffer_layout()

    def _depth_frame_is_registered(self, frame_id: Optional[str]) -> bool:
        """Whether the depth image is in the frame the camera intrinsics
        describe. A mismatch is reported (throttled, the condition persists)
        and treated as no depth."""
        if (
            not frame_id
            or not self._depth_camera_frame
            or frame_id == self._depth_camera_frame
        ):
            return True
        get_logger(self.node_name or "VisionDepthSource").error(
            f"Depth image on '{self._depth_source.input_topic.name}' is in frame "
            f"'{frame_id}' while the camera intrinsics are for "
            f"'{self._depth_camera_frame}'. The depth image must be registered to "
            "the detection camera -> no depth this tick",
            throttle_duration_sec=5.0,
        )
        return False


# EmbodiedAgents Conditional Callback Classes
if EmbodiedAgentsCallbacks is not None:

    class DetectionsCallback(
        _ExternalDepthMixin, EmbodiedAgentsCallbacks.DetectionsCallback
    ):
        """ROS2 Detections Callback Handler to process and transform automatika_agents_interfaces/Detections data"""

        def __init__(
            self,
            input_topic,
            node_name: Optional[str] = None,
            buffer_size: int = 1,
        ) -> None:
            """__init__.

            :param input_topic:
            :param node_name:
            :type node_name: Optional[str]
            :param buffer_size:
            :type buffer_size: Optional[Int], default 10
            :rtype: None
            """
            super().__init__(input_topic, node_name)
            self._detected_boxes: List[Tuple[str, Bbox2D]] = []
            self._img_size: Optional[np.ndarray] = None
            # Initial time of the first detection is used to reset ROS time to
            # zero on the first detection and avoid sending large timestamps to core
            self._initial_time = 0.0
            self._depth_image: Optional[np.ndarray] = None
            self._depth_meta: Optional[Tuple[np.dtype, float]] = None
            self._label: Optional[str] = None
            self._buffer_items: int = 0
            self._max_buffer_size = buffer_size
            self._feature_items: int = 4  # (center_x, center_y, size_x, size_y)
            self._detections_buffer = np.ones((
                buffer_size,
                self._feature_items,
            ))  # num_detections x num_features
            self._depth_detector: Optional[DepthDetector] = None

        def set_depth_detector(self, depth_detector: DepthDetector) -> None:
            """Sets the depth detector to be used with the points of interest

            :param depth_detector: DepthDetector object to be used with the points of interest
            :type depth_detector: DepthDetector
            """
            self._depth_detector = depth_detector

        def __get_img_size(self, msg) -> Optional[np.ndarray]:
            """Get image size from a detection set

            :param detections_set: _description_
            :type detections_set: _type_
            :return: Image size (width, height)
            :rtype: np.ndarray[dtype=np.int32]
            """
            # Get image size
            img_size = None
            if msg.depth.data:
                img_size = np.array(
                    [msg.depth.width, msg.depth.height],
                    dtype=np.int32,
                )
            elif msg.image.data:
                img_size = np.array(
                    [msg.image.width, msg.image.height],
                    dtype=np.int32,
                )
            # TODO: get compressed image size
            if img_size is None:
                get_logger(self.node_name).debug(
                    f"No image is provided with the detections message. Unknown image size can lead to errors! {len(msg.depth.data)} and {len(msg.image.data)} and {len(msg.compressed_image.data)}",
                    once=True,
                )
            return img_size

        def set_buffer_size(self, value: int, clear_old: bool = False) -> None:
            """Resizes detections buffer (while maintaining old buffer items in the resized buffer)

            :param value: New buffer size
            :type value: int
            """
            new_buffer = np.ones((value, self._feature_items))

            if clear_old:
                self._buffer_items = 0
            else:
                if value >= self._max_buffer_size:
                    new_buffer[-self._max_buffer_size :] = self._detections_buffer
                else:
                    new_buffer = self._detections_buffer[-self._max_buffer_size :]
            self._max_buffer_size = value
            self._detections_buffer = new_buffer

        def _process_raw_data(self, msg) -> None:
            """Process new raw detections data and add it to buffer if available"""
            if not msg or not msg.boxes:
                # No detections received -> Reduce buffer items
                self._buffer_items = max(self._buffer_items - 1, 0)
                return
            # Clear old detections
            self._detected_boxes = []

            # Get depth image if available (raw zero-copy view)
            if msg.depth.data:
                self._depth_image, self._depth_meta = _depth_view_and_meta(
                    msg.depth, self._depth_meta
                )

            # Get timestamp for tracking
            timestamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

            # Get image size (Only update if the image is sent with the new detections)
            if self._img_size is None:
                self._img_size = self.__get_img_size(msg)

            got_label = False
            for idx, box in enumerate(msg.boxes):
                label = msg.labels[idx] if idx < len(msg.labels) else ""
                # Add new detection to the buffer
                self._detections_buffer[-1:] = [
                    (box.bottom_right_x + box.top_left_x) / 2,  # center_x
                    (box.top_left_y + box.bottom_right_y) / 2,  # center_y
                    abs(box.bottom_right_x - box.top_left_x),  # size_x
                    abs(box.bottom_right_y - box.top_left_y),  # size_y
                ]
                self._buffer_items = min(self._buffer_items + 1, self._max_buffer_size)
                got_label = True

                box_2d = Bbox2D(
                    top_left_corner=np.array(
                        [box.top_left_x, box.top_left_y], dtype=np.int32
                    ),
                    size=np.array(
                        [
                            abs(box.bottom_right_x - box.top_left_x),
                            abs(box.bottom_right_y - box.top_left_y),
                        ],
                        dtype=np.int32,
                    ),
                    timestamp=timestamp - self._initial_time,
                    label=label,
                )
                if self._img_size is not None:
                    box_2d.set_img_size(self._img_size)
                self._detected_boxes.append((label, box_2d))
            if not got_label:
                self._buffer_items = max(self._buffer_items - 1, 0)
            if self._initial_time == 0.0:
                # Get the initial time of the first detection
                self._initial_time = timestamp

        def callback(self, msg) -> None:
            """
            Topic subscriber callback

            :param msg: Received ros msg
            :type msg: Any
            """
            super().callback(msg)
            self._process_raw_data(msg)

        @property
        def depth_image(self) -> Optional[np.ndarray]:
            """
            Sets the depth image to be used with the detections

            :param depth_image: Depth image in numpy array format
            :type depth_image: Optional[np.ndarray]
            """
            return self._depth_image

        @property
        def depth_scale(self) -> Optional[float]:
            """Raw-depth-to-meters scale from the encoding (see
            depth_conversion_factor), None until a depth image arrives"""
            return self._depth_meta[1] if self._depth_meta else None

        def _get_output_state(
            self, boxes: List[Bbox2D], robot_state: Optional[RobotState]
        ):
            depth = self.depth
            if not self._depth_detector or depth is None or robot_state is None:
                return None
            try:
                return _lift_to_state(self._depth_detector, depth, boxes, robot_state)
            except KeyError:
                return None

        def _get_output(
            self,
            label: Optional[str] = None,
            to_robot_state: bool = False,
            **kwargs,
        ) -> Union[None, List[Bbox2D]]:
            """
            Gets the trackings data
            :returns:   Topic content
            :rtype:     Union[ROSTrackings, np.ndarray, None]
            """
            if not label:
                all_boxes = [box for _, box in self._detected_boxes]
                if to_robot_state:
                    return self._get_output_state(all_boxes, **kwargs)
                return all_boxes
            try:
                self._label = label
                if self._buffer_items <= 0:
                    return None
                else:
                    last_detections = self._detections_buffer[-self._buffer_items :]
                    # Create weights array: [1, 2, ..., n]
                    weights = np.arange(1, self._buffer_items + 1).reshape(-1, 1)
                    # Multiply each row by its weight then divide by the sum
                    average_det = np.sum(last_detections * weights, axis=0) / np.sum(
                        weights
                    )
                    detection_at_index = next(
                        (box for lbl, box in self._detected_boxes if lbl == label),
                        None,
                    )
                    if detection_at_index is None:
                        return None
                    # Buffer rows hold (center, size) -> convert to corner
                    average_box = Bbox2D(
                        top_left_corner=np.array(
                            [
                                average_det[0] - average_det[2] / 2,
                                average_det[1] - average_det[3] / 2,
                            ],
                            dtype=np.int32,
                        ),
                        size=np.array(
                            [
                                average_det[2],
                                average_det[3],
                            ],
                            dtype=np.int32,
                        ),
                        timestamp=detection_at_index.timestamp,  # Gets last timestamp
                        label=label,
                    )
                    average_box.set_img_size(detection_at_index.img_size)
                    if to_robot_state:
                        return self._get_output_state([average_box], **kwargs)
                    return [average_box]
            except KeyError:
                return None

    class PointsOfInterestCallback(
        _ExternalDepthMixin, EmbodiedAgentsCallbacks.PointsOfInterestCallback
    ):
        """ROS2 Points of interest Callback Handler to process and transform automatika_agents_interfaces/PointsOfInterest data"""

        def __init__(
            self,
            input_topic,
            node_name: Optional[str] = None,
        ) -> None:
            """__init__.

            :param input_topic:
            :param node_name:
            :type node_name: Optional[str]
            :param buffer_size:
            :type buffer_size: Optional[Int], default 10
            :rtype: None
            """
            super().__init__(input_topic, node_name)
            self._img_size: Optional[np.ndarray] = None
            self._depth_image: Optional[np.ndarray] = None
            self._depth_detector: Optional[DepthDetector] = None
            self._depth_meta: Optional[Tuple[np.dtype, float]] = None

        def set_depth_detector(self, depth_detector: DepthDetector) -> None:
            """Sets the depth detector to be used with the points of interest

            :param depth_detector: DepthDetector object to be used with the points of interest
            :type depth_detector: DepthDetector
            """
            self._depth_detector = depth_detector

        def callback(self, msg) -> None:
            """
            Topic subscriber callback

            :param msg: Received ros msg
            :type msg: Any
            """
            super().callback(msg)
            # Get depth image if available (raw zero-copy view)
            if msg.depth.data:
                self._depth_image, self._depth_meta = _depth_view_and_meta(
                    msg.depth, self._depth_meta
                )
                if self._img_size is None:
                    # Image size convention is (width, height); shape is (rows, cols)
                    self._img_size = np.array(
                        [self._depth_image.shape[1], self._depth_image.shape[0]],
                        dtype=np.int32,
                    )

        def _get_output(
            self,
            robot_state: Optional[RobotState] = None,
            **_,
        ) -> Optional[RobotState]:
            """
            Gets the trackings data
            :returns:   Topic content
            :rtype:     Union[ROSTrackings, np.ndarray, None]
            """
            depth = self.depth
            if not self._depth_detector or depth is None or robot_state is None:
                return None
            points = []

            for point in self.msg.points:
                points.append(np.array([point.x, point.y], dtype=np.int32))
            try:
                return _lift_to_state(
                    self._depth_detector,
                    depth,
                    PointsOfInterest(points=points, img_size=self._img_size),
                    robot_state,
                )
            except KeyError:
                return None

    class TrackingsCallback(
        _ExternalDepthMixin, EmbodiedAgentsCallbacks.DetectionsCallback
    ):
        """ROS2 Trackings Callback Handler to process and transform automatika_agents_interfaces/Trackings data"""

        def __init__(
            self,
            input_topic,
            node_name: Optional[str] = None,
            buffer_size: int = 10,
        ) -> None:
            """__init__.

            :param input_topic:
            :param node_name:
            :type node_name: Optional[str]
            :param buffer_size:
            :type buffer_size: Optional[Int], default 10
            :rtype: None
            """
            super().__init__(input_topic, node_name)
            self._buffer_items: int = 0
            self._max_buffer_size = buffer_size
            self._feature_items: int = (
                6  # (center_x, center_y, size_x, size_y, vel_x, vel_y)
            )
            self._detections_buffer = np.ones((
                buffer_size,
                self._feature_items,
            ))  # num_detections x num_features
            self._label: Optional[str] = None
            self._id: Optional[int] = None
            self._initial_time = 0.0
            self._depth_image: Optional[np.ndarray] = None
            self._depth_meta: Optional[Tuple[np.dtype, float]] = None
            # Frame id of the camera the buffered trackings came from
            self._img_frame_id: Optional[str] = None

        def callback(self, msg) -> None:
            """
            Topic subscriber callback

            :param msg: Received ros msg
            :type msg: Any
            """
            super().callback(msg)
            self._process_raw_data()

        def _get_output(
            self,
            label: Optional[str] = None,
            idx: Optional[int] = 0,
            **_,
        ) -> Union[Any, Bbox2D, None]:
            """
            Gets the trackings data
            :returns:   Topic content
            :rtype:     Union[ROSTrackings, np.ndarray, None]
            """
            self._label = label or self._label
            self._id = idx

            if not self._buffer_items:
                return None

            last_detections = self._detections_buffer[-self._buffer_items :]

            # Create weights array: [1, 2, ..., n]
            weights = np.arange(1, self._buffer_items + 1).reshape(-1, 1)

            # Multiply each row by its weight then divide by the sum
            average_det = np.sum(last_detections * weights, axis=0) / np.sum(weights)

            timestamp = self.msg.header.stamp.sec + 1e-9 * self.msg.header.stamp.nanosec
            # Buffer rows hold (center, size, vel) -> convert to corner
            box = Bbox2D(
                top_left_corner=np.array(
                    [
                        average_det[0] - average_det[2] / 2,
                        average_det[1] - average_det[3] / 2,
                    ],
                    dtype=np.int32,
                ),
                size=np.array(
                    [
                        average_det[2],
                        average_det[3],
                    ],
                    dtype=np.int32,
                ),
                timestamp=timestamp - self._initial_time,
                label=self._label,
            )
            box.set_vel(
                np.array(
                    [average_det[4], average_det[5], 0.0],
                    dtype=np.float32,
                )
            )
            if self._initial_time == 0:
                # Get the initial time of the first detection
                self._initial_time = timestamp
            return box

        def set_buffer_size(self, value: int, clear_old: bool = False) -> None:
            """Resizes detections buffer (while maintaining old buffer items in the resized buffer)

            :param value: New buffer size
            :type value: int
            """
            new_buffer = np.ones((value, self._feature_items))
            if clear_old:
                self._buffer_items = 0
            else:
                if value >= self._max_buffer_size:
                    new_buffer[-self._max_buffer_size :] = self._detections_buffer
                else:
                    new_buffer = self._detections_buffer[-self._max_buffer_size :]
            self._max_buffer_size = value
            self._detections_buffer = new_buffer

        def set_target(self, label: str, idx: Optional[int] = None):
            """Sets tracked target label and id prior to calling get_output

            :param label: Tracked target label
            :type label: str
            :param idx: Tracked target index, defaults to None
            :type idx: Optional[int], optional
            """
            self._label = label
            self._id = idx

        @property
        def depth_image(self) -> Optional[np.ndarray]:
            """
            Sets the depth image to be used with the detections

            :param depth_image: Depth image in numpy array format
            :type depth_image: Optional[np.ndarray]
            """
            return self._depth_image

        @property
        def depth_scale(self) -> Optional[float]:
            """Raw-depth-to-meters scale from the encoding (see
            depth_conversion_factor), None until a depth image arrives"""
            return self._depth_meta[1] if self._depth_meta else None

        def _process_raw_data(self) -> None:
            """Process new raw trackings data and add it to buffer if available"""
            # Remove a buffer item
            self._detections_buffer = np.roll(self._detections_buffer, -1, axis=0)

            if not self._label or not self.msg:
                # No trackings -> reduce buffer items
                self._buffer_items = max(self._buffer_items - 1, 0)
                return

            # Get depth image if available (raw zero-copy view)
            if self.msg.depth.data:
                self._depth_image, self._depth_meta = _depth_view_and_meta(
                    self.msg.depth, self._depth_meta
                )

            # Get requested item from trackings using id or label
            label_index = None
            id_index = None
            if self._id is not None and self._id in self.msg.ids:
                id_index = self.msg.ids.index(self._id)
            elif self._label and self._label in self.msg.labels:
                label_index = self.msg.labels.index(self._label)

            # If requested label/id not in detections -> return None
            if label_index is None and id_index is None:
                # No detection -> reduce buffer items
                self._buffer_items = max(self._buffer_items - 1, 0)
                return

            id_index = id_index if id_index is not None else label_index

            bbox_2d = self.msg.boxes[id_index]

            # If tracking source camera changes, clear buffers
            if self.msg.image.data:
                frame_id = self.msg.image.header.frame_id
            elif self.msg.compressed_image.data:
                frame_id = self.msg.compressed_image.header.frame_id
            else:
                frame_id = None
            if frame_id is not None and frame_id != self._img_frame_id:
                if self._img_frame_id is not None:
                    get_logger(self.node_name).warning(
                        f"Trackings source camera changed from "
                        f"'{self._img_frame_id}' to '{frame_id}' -> clearing "
                        "buffered detections"
                    )
                    self._detections_buffer = np.ones((
                        self._max_buffer_size,
                        self._feature_items,
                    ))
                    self._buffer_items = 0
                self._img_frame_id = frame_id

            # Add new detection to the buffer
            self._detections_buffer[-1:] = [
                self.msg.centroids[id_index].x,  # center_x
                self.msg.centroids[id_index].y,  # center_y
                abs(bbox_2d.bottom_right_x - bbox_2d.top_left_x),  # size_x
                abs(bbox_2d.bottom_right_y - bbox_2d.top_left_y),  # size_y
                self.msg.estimated_velocities[id_index].x,  # vel_x
                self.msg.estimated_velocities[id_index].y,  # vel_y
            ]
            self._buffer_items = min(self._buffer_items + 1, self._max_buffer_size)

else:

    class DetectionsCallback(GenericCallback):
        def __init__(self, *args, **kwargs) -> None:
            raise ModuleNotFoundError(
                "'automatika_embodied_agents' module is required to use 'Detections' msg type but it is not installed. Install it with `sudo apt install ros-$ROS_DISTRO-automatika-embodied-agents`"
            )

    class TrackingsCallback(GenericCallback):
        def __init__(self, *args, **kwargs) -> None:
            raise ModuleNotFoundError(
                "'automatika_embodied_agents' module is required to use 'Trackings' msg type but it is not installed. Install it with `sudo apt install ros-$ROS_DISTRO-automatika-embodied-agents`"
            )

    class PointsOfInterestCallback(GenericCallback):
        def __init__(self, *args, **kwargs) -> None:
            raise ModuleNotFoundError(
                "'automatika_embodied_agents' module is required to use 'PointsOfInterest' msg type but it is not installed. Install it with `sudo apt install ros-$ROS_DISTRO-automatika-embodied-agents`"
            )
