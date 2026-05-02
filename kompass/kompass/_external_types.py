from typing import Optional, Dict, List, Tuple, Union, Any
import numpy as np
from rclpy.logging import get_logger

from kompass_core.datatypes import Bbox2D, PointsOfInterest
from kompass_core.vision import DepthDetector
from kompass_core.models import RobotState
from .ros import GenericCallback
from .utils import image_pre_processing, process_encoding

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


# EmbodiedAgents Conditional Callback Classes
if EmbodiedAgentsCallbacks is not None:

    class DetectionsCallback(EmbodiedAgentsCallbacks.DetectionsCallback):
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
            # Initial time of the first detection is used to reset ROS time to zero on the first detection and avoid sending large timestamps to core
            self._initial_time = 0.0
            self._depth_image: Optional[np.ndarray] = None
            self._depth_encoding: Optional[Dict] = None
            self._label: Optional[str] = None
            self._buffer_items: int = 0
            self._max_buffer_size = buffer_size
            self._feature_items: int = (
                4  # (top_left_corner_x, top_left_corner_y, size_x, size_y)
            )
            self._detections_buffer = np.ones((
                buffer_size,
                self._feature_items,
            ))  # num_detections x num_features
            self._depth_detector: Optional[DepthDetector] = None

        @classmethod
        def prepare_depth_for_cpp(cls, image_array, orig_dtype, channels) -> np.ndarray:
            """
            Converts a dynamic ROS Image message into a uint16,
            Fortran-contiguous, 2D numpy array for C++ bindings.
            """
            if channels > 1:
                get_logger("Callback").error(
                    f"Expects a 2D depth image, but received a "
                    f"{channels}-channel image."
                )

            # Conserve data by handling floats vs. integers
            if np.issubdtype(orig_dtype, np.floating):
                # Floats (32FC1, 64FC1) are for values in meters.
                # Scrub invalid data and scale to millimeters.
                clean_depth = np.nan_to_num(
                    image_array, nan=0.0, posinf=0.0, neginf=0.0
                )
                depth_scaled = clean_depth * 1000.0
            else:
                # Integers (16UC1, 8UC1) are typically already in millimeters or raw units.
                # If it's a signed integer (16SC1), clip negative values to 0 to avoid wrap-around
                # when casting to unsigned uint16.
                depth_scaled = np.clip(image_array, a_min=0, a_max=None)

            # Cast to uint16 and force Fortran memory layout
            depth_cpp_ready = np.require(
                depth_scaled, dtype=np.uint16, requirements=["F"]
            )

            return depth_cpp_ready

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

            # Get depth image if available
            if msg.depth.data:
                if not self._depth_encoding:
                    self._depth_encoding = process_encoding(msg.depth.encoding)
                image_array = image_pre_processing(msg.depth, *self._depth_encoding)
                self._depth_image = self.prepare_depth_for_cpp(
                    image_array, *self._depth_encoding
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

        def _get_output_state(
            self, boxes: List[Bbox2D], robot_state: Optional[RobotState]
        ):
            if (
                not self._depth_detector
                or self.depth_image is None
                or robot_state is None
            ):
                return None
            try:
                box_3d = self._depth_detector.compute_3d_detections(
                    depth_img=self._depth_image,
                    input=boxes,
                    robot_x=robot_state.x,
                    robot_y=robot_state.y,
                    robot_yaw=robot_state.yaw,
                    robot_speed=robot_state.speed,
                )
                points_center = box_3d[0].center
                return RobotState(x=points_center[0], y=points_center[1])
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
                    average_box = Bbox2D(
                        top_left_corner=np.array(
                            [average_det[0], average_det[1]], dtype=np.int32
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
                        return self._get_output_state([average_box])
                    return [average_box]
            except KeyError:
                return None

    class PointsOfInterestCallback(EmbodiedAgentsCallbacks.PointsOfInterestCallback):
        """ROS2 Detections Callback Handler to process and transform automatika_agents_interfaces/Detections data"""

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
            self._depth_encoding: Optional[Dict] = None

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
            # Get depth image if available
            if msg.depth.data:
                if not self._depth_encoding:
                    self._depth_encoding = process_encoding(msg.depth.encoding)
                image_array = image_pre_processing(msg.depth, *self._depth_encoding)
                self._depth_image = DetectionsCallback.prepare_depth_for_cpp(
                    image_array, *self._depth_encoding
                )
                if not self._img_size:
                    self._img_size = np.array(
                        self._depth_image.shape[:2], dtype=np.int32
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
            if (
                not self._depth_detector
                or self._depth_image is None
                or robot_state is None
            ):
                return None
            points = []

            for point in self.msg.points:
                points.append(np.array([point.x, point.y], dtype=np.int32))
            try:
                box_3d = self._depth_detector.compute_3d_detections(
                    depth_img=self._depth_image,
                    input=PointsOfInterest(
                        points=points,
                        img_size=self._img_size,
                    ),
                    robot_x=robot_state.x,
                    robot_y=robot_state.y,
                    robot_yaw=robot_state.yaw,
                    robot_speed=robot_state.speed,
                )
                points_center = box_3d[0].center
                return RobotState(x=points_center[0], y=points_center[1])
            except KeyError:
                return None

    class TrackingsCallback(EmbodiedAgentsCallbacks.DetectionsCallback):
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
            box = Bbox2D(
                top_left_corner=np.array(
                    [
                        average_det[0],
                        average_det[1],
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

        def _process_raw_data(self) -> None:
            """Process new raw trackings data and add it to buffer if available"""
            # Remove a buffer item
            self._detections_buffer = np.roll(self._detections_buffer, -1, axis=0)

            if not self._label or not self.msg:
                # No trackings -> reduce buffer items
                self._buffer_items = max(self._buffer_items - 1, 0)
                return

            # Get depth image if available
            if self.msg.depth.data:
                if not self._depth_encoding:
                    self._depth_encoding = process_encoding(self.msg.depth.encoding)
                self._depth_image = image_pre_processing(
                    self.msg.depth, *self._depth_encoding
                )

            # Get requested item from trackings using id or label
            label_index = None
            id_index = None
            # for tracking_id, tracking_label in zip(self.msg.ids, self.msg.labels):
            if self._id and self._id in self.msg.ids:
                id_index = self.msg.ids.index(id)
            elif self._label and self._label in self.msg.labels:
                label_index = self.msg.labels.index(self._label)

            # If requested label/id not in detections -> return None
            if label_index is None and id_index is None:
                # No detection -> reduce buffer items
                self._buffer_items = max(self._buffer_items - 1, 0)
                return

            id_index = id_index or label_index

            bbox_2d = self.msg.boxes[id_index]

            # Update the source image data if available
            if self.msg.image.data:
                if (
                    not self._img_metadata
                    or self.msg.image.header.frame_id != self._img_metadata.frame_id
                ):
                    # If a new image meta data is detected -> clear the buffer (the detection is coming from a new camera -> clear old camera data)
                    self._detections_buffer = np.ones((
                        self._max_buffer_size,
                        self._feature_items,
                    ))
                    self._buffer_items = 0

            elif self.msg.compressed_image.data:
                if (
                    not self._img_metadata
                    or self.msg.compressed_image.header.frame_id
                    != self._img_metadata.frame_id
                ):
                    self._detections_buffer = np.ones((
                        self._max_buffer_size,
                        self._feature_items,
                    ))
                    self._buffer_items = 0

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
