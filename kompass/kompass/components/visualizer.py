from .component import Component, ComponentConfig
from ..topic import Topic
from ..utils import visualize_tracking
from typing import Optional
from attrs import define, field
from ..callbacks import TrackingsCallback


@define
class VisualizerConfig(ComponentConfig):
    tracked_label: Optional[str] = field(default=None)


class Visualizer(Component):
    def __init__(
        self,
        component_name: str,
        input_topic: Optional[Topic] = None,
        label: Optional[str] = None,
        config=None,
        **_
    ):
        if not input_topic:
            input_topic = Topic(name="trackings", msg_type="Trackings")

        config = VisualizerConfig(tracked_label=label) if label else config

        super().__init__(
            component_name,
            inputs={'trackings': input_topic},
            config=config,
        )

    def attach_callbacks(self):
        for callback in self.callbacks.values():
            if isinstance(callback, TrackingsCallback):
                self.get_logger().info("Adding cv2 visualization to trackings callback")
                callback.on_callback_execute(self._visualize_tracking)
            else:
                self.get_logger().info("Adding cv2 visualization to detections callback")
                callback.on_callback_execute(self._visualize_detections)

    def _visualize_tracking(self, msg, **_):
        # Get tracked label
        if not self.config.tracked_label:
            return
        id_index = None
        values = msg.trackings
        for tracking in values:
            if self.config.tracked_label in tracking.labels:
                id_index = tracking.labels.index(self.config.tracked_label)
                break
        if id_index is not None:
            self.get_logger().info(
                f"Visualizing data for {type(msg)} and label: {self.config.tracked_label}"
            )
            visualize_tracking(values[id_index], name="Tracking")
            return

    def _visualize_detections(self, msg, **_):
        # Get tracked label
        if not self.config.tracked_label:
            return
        id_index = None
        values = msg.detections
        for tracking in values:
            if self.config.tracked_label in tracking.labels:
                id_index = tracking.labels.index(self.config.tracked_label)
                break
        if id_index is not None:
            self.get_logger().info(
                f"Visualizing data for {type(msg)} and label: {self.config.tracked_label}"
            )
            visualize_tracking(values[id_index], centroid=False, name="Detection")
            return
