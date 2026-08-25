"""Vision target tracking helper for the Controller component.

Encapsulates the ``Vision Follower`` lifecycle (setup, initial-target
acquisition, action loop, termination) and the vision-only state
(``_vision_controller``, ``_tracked_center``, latest detections, depth image
and metadata) that previously lived directly on the Controller.

The helper holds a back-reference to its owning ROS ``Controller`` component
(``self._component``) and reads shared infrastructure (TF listeners,
callbacks, robot model, publishers, logger) through it.

The helper never mutates Component state except via the documented methods
``_publish`` and ``_stop_robot``, plus ``config._frame_mode`` which the
helper sets at setup time based on TF availability.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

import numpy as np

from ros_sugar.io import CameraIntrinsics

from kompass_core.control import (
    ControlClasses,
    ControlConfigClasses,
    ControllersID,
    ControllerType,
)
from kompass_core.datatypes import Bbox2D
from kompass_interfaces.action import TrackVisionTarget

from ._modes import ControllerMode, FrameMode
from .defaults import TopicsKeys

if TYPE_CHECKING:
    from .controller import Controller


class VisionFollower:
    """Owns the vision tracking lifecycle for a Controller component in vision mode."""

    def __init__(self, component: "Controller") -> None:
        self._component = component
        self._vision_controller: Optional[ControllerType] = None
        self._tracked_center: Optional[np.ndarray] = None
        self.vision_detections: Optional[Bbox2D] = None
        self.depth_image: Optional[np.ndarray] = None
        self.depth_image_info: Optional[CameraIntrinsics] = None

    # ------------------------------------------------------------------
    # Public API consumed by the Controller component
    # ------------------------------------------------------------------

    @property
    def is_tracking(self) -> bool:
        return self._tracked_center is not None

    def optimal_path(self):
        """Return the kompass_core optimal path for the current vision controller, if any."""
        if not self._vision_controller:
            return None
        return self._vision_controller.optimal_path()

    @property
    def _depth_tf_listener(self):
        """Transform listener from the depth camera frame to the robot body.

        The camera frame is read from the camera info messages, so this is
        None until the first one arrives.
        """
        cmp = self._component
        return cmp.input_tf_listener(
            TopicsKeys.DEPTH_CAM_INFO, cmp.config.frames.robot_base, static_tf=True
        )

    def setup(self) -> bool:
        """Build the core vision controller. Stores it on success.

        Decides ``_frame_mode`` from TF availability: if the odom→world TF
        (or matching frames) is available we operate in GLOBAL, otherwise
        we fall back to LOCAL (robot-relative).
        """
        cmp = self._component
        timeout = 0.0
        # Re-acquire the intrinsics per setup
        self.depth_image_info = None
        # Get the depth image transform if the input is provided
        if cmp.in_topic_name(TopicsKeys.DEPTH_CAM_INFO):
            while (
                not (
                    (depth_tf := self._depth_tf_listener)
                    and depth_tf.got_transform
                    and self.depth_image_info
                )
                and timeout < cmp.config.topic_subscription_timeout
            ):
                self._update_inputs()
                cmp.get_logger().info(
                    "Waiting to get Depth camera to body TF to initialize Vision Follower...",
                    once=True,
                )
                time.sleep(1 / cmp.config.loop_rate)
                timeout += 1 / cmp.config.loop_rate

        if timeout >= cmp.config.topic_subscription_timeout:
            return False

        cmp.get_logger().info(
            "Got Depth camera to body TF -> Setting up Vision Follower controller"
        )

        # NOTE: If the location->world TF is available the follower tracks in GLOBAL,
        # otherwise it falls back to LOCAL (robot-relative). The mode is reset to
        # GLOBAL before probing because _update_state(block=True) only waits for
        # the TF in GLOBAL mode, and setup() can re-enter while the mode is still
        # LOCAL. A robot already localized in the world frame resolves to the identity
        # transform, so it needs no special case here.
        cmp.config._frame_mode = FrameMode.GLOBAL
        cmp._update_state(block=True)
        has_tf = cmp.odom_tf_listener is not None and cmp.odom_tf_listener.got_transform
        cmp.config._frame_mode = FrameMode.GLOBAL if has_tf else FrameMode.LOCAL
        use_local = cmp.config._frame_mode == FrameMode.LOCAL

        if use_local:
            cmp.get_logger().info(
                "No global localization available — vision follower will "
                "operate in LOCAL (robot-relative) frame"
            )
        else:
            cmp.get_logger().info(
                "Global localization available — vision follower will "
                "operate in GLOBAL frame with velocity tracking"
            )

        depth_tf = self._depth_tf_listener
        config = ControlConfigClasses[cmp.algorithm](
            control_time_step=cmp.config.control_time_step,
            camera_position_to_robot=depth_tf.translation if depth_tf else None,
            camera_rotation_to_robot=depth_tf.rotation if depth_tf else None,
            _use_local_coordinates=use_local,
        )

        _controller_config = cmp._configure_algorithm(config)

        # Apply the (possibly user-overridden) buffer size to the detections callback
        detections_callback = cmp.get_callback(TopicsKeys.VISION_DETECTIONS)
        if detections_callback:
            detections_callback.set_buffer_size(_controller_config.buffer_size)

        # re-read the file and overwrite any programmatic overrides.
        self._vision_controller = ControlClasses[cmp.algorithm](
            robot=cmp._robot,
            ctrl_limits=cmp._robot_ctr_limits,
            config=_controller_config,
            camera_focal_length=np.array([
                self.depth_image_info.fx,
                self.depth_image_info.fy,
            ])
            if self.depth_image_info
            else None,
            camera_principal_point=np.array([
                self.depth_image_info.cx,
                self.depth_image_info.cy,
            ])
            if self.depth_image_info
            else None,
        )

        cmp.get_logger().info(
            f"Vision Controller '{cmp.algorithm}' is initialized and ready to use"
        )
        return True

    def _refresh_frame_mode(self) -> bool:
        """Upgrade a LOCAL-frame controller to GLOBAL once localization exists.

        Controller built before localization came up (LOCAL) must be rebuilt to
        track in the world frame. Returns False only if the rebuild was needed and failed.
        """
        cmp = self._component
        if cmp.config._frame_mode != FrameMode.LOCAL:
            return True
        if not (cmp.odom_tf_listener and cmp.odom_tf_listener.got_transform):
            return True
        cmp.get_logger().info(
            "Global localization became available -> rebuilding vision "
            "follower to track in GLOBAL frame"
        )
        return self.setup()

    def execute_action(self, goal_handle) -> TrackVisionTarget.Result:
        """Run the vision tracking action to completion."""
        cmp = self._component
        cmp._reached_end = False
        request_msg = goal_handle.request
        feedback_msg = TrackVisionTarget.Feedback()
        result = TrackVisionTarget.Result()
        start_time = time.time()

        if not self._vision_controller and not self.setup():
            cmp.get_logger().error("Could not initialize controller -> ABORTING ACTION")
            return self._terminate_action(goal_handle, result, start_time, "abort")

        if not self._refresh_frame_mode():
            cmp.get_logger().error(
                "Could not rebuild controller for GLOBAL frame -> ABORTING ACTION"
            )
            return self._terminate_action(goal_handle, result, start_time, "abort")

        if not self._acquire_initial_target(
            request_msg.label,
            request_msg.pose_x,
            request_msg.pose_y,
        ):
            cmp.get_logger().error("No Target found on image -> ABORTING ACTION")
            return self._terminate_action(goal_handle, result, start_time, "abort")

        self._tracked_center = np.array([request_msg.pose_x, request_msg.pose_y])

        while (
            cmp.config._mode == ControllerMode.VISION_FOLLOWER and not cmp._reached_end
        ):
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                cmp.get_logger().info("Vision Following Action Canceled!")
                return self._terminate_action(goal_handle, result, start_time, "cancel")

            # Refresh inputs non-blocking; follower tolerates missing frames via
            # its internal target_wait_timeout / enable_search.
            self._update_inputs()
            cmp._update_state(block=False)

            found_ctrl = self._vision_controller.loop_step(
                detections_2d=self.vision_detections or [],
                current_state=cmp.robot_state,
                depth_image=self.depth_image,
            )

            if not found_ctrl:
                cmp.get_logger().warning(
                    "Vision follower lost the target -> Aborting action"
                )
                return self._terminate_action(goal_handle, result, start_time, "abort")

            feedback_msg.distance_error = float(self._vision_controller.dist_error)
            feedback_msg.orientation_error = float(
                self._vision_controller.orientation_error
            )
            goal_handle.publish_feedback(feedback_msg)

            cmp.get_logger().debug(
                f"Following tracked target with control: {self._vision_controller.linear_x_control}, {self._vision_controller.angular_control}"
            )
            cmp._publish(
                self._vision_controller.linear_x_control,
                self._vision_controller.linear_y_control,
                self._vision_controller.angular_control,
            )

            time.sleep(1 / cmp.config.loop_rate)

        cmp.get_logger().warning("Ending tracking action")
        return self._terminate_action(goal_handle, result, start_time, "succeed")

    def request_stop(self) -> bool:
        """Request termination of an ongoing vision tracking action.

        Returns True on success (or if no action is ongoing), False if the
        action server fails to acknowledge within ten loop ticks.
        """
        cmp = self._component
        if (
            cmp.config._mode == ControllerMode.PATH_FOLLOWER
            or self._tracked_center is None
        ):
            cmp.get_logger().warning("No ongoing vision tracking")
            return True

        cmp.get_logger().warning("Ending Vision Tracking Action for target")

        # Set reached_end so the action server can end the loop and reset.
        cmp._reached_end = True

        for _ in range(10):
            time.sleep(1 / cmp.config.loop_rate)
            if not cmp._reached_end:
                self._tracked_center = None
                return True

        return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_inputs(self) -> None:
        """Refresh detections and depth metadata from input callbacks."""
        cmp = self._component
        vision_callback = cmp.get_callback(TopicsKeys.VISION_DETECTIONS)
        self.vision_detections = (
            vision_callback.get_output(clear_last=True) if vision_callback else None
        )
        self.depth_image = vision_callback.depth_image if vision_callback else None
        # Intrinsics are only consumed at setup(). Stop reading it once acquired
        if self.depth_image_info is None:
            depth_img_info_callback = cmp.get_callback(TopicsKeys.DEPTH_CAM_INFO)
            self.depth_image_info = (
                depth_img_info_callback.get_output()
                if depth_img_info_callback
                else None
            )

    def _acquire_initial_target(self, label: str, pose_x: int, pose_y: int) -> bool:
        """Set the initial target on the kompass_core controller.

        Returns True if the controller accepted the initial target.
        """
        cmp = self._component

        # In LOCAL frame mode robot_state is intentionally None — only wait
        # for it when we're operating in GLOBAL.
        needs_state = cmp.config._frame_mode == FrameMode.GLOBAL
        if label != "":
            target_2d = None
            vision_callback = cmp.get_callback(TopicsKeys.VISION_DETECTIONS)
            timeout = 0.0
            while not target_2d and timeout < cmp.config.topic_subscription_timeout:
                self._update_inputs()
                target_2d = (
                    vision_callback.get_output(label=label) if vision_callback else None
                )
                cmp.get_logger().info(
                    f"Waiting to get target {label} from vision detections...",
                    once=True,
                )
                timeout += 1 / cmp.config.loop_rate
                time.sleep(1 / cmp.config.loop_rate)
            if not target_2d:
                cmp.get_logger().error(
                    f"Could not find target {label} in vision detections"
                )
                return False
            timeout = 0.0
            while (
                needs_state and not cmp.robot_state
            ) and timeout < cmp.config.topic_subscription_timeout:
                cmp._update_state(block=False)
                timeout += 1 / cmp.config.loop_rate
                time.sleep(1 / cmp.config.loop_rate)
            if needs_state and not cmp.robot_state:
                cmp.get_logger().error(
                    f"Could not get robot state required for Global mode after {cmp.config.topic_subscription_timeout} seconds"
                )
                return False
            cmp.get_logger().info(
                f"Got initial target {label}, setting to controller..."
            )
            found_target = self._vision_controller.set_initial_tracking_2d_target(
                target_box=target_2d[0],
                current_state=cmp.robot_state,
                aligned_depth_image=self.depth_image,
            )
        else:
            if cmp.algorithm == ControllersID.VISION_IMG:
                cmp.get_logger().error(
                    "Cannot use Vision RGB Follower without providing a target label"
                )
                cmp.health_status.set_fail_algorithm(
                    algorithm_names=[cmp.algorithm.value]
                )
                return False
            timeout = 0.0
            while (
                self.depth_image is None or self.vision_detections is None
            ) and timeout < cmp.config.topic_subscription_timeout:
                self._update_inputs()
                timeout += 1 / cmp.config.loop_rate
                time.sleep(1 / cmp.config.loop_rate)
            if self.depth_image is None or self.vision_detections is None:
                cmp.get_logger().error(
                    f"Could not get initial vision detections to setup the vision follower controller after {cmp.config.topic_subscription_timeout} seconds"
                )
                return False

            # If no label is provided, use the provided pixel coordinates (Only works with depth)
            found_target = self._vision_controller.set_initial_tracking_image(
                cmp.robot_state,
                pose_x,
                pose_y,
                self.vision_detections,
                self.depth_image,
            )
        return found_target

    def _terminate_action(
        self,
        goal_handle,
        result: TrackVisionTarget.Result,
        start_time: float,
        status: str,
    ) -> TrackVisionTarget.Result:
        """Finalize the action (succeed/abort/cancel), stop the robot, reset state."""
        cmp = self._component
        result.tracked_duration = time.time() - start_time
        result.success = status == "succeed"
        with cmp._main_goal_lock:
            if status == "cancel" and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif status == "succeed":
                goal_handle.succeed()
            elif goal_handle.is_active:
                goal_handle.abort()
        cmp._reached_end = True
        self._tracked_center = None
        self.vision_detections = None
        cmp._stop_robot()
        return result
