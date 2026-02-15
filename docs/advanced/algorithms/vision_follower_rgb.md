# Vision Follower (RGB)

**2D Visual Servoing for target centering.**

The **VisionFollowerRGB** is a reactive controller designed to keep a visual target (like a person or another robot) centered within the camera frame. Unlike the [RGB-D variant](./vision_follower_rgbd.md), this controller operates purely on 2D image coordinates, making it compatible with any standard monocular camera.

It calculates velocity commands based on the **relative shift** and **apparent size** of a 2D bounding box.

## How it Works

The controller uses a proportional control law to minimize the error between the target's current position in the image and the desired center point.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`center_focus_weak` Horizontal Centering</span> - **Rotation.** The robot rotates to minimize the horizontal offset (-axis) of the target bounding box relative to the image center.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`aspect_ratio` Scale Maintenance</span> - **Linear Velocity.** The robot moves forward or backward to maintain a consistent bounding box size, effectively keeping a fixed relative distance without explicit depth data.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`manage_search` Target Recovery</span> - **Search Behavior.** If the target is lost, the controller can initiate a search pattern (rotation in place) to re-acquire the target bounding box.

## Supported Inputs

This controller requires 2D detection or tracking data.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`settings_overscan` Detections / Trackings</span> - Must provide [Detections2D](../types.md/#supported-ros2-messages), [Trackings2D](../types.md/#supported-ros2-messages) (contains the original image info).

:::{admonition} Data Synchronization
:class: note
The `Controller` does not subscribe directly to raw images. It expects the detection metadata (bounding boxes) to be provided by an external vision pipeline.
:::

## Configuration Parameters

```{list-table}
:widths: 20 15 15 50
:header-rows: 1
* - Name
  - Type
  - Default
  - Description
* - **rotation_gain**
  - `float`
  - `1.0`
  - Proportional gain for angular control (centering the target).
* - **speed_gain**
  - `float`
  - `0.7`
  - Proportional gain for linear speed (maintaining distance).
* - **tolerance**
  - `float`
  - `0.1`
  - Error margin for tracking before commands are issued.
* - **target_search_timeout**
  - `float`
  - `30.0`
  - Maximum duration (seconds) to perform search before timing out.
* - **enable_search**
  - `bool`
  - `True`
  - Whether to rotate the robot to find a target if it exits the FOV.
* - **min_vel**
  - `float`
  - `0.1`
  - Minimum linear velocity allowed during following.

```

## Usage Example

```python
import numpy as np
from kompass.control import VisionRGBFollowerConfig

# Configure the algorithm
config = VisionRGBFollowerConfig(
    rotation_gain=0.9,
    speed_gain=0.8,
    enable_search=True
)
```

For full system integration including ML model deployment, check the Vision Tracking Tutorial:

:::{button-link} ../../tutorials/vision_tracking.html
:color: primary
:ref-type: doc
:outline:
Vision Tracking Tutorial →
:::

