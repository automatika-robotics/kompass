# Vision Follower (RGB-Depth)

**Depth-aware target tracking with integrated obstacle avoidance.**

The **VisionFollowerRGBD** is a sophisticated 3D visual servoing controller. It combines 2D object detections with depth information to estimate the precise 3D position and velocity of a target.

Unlike the pure [RGB variant](https://www.google.com/search?q=vision_follower_rgb.md), this controller uses a sampling-based planner (based on **[DWA](https://www.google.com/search?q=dwa.md)**) to compute motion. This allows the robot to follow a target while simultaneously navigating around obstacles, making it the ideal choice for "Follow Me" applications in complex environments.

## How it Works

The controller utilizes a high-performance C++ core (**VisionDWA**) to execute the following pipeline:

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`view_in_ar` 1. 3D Projection</span> - **Depth Fusion.** Projects 2D bounding boxes into 3D space using the depth image and camera intrinsics.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`timeline` 2. DWA Sampling</span> - **Trajectory Rollout.** Generates candidate velocity trajectories based on the robot's current speed and acceleration limits.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`security` 3. Collision Checking</span> - **Safety First.** Evaluates each trajectory against active sensor data (LaserScan/PointCloud) to ensure the robot doesn't hit obstacles while following.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`my_location` 4. Goal Scoring</span> - **Relative Pose.** Selects the trajectory that best maintains the configured **Target Distance** and **Target Orientation**.

## Key Features

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`social_distance` Relative Positioning</span> - Maintain a specific distance and bearing relative to the target (e.g., follow from  away at a  angle).
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`directions_run` Velocity Tracking</span> - Capable of estimating target velocity to provide smoother, more predictive following.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`manage_search` Recovery Behaviors</span> - Includes configurable **Wait** and **Search** (rotating in place) logic for when the target is temporarily occluded or leaves the field of view.

## Supported Inputs

This controller requires synchronized vision and spatial data.

* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`settings_overscan` Detections</span> - 2D bounding boxes ([Detections2D](../types.md/#supported-ros2-messages), [Trackings2D](../types.md/#supported-ros2-messages)).
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`photo_camera` Depth Image Information</span> - Aligned depth image info for 3D coordinate estimation.
* <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radar` Obstacle Data</span> - `LaserScan`, `PointCloud`, or `LocalMap` for active avoidance.

## Configuration Parameters

The RGB-D follower inherits all parameters from [DWA](./index.md) and adds vision-specific settings.

```{list-table}
:widths: 20 15 15 50
:header-rows: 1
* - Name
  - Type
  - Default
  - Description
* - **target_distance**
  - `float`
  - `None`
  - The desired distance (m) to maintain from the target.
* - **target_orientation**
  - `float`
  - `0.0`
  - The desired bearing angle (rad) relative to the target.
* - **prediction_horizon**
  - `int`
  - `10`
  - Number of future steps to project for collision checking.
* - **target_search_timeout**
  - `float`
  - `30.0`
  - Max time to search for a lost target before giving up.
* - **depth_conversion_factor**
  - `float`
  - `1e-3`
  - Factor to convert raw depth values to meters (e.g., $0.001$ for mm).
* - **camera_position_to_robot**
  - `np.array`
  - `[0,0,0]`
  - 3D translation vector $(x, y, z)$ from camera to robot base.

```

## Usage Example

```python
from kompass.control import VisionRGBDFollowerConfig

config = VisionRGBDFollowerConfig(
    target_distance=1.5,
    target_orientation=0.0,
    enable_search=True,
    max_linear_samples=15
)

```

For full system integration including ML model deployment, check the Vision-Depth Tracking Tutorial:

:::{button-link} ../../tutorials/vision_tracking_depth.html
:color: primary
:ref-type: doc
:outline:
Vision-Depth Tracking Tutorial →
:::

