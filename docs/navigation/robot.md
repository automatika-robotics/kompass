# Robot Configuration

Before Kompass can drive your robot, it needs to understand its physical constraints. You define this "Digital Twin" using the `RobotConfig` object, which aggregates the Motion Model, Geometry, and Control Limits.

```python
import numpy as np
from kompass.robot import RobotConfig, RobotType, RobotGeometry

# Example: Defining a simple box-shaped Ackermann robot
robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([1.0, 1.0, 1.0]) # x, y, z
)

```

## Motion Models

Kompass supports three distinct kinematic models. Choose the one that matches your robot's drivetrain.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`directions_car;1.5em;sd-text-primary` Ackermann</span> - **Car-Like Vehicles**
Non-holonomic constraints (bicycle model). The robot has a limited steering angle and cannot rotate in place.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`sync;1.5em;sd-text-primary` Differential</span> - **Two-Wheeled Robots**
Robots like the Turtlebot. Capable of forward/backward motion and zero-radius rotation (spinning in place).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`open_with;1.5em;sd-text-primary` Omni</span> - **Holonomic Robots**
Robots like Mecanum-wheel platforms or Quadrupeds. Capable of instantaneous motion in any direction (x, y) and rotation.


## Robot Geometry

The geometry defines the collision volume of the robot. This is used by the local planner for obstacle avoidance.

The `geometry_params` argument expects a **NumPy array** containing specific dimensions based on the selected type:

| Type | Parameters (np.array) | Description |
| ---  | --- | --- |
| **BOX**  | `[length, width, height]` | Axis-aligned box. |
| **CYLINDER**  | `[radius, length_z]` | Vertical cylinder. |
| **SPHERE**  | `[radius]` | Perfect sphere. |
| **ELLIPSOID**  | `[axis_x, axis_y, axis_z]` | Axis-aligned ellipsoid. |
| **CAPSULE**  | `[radius, length_z]` | Cylinder with hemispherical ends. |
| **CONE** | `[radius, length_z]` | Vertical cone. |

**Configuration Example:**

```python
# A cylinder robot (Radius=0.5m, Height=1.0m)
cylinder_robot_config = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.5, 1.0])
)

# A box robot (Length=0.5, Width=0.5, Height=1.0m)
box_robot_config = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([0.5, 0.5, 1.0])
)

```


## Control Limits

Safety is paramount. You must explicitly define the kinematic limits for linear and angular velocities.

Kompass separates **Acceleration** limits from **Deceleration** limits. This allows you to configure a "gentle" acceleration for smooth motion, but a "hard" deceleration for emergency braking.

```python
from kompass.robot import LinearCtrlLimits, AngularCtrlLimits, RobotConfig, RobotType, RobotGeometry
import numpy as np

# 1. Linear Limits (Forward/Backward)
# Max Speed: 1.0 m/s
# Acceleration: 1.5 m/s^2 (Smooth start)
# Deceleration: 2.5 m/s^2 (Fast stop)
ctrl_vx = LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5)

# 2. Angular Limits (Rotation)
# Max Steer is only used for Ackermann robots
ctrl_omega = AngularCtrlLimits(
    max_vel=1.0,
    max_acc=2.0,
    max_decel=2.0,
    max_steer=np.pi / 3
)

 # Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=ctrl_vx,
    ctrl_omega_limits=ctrl_omega,
)

```

:::{tip}
For Ackermann robots, `ctrl_omega_limits.max_steer` defines the maximum physical steering angle of the wheels in radians.
:::


## Coordinate Frames

Kompass needs to know the names of your TF frames to perform lookups. You configure this using the `RobotFrames` object.

The components will automatically subscribe to `/tf` and `/tf_static` to track these frames.

```python
from kompass.config import RobotFrames

frames = RobotFrames(
    world='map',          # The fixed global reference frame
    odom='odom',          # The drift-prone odometry frame
    robot_base='base_link', # The center of the robot
    scan='scan',          # Lidar frame
    rgb='camera/rgb',     # RGB Camera frame
    depth='camera/depth'  # Depth Camera frame
)

```

| Frame | Description |
| --- | --- |
| **world** | The global reference for path planning (usually `map`). |
| **odom** | The continuous reference for local control loops. |
| **robot_base** | The physical center of the robot. All geometry is relative to this. |
| **Sensors** | `scan`, `rgb`, `depth` frames are used to transform sensor data into the robot's frame. |
