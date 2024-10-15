# Robot Configuration

A robot can be configured using RobotConfig and RobotFrames Classes by specifying the robot:
- [motion model](#motion-model)
- [geometry](#geometry)
- [control limits](#control-limits)
- and [coordinates frames](#coordinate-frames).

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([1.0, 1.0, 1.0])
)
```

## Motion Model

Kompass supports three types of robot motion models:

- ACKERMANN: Non-holonomic (bicycle model) robots, like vehicles for example. ([more](https://en.wikipedia.org/wiki/Ackermann_steering_geometry) on this motion model)

- DIFFERENTIAL_DRIVE: Two wheeled robots that can perform forward/backward and rotation in place. Turtlebot robots fall in this category. ([more](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) on this motion model)

- OMNI: Omni-motion robots can perform lateral motion as well. Quadrupeds are usually from this category.


## Geometry

The robot geometry can be presented with one of the following 3D geometry:

- BOX: Axis-aligned box with given side lengths.

Parameters: (x, y, z)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([1.0, 1.0, 1.0])
)
```
- CYLINDER: Cylinder with given radius and height along z-axis

Parameters: (rad, lz)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.5, 1.0])
)
```

- SPHERE: Sphere with given radius

Parameters: (rad)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.SPHERE,
    geometry_params=np.array([0.5])
)
```

- ELLIPSOID: Axis-aligned ellipsoid with given radius

Parameters: (x, y, z)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.ELLIPSOID,
    geometry_params=np.array([0.3, 0.2, 0.2])
)
```

- CAPSULE: Capsule with given radius and height along z-axis

Parameters: (rad, lz)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.CAPSULE,
    geometry_params=np.array([0.3, 1.0])
)
```

- CONE: Cone with given radius and height along z-axis

Parameters: (rad, lz)

Example:

```python
import numpy as np
from kompass_core.models import RobotConfig, RobotType, RobotGeometry

robot_config = RobotConfig(
    model_type=RobotType.ACKERMANN,
    geometry_type=RobotGeometry.Type.CONE,
    geometry_params=np.array([0.3, 1.0])
)
```

## Control Limits

It is essential to provide the right control limits for your robot as these limits will drive the computation of the control commands during the navigation. Two classes are provided to set the linear control limits and the angular control limits, as shown in the example below.

For both linear and angular control limits we need to set:

- Maximum velocity (m/s) or (rad/s)
- Maximum acceleration (x/s^2) or (rad/s^2)
- Maximum deceleration (x/s^2) or (rad/s^2)

Additionally, for angular control limits we can set the maximum steering angle (rad)

```python
from kompass_core.models import LinearCtrlLimits, AngularCtrlLimits
import numpy as np

ctrl_vx_limits = LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5)
ctrl_vy_limits = LinearCtrlLimits(max_vel=0.5, max_acc=0.7, max_decel=3.5)
ctrl_omega_limits=AngularCtrlLimits(
        max_vel=1.0, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),

```
:::{tip} Deceleration limit is separated from the acceleration limit to allow the robot to decelerate faster thus ensuring safety.
:::

## Coordinate Frames

KOMPASS currently supports the following coordinate frames, the user is required to configure the names of these frames so KOMPASS can internally lookup the transformations during navigation.

- world: Reference world frame, usually 'map'
- odom: Robot Odometry frame
- robot_base: Frame attached to the robot body. The navigation considers this as the center of the robot geometry.

Sensor frames:
- scan: Lasescan frame
- rgb: RGB camera frame
- depth: Depth camera frame

```python
from kompass.config import RobotFrames

robot_frames = RobotFrames(
    robot_base='base_link',
    odom='odom',
    world='map',
    scan='scan',
    rgb='camera/rgb',
    depth='camera/depth',
)

```
