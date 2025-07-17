"""Configuration classes for a Component and a robot in Kompass"""

import math
from typing import Union

import numpy as np
import ros_sugar.config.base_validators as BaseValidators
from ros_sugar.config.base_config import (
    _convert_logging_severity_to_str,
    LoggingSeverity,
)
from attrs import Factory, define, field
from kompass_core.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)
from ros_sugar.config import (
    BaseAttrs,
    BaseComponentConfig,
    BaseConfig,
    ComponentRunType,
)

__all__ = [
    "BaseAttrs",
    "BaseValidators",
    "BaseConfig",
    "ComponentRunType",
    "RobotFrames",
    "RobotConfig",
    "BaseComponentConfig",
]


@define(kw_only=True)
class RobotFrames(BaseAttrs):
    """
    Class for robot coordinate frames configuration
    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Frame
      - Default
      - Description

    * - **world**
      - "map"
      - Reference world frame

    * - **robot_base**
      - "base_link"
      - Robot base link frame

    * - **odom**
      - "odom"
      - Robot odometry frame

    * - **scan**
      - "base_link"
      - LaseScan sensor base frame

    * - **rgb**
      - "camera_link"
      - RGB camera base frame

    * - **depth**
      - "camera_depth_link"
      - Depth camera base frame

    ```
    """

    robot_base: str = field(default="base_link")
    odom: str = field(default="odom")
    world: str = field(default="map")
    scan: str = field(default="base_link")
    rgb: str = field(default="camera_link")
    depth: str = field(default="camera_depth_link")
    point_cloud: str = field(default="point_cloud_link")


@define(kw_only=True)
class RobotConfig(BaseAttrs):
    """
    Class for robot configuration (type, model, limitations, etc.)

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **model_type**
      - [RobotType](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L1095) | str, `ACKERMANN`
      - Robot motion model type

    * - **geometry_type**
      - [RobotGeometry.Type](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L643) | str, `CYLINDER`
      - Robot 3D geometry shape type

    * - **geometry_type**
      - `np.ndarray`, `[0.2, 1.0]`
      - Robot 3D geometry shape parameters

    * - **ctrl_vx_limits**
      - [LinearCtrlLimits](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L1153)
      - Forward linear velocity (x-direction) control limits parameters

    * - **ctrl_omega_limits**
      - [AngularCtrlLimits](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L1168)
      - Angular velocity control limits parameters

    * - **ctrl_vy_limits**
      - [LinearCtrlLimits](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_core/models.py#L1153)
      - Lateral linear velocity (y-direction) control limits parameters

    ```
    """

    # Robot Motion Model Type
    model_type: Union[str, RobotType] = field(
        default=RobotType.ACKERMANN,
        converter=lambda value: RobotType.to_str(value),
        validator=BaseValidators.in_(RobotType.values()),
    )

    # Geometry
    geometry_type: Union[str, RobotGeometry.Type] = field(
        default=RobotGeometry.Type.CYLINDER,
        converter=lambda value: RobotGeometry.Type.from_str(value),
    )
    geometry_params: np.ndarray = field(default=np.array([0.2, 1.0]))

    # Control limits
    ctrl_vx_limits: LinearCtrlLimits = field(
        default=LinearCtrlLimits(max_vel=1.0, max_acc=3.0, max_decel=5.0)
    )
    ctrl_omega_limits: AngularCtrlLimits = field(
        default=AngularCtrlLimits(
            max_vel=5.0, max_steer=math.pi, max_acc=10.0, max_decel=10.0
        )
    )
    ctrl_vy_limits: LinearCtrlLimits = field(
        default=LinearCtrlLimits(max_vel=0.0, max_acc=0.0, max_decel=0.0)
    )  # Default to ackermann robot

    @geometry_params.validator
    def validate_params(self, _, value):
        """
        Validates geometry parameters against geometry type
        """
        if not RobotGeometry.is_valid_parameters(self.geometry_type, value):
            raise ValueError(
                f"Robot geometry parameters '{value}' are incompatible with given robot geometry {self.geometry_type}. Requires '{RobotGeometry.ParamsLength[self.geometry_type].value}' strictly positive parameters"
            )


@define(kw_only=True)
class ComponentConfig(BaseComponentConfig):
    """
    KOMPASS Component extended parameters

    ```{list-table}
    :widths: 20 20 60
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **robot**
      - RobotConfig, `RobotConfig()`
      - Robot configuration parameters. See [RobotConfig](#kompass.config.RobotConfig) class for details.

    * - **frames**
      - RobotFrames, `RobotFrames()`
      - Robot TF frames configuration. See [RobotFrames](#kompass.config.RobotFrames) class for details.

    * - **topic_subscription_timeout**
      - float, `20.0`
      - Timeout when waiting for an input topic to become available (s)

    * - **topic_try_wait_timeout**
      - float, `0.1`
      - Time interval when waiting for input topic to become available (s)

    * - **topic_try_wait_timeout**
      - float, `0.1`
      - Time interval when waiting for input topic to become available (s)

    * - **core_log_level**
      - str, `LoggingSeverity.WARN`
      - Debug level for the component core algorithm
    ```
    """

    # Robot config
    robot: RobotConfig = field(default=Factory(RobotConfig))

    # Robot Frames
    frames: RobotFrames = field(default=Factory(RobotFrames))

    topic_subscription_timeout: float = field(
        default=20.0, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    topic_try_wait_timeout: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    core_log_level: str = field(
        default=LoggingSeverity.WARN, converter=_convert_logging_severity_to_str
    )
