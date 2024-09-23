"""Configuration classes for a Component and a robot in Kompass"""

import math
from typing import Union

import numpy as np
import ros_sugar.config.base_validators as BaseValidators
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
    QoSConfig,
)

__all__ = [
    "BaseAttrs",
    "BaseValidators",
    "BaseConfig",
    "ComponentRunType",
    "RobotFrames",
    "RobotConfig",
    "BaseComponentConfig",
    "QoSConfig",
]


@define(kw_only=True)
class RobotFrames(BaseAttrs):
    """
    Class for robot coordinate frames configuration
    """

    robot_base: str = field(default="base_link")
    odom: str = field(default="odom")
    world: str = field(default="map")
    scan: str = field(default="base_link")
    rgb: str = field(default="camera_link")
    depth: str = field(default="camera_depth_link")


@define(kw_only=True)
class RobotConfig(BaseAttrs):
    """
    Class for robot configuration (type, model, limitations, etc.)
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
    """

    topic_subscription_timeout: float = field(
        default=20.0, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    topic_try_wait_timeout: float = field(
        default=0.1, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    # Robot config
    robot: RobotConfig = field(default=Factory(RobotConfig))

    # Robot Frames
    frames: RobotFrames = field(default=Factory(RobotFrames))

    action_type: str = field(default="test")  # used only if run_type='ActionServer'
    service_type: str = field(default="test")  # used only if run_type='Server'
