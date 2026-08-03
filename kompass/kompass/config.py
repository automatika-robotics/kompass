"""Configuration classes for a Component and a robot in Kompass"""

import ros_sugar.config.base_validators as BaseValidators
from ros_sugar.config.base_config import (
    _convert_logging_severity_to_str,
    LoggingSeverity,
)
from attrs import define, field
from ros_sugar.config import (
    AngularCtrlLimits,
    BaseAttrs,
    BaseComponentConfig,
    BaseConfig,
    ComponentRunType,
    LinearCtrlLimits,
    RobotConfig,
    RobotFrames,
    RobotGeometryType,
    RobotType,
)

__all__ = [
    "BaseAttrs",
    "BaseValidators",
    "BaseConfig",
    "ComponentRunType",
    "BaseComponentConfig",
    "RobotConfig",
    "RobotFrames",
    "RobotType",
    "RobotGeometryType",
    "LinearCtrlLimits",
    "AngularCtrlLimits",
]


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
    topic_subscription_timeout: float = field(
        default=5.0, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    topic_try_wait_timeout: float = field(
        default=0.05, validator=BaseValidators.in_range(min_value=1e-3, max_value=1e9)
    )

    core_log_level: str = field(
        default=LoggingSeverity.WARN, converter=_convert_logging_severity_to_str
    )

    # Override to Disable handling the robot plugin in the component
    _enable_plugin_feedbacks_handling: bool = field(
        default=True, alias="_enable_plugin_feedback_handling"
    )
    _enable_plugin_actions_handling: bool = field(
        default=False, alias="_enable_plugin_actions_handling"
    )
