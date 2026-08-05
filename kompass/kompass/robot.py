# The robot description a recipe declares.

from .config import (
    RobotConfig,
    RobotFrames,
    RobotType,
    RobotGeometryType,
    LinearCtrlLimits,
    AngularCtrlLimits,
)


__all__ = [
    "RobotConfig",
    "RobotFrames",
    "RobotType",
    "RobotGeometryType",
    "LinearCtrlLimits",
    "AngularCtrlLimits",
]


# Backward compatibility: Names this module used to re-export from kompass_core.
_MOVED = {
    "RobotGeometry": (
        "Declare the shape with 'RobotGeometryType' instead:\n"
        "    from kompass.robot import RobotConfig, RobotGeometryType\n"
        "    RobotConfig(geometry_type=RobotGeometryType.CYLINDER, ...)\n"
        "'RobotGeometry.Type' no longer works here.\n"
        "If you genuinely need the geometry helpers (get_radius, get_height, "
        "get_footprint), import them from 'kompass_core.models'."
    ),
    "RobotCtrlLimits": (
        "Declare the velocity envelope on the robot description instead:\n"
        "    from kompass.robot import RobotConfig, LinearCtrlLimits, AngularCtrlLimits\n"
        "    RobotConfig(ctrl_vx_limits=LinearCtrlLimits(...), "
        "ctrl_omega_limits=AngularCtrlLimits(...), ...)\n"
        "If you are writing a component and need that type directly, import it "
        "from 'kompass_core.models'."
    ),
}


def __getattr__(name: str):
    """Point at the new spelling for names this module no longer exports.

    ``ImportError`` rather than ``AttributeError`` on purpose: a ``from ...
    import ...`` statement discards an ``AttributeError`` and replaces it with
    its own generic message, which would throw this guidance away.
    """
    if name in _MOVED:
        raise ImportError(
            f"'{name}' is no longer part of 'kompass.robot'.\n\n{_MOVED[name]}"
        )
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
