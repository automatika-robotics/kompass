"""Kompass navigation system"""

from kompass.components import (
    Controller,
    ControllerConfig,
    DriveManager,
    DriveManagerConfig,
    MotionServer,
    MotionServerConfig,
    Planner,
    PlannerConfig,
)

global components_base_dict, config_base_dict

components_list = [Planner, Controller, DriveManager, MotionServer]

configs_list = [
    PlannerConfig,
    ControllerConfig,
    DriveManagerConfig,
    MotionServerConfig,
]


__all__ = ["components_list", "configs_list"]
