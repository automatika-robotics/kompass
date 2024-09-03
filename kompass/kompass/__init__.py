"""Kompass navigation system"""

from typing import List, Optional

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
from kompass.components.component import Component

global components_base_dict, config_base_dict

components_base_dict = {
    "Planner": Planner,
    "Controller": Controller,
    "DriveManager": DriveManager,
    "MotionServer": MotionServer,
}

config_base_dict = {
    "PlannerConfig": PlannerConfig,
    "ControllerConfig": ControllerConfig,
    "DriveManagerConfig": DriveManagerConfig,
    "MotionServerConfig": MotionServerConfig,
}


def init(custom_components: Optional[List[type[Component]]] = None) -> None:
    global components_base_dict, config_base_dict

    if not custom_components:
        return

    # Update known components dict with selected custom components
    for comp in custom_components:
        comp_class = comp
        obj = comp(node_name="dummy_name")
        comp_config_class = obj.config.__class__
        components_base_dict[str(comp_class)] = comp_class
        config_base_dict[str(comp_config_class)] = comp_config_class


components_dict = components_base_dict
config_dict = config_base_dict


__all__ = ["components_dict", "config_dict", "init"]
