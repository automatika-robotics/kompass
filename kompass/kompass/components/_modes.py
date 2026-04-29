"""Shared enums for the Controller component and its helpers."""

from ..utils import StrEnum


class CmdPublishType(StrEnum):
    """
    Control command publishing method:

    ```{list-table}
    :widths: 20 70
    :header-rows: 1

    * - Value
      - Description

    * - **TWIST_SEQUENCE (Literal "Sequence")**
      - the controller publishes a Twist message in the same thread running the control algorithm. If a series of commands is computed (up to the control horizon), the controller publishes the commands one by one before running the control algorithm again

    * - **TWIST_PARALLEL (Literal "Parallel")**
      - the controller handles publishing a Twist message in a new thread. If a series of commands is computed (up to the control horizon), the controller publishes the commands one by one in parallel while running the control algorithm again

    * - **TWIST_ARRAY (Literal "Array")**
      - the controller publishes a TwistArray msg of all the computed control commands (up to the control horizon)
    ```

    """

    TWIST_SEQUENCE = "Sequence"
    TWIST_PARALLEL = "Parallel"
    TWIST_ARRAY = "Array"


class ControllerMode(StrEnum):
    PATH_FOLLOWER = "path_follower"
    VISION_FOLLOWER = "vision_object_follower"


class FrameMode(StrEnum):
    """Frame in which the controller reasons about the world.

    - ``GLOBAL``: robot state and tracked targets live in the world frame.
      Requires a valid odom→world transform (or matching frames).
    - ``LOCAL``: robot state is ignored; sensor data and targets are treated
      as robot-relative. Only available in vision follower mode.
    """

    GLOBAL = "global"
    LOCAL = "local"


class PathControlStatus(StrEnum):
    """Outcome of a single ``Controller._path_control`` step."""

    IDLE = "idle"  # no controller or no path set
    WAITING_INPUTS = "waiting"  # robot state / TF not yet available
    RUNNING = "running"  # command computed and published
    GOAL_REACHED = "goal_reached"  # robot inside goal tolerance
    FAILED = "failed"  # algorithm could not produce a command
