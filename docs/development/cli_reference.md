---
title: CLI Tool Reference
---

# CLI Tool Reference

Kompass ships a command-line tool for inspecting available algorithms and their parameters. It is installed as a ROS 2 executable.

## Usage

```bash
ros2 run kompass kompass_cli <command> [subcommand] [args]
```

## Commands

### `controller list`

Lists all available control algorithms registered in `kompass_core`:

```bash
ros2 run kompass kompass_cli controller list
```

```
Available control algorithms:
- Stanley
- DWA
- DVZ
- VisionRGBFollower
- VisionRGBDFollower
- PurePursuit
```

These names correspond to `ControllersID` enum values and can be passed directly to `ControllerConfig(algorithm="...")`.

### `controller params <algorithm>`

Displays default configuration parameters for a control algorithm:

```bash
ros2 run kompass kompass_cli controller params DWA
```

```
Parameters for 'DWA' Controller in Kompass:
------------------------
Name: Default Value
control_time_step: 0.1
max_linear_samples: 20
max_angular_samples: 20
prediction_horizon: 1.0
control_horizon: 0.2
octree_resolution: 0.1
...
```

The output shows all `attrs` fields from the algorithm's config class (e.g. `DWAConfig`), with their default values. Use these names as keys in your YAML/TOML configuration files.

### `planner list`

Lists all available planning algorithms from OMPL Geometric Planners:

```bash
ros2 run kompass kompass_cli planner list
```

```
Available planning algorithms from OMPL Geometric Planners:
- RRT
- RRTstar
- RRTConnect
- PRM
- PRMstar
- KPIECE1
- BKPIECE1
- LazyPRM
- ...
```

### `planner params <algorithm>`

Displays default parameters for a planning algorithm:

```bash
ros2 run kompass kompass_cli planner params RRTstar
```

```
'ompl.geometric.RRTstar' Parameters:
------------------------
Name: Default Value
range: 0.0
goal_bias: 0.05
delay_collision_checking: True
...
```

The algorithm name matching is case-insensitive. You can use either the short name (`RRTstar`) or the full OMPL name (`ompl.geometric.RRTstar`).

### `accelerators_support`

Lists SYCL-compatible GPU accelerators available on the system:

```bash
ros2 run kompass kompass_cli accelerators_support
```

```
Available GPU accelerators:
- NVIDIA GeForce RTX 3080
```

Or if no GPU support is available:

```
No GPU accelerators are available on this machine
```

This requires `kompass-core` to be installed with GPU support.

### `info`

Displays help text with all commands and usage examples:

```bash
ros2 run kompass kompass_cli info
```
