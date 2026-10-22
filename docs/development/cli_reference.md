---
title: CLI Tool Reference
---

# CLI Tool Reference

Kompass ships a command-line tool for inspecting available algorithms and their parameters. It is installed as a ROS 2 executable.

## Usage

```bash
ros2 run kompass cli <command> [subcommand] [args]
```

## Commands

### `controller list`

Lists all available control algorithms registered in `kompass_core`:

```bash
ros2 run kompass cli controller list
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
ros2 run kompass cli controller params DWA
```

```
Parameters for 'DWA' Controller in Kompass:
------------------------
Name: Default Value

max_point_interpolation_distance: 0.01
lookahead_distance: 1.0
goal_dist_tolerance: 0.1
goal_orientation_tolerance: 0.1
path_segment_length: 1.0
...
```

The output shows all `attrs` fields from the algorithm's config class (e.g. `DWAConfig`), with their default values. Use these names as keys in your YAML/TOML configuration files.

### `planner list`

Lists all available planning algorithms from OMPL Geometric Planners:

```bash
ros2 run kompass cli planner list
```

```
Available planning algorithms from OMPL Geometric Planners:
- ABITstar
- AITstar
- BFMT
- BITstar
- BKPIECE1
- ...
- RRTstar
- ...
```

### `planner params <algorithm>`

Displays default parameters for a planning algorithm:

```bash
ros2 run kompass cli planner params RRTstar
```

```
'ompl.geometric.RRTstar' Parameters:
------------------------
Name: Default Value

delay_collision_checking: True
focus_search: False
goal_bias: 0.05
...
```

The algorithm name matching is case-insensitive. You can use either the short name (`RRTstar`) or the full OMPL name (`ompl.geometric.RRTstar`).

### `accelerators_support`

Lists SYCL-compatible GPU accelerators available on the system:

```bash
ros2 run kompass cli accelerators_support
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

Displays help text with all commands and usage examples, followed by links to the [Kompass Documentation](https://emos.automatikarobotics.com) and these Developer Docs:

```bash
ros2 run kompass cli info
```

For argument help, use `-h` (e.g. `ros2 run kompass cli controller -h`); usage and error lines are printed as `usage: ros2 run kompass cli ...`.
