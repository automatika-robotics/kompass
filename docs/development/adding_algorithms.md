---
title: Adding a New Control Algorithm
---

# Adding a New Control Algorithm

Kompass control algorithms live in [`kompass-core`](https://github.com/automatika-robotics/kompass-core) and are dispatched by the Controller component through enum-based registries (`ControlClasses` / `ControlConfigClasses`).

There are two ways to add a new algorithm, depending on where the core logic lives:

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`code;1.5em;sd-text-primary` Python Algorithm
:link: adding_python_algorithms
:link-type: doc
:class-card: sugar-card

**Pure Python or Python + C++ wrapper**

Implement your algorithm entirely in Python by inheriting `ControllerTemplate`, or write a Python wrapper around an existing C++ backend by inheriting `FollowerTemplate`. Covers configuration, registration, single vs. multi-command (MPC-style) output, and publishing modes.
:::

:::{grid-item-card} {material-regular}`memory;1.5em;sd-text-primary` C++ Algorithm
:link: adding_cpp_algorithms
:link-type: doc
:class-card: sugar-card

**C++ implementation with nanobind bindings**

Implement a high-performance path-following algorithm in C++ by extending the `Follower` class in `kompass_cpp`. Covers the C++ class hierarchy, parameter system, nanobind bindings, and the build pipeline.
:::

::::

## Architecture at a Glance

| Layer | Role |
|---|---|
| **kompass** | ROS 2 lifecycle node (`Controller`), topic I/O, config bridging |
| **kompass-core** | Algorithm Python classes, config (`attrs`), registry enums |
| **kompass_cpp** | Performance-critical C++ implementations, exposed via nanobind |

### Existing Algorithms

| `ControllersID` | Algorithm | Base Class | Config Class |
|---|---|---|---|
| `DWA` | Dynamic Window Approach | `FollowerTemplate` | `DWAConfig` |
| `PURE_PURSUIT` | Pure Pursuit | `FollowerTemplate` | `PurePursuitConfig` |
| `STANLEY` | Stanley Controller | `FollowerTemplate` | `StanleyConfig` |
| `DVZ` | Deformable Virtual Zone | `FollowerTemplate` | `DVZConfig` |
| `VISION_IMG` | Vision Follower (RGB) | `ControllerTemplate` | `VisionRGBFollowerConfig` |
| `VISION_DEPTH` | Vision Follower (RGB-D) | `ControllerTemplate` | `VisionRGBDFollowerConfig` |

### Which approach to choose?

- **Python-only algorithm** (e.g. a proportional controller, a vision-based follower) -- use `ControllerTemplate`. No C++ needed.
- **Python wrapper for an existing C++ follower** -- use `FollowerTemplate`. Gets path management for free from the C++ backend.
- **New high-performance path follower from scratch** -- implement in C++ first ([C++ guide](./adding_cpp_algorithms.md)), then wrap in Python ([Python guide](./adding_python_algorithms.md)).

```{toctree}
:maxdepth: 2
:caption: Developer Guide
:hidden:

adding_python_algorithms
adding_cpp_algorithms
```
