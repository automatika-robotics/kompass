# Navigation Components Overview

**The modular building blocks of Kompass**

Kompass is designed to make autonomous navigation **easy to set up, flexible to customize, and powerful enough for robots working in changing real-world conditions**.
Think of it as a collection of building blocks: you choose the ones you need, configure them for your robot, and you're good to go!

```{figure} ../_static/images/diagrams/system_components_dark.png
:class: dark-only
:alt: Kompass Components and Main Tasks
:align: center

```

```{figure} ../_static/images/diagrams/system_components_light.png
:class: light-only
:alt: Kompass Components and Main Tasks
:align: center

Kompass Components and Main Tasks
```

Each component runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:



```{figure} ../_static/images/diagrams/system_graph_dark.png
:class: dark-only
:alt: Kompass Full System
:align: center

```

```{figure} ../_static/images/diagrams/system_graph_light.png
:class: light-only
:alt: Kompass Full System
:align: center

System Diagram for Point Navigation
```

In the following, we give a quick tour of the core navigation components, what each one does, and what sensors you actually need to get Kompass up and running.

---

## Core Components

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`map;1.5em;sd-text-primary` Global Planner
:link: path_planning
:link-type: doc
:class-card: sugar-card

**Long-range Path finding**
Computes the optimal route from the robot's current pose to a goal.
:::

:::{grid-item-card} {material-regular}`settings_input_component;1.5em;sd-text-primary` Controller
:link: control
:link-type: doc

**Local Motion Control**
Generates precise motion commands.

:::

:::{grid-item-card} {material-regular}`layers;1.5em;sd-text-primary` Local Mapper
:link: mapper
:link-type: doc

**Real-time Environment Context**
Builds a high-speed occupancy representation of the immediate surroundings.
:::

:::{grid-item-card} {material-regular}`precision_manufacturing;1.5em;sd-text-primary` Drive Manager
:link: driver
:link-type: doc

**Hardware Interface & Safety**
The final gateway to the robot's actuation.
:::

:::{grid-item-card} {material-regular}`storage;1.5em;sd-text-primary` Map Server
:link: map_server
:link-type: doc

**Global Data Management**
Serves static 2D/3D map files to the rest of the stack.

:::

:::{grid-item-card} {material-regular}`fiber_manual_record;1.5em;sd-text-primary` Motion Server
:link: map_server
:link-type: doc

**Calibration & recording**
Runs automated tests and performs motion recording for calibration and validation

:::

::::




---

## Sensing Requirements

Kompass scales with your hardware, from minimal lab setups to sensor-heavy industrial platforms.

### {material-regular}`check_circle` Minimum Requirements

To get the baseline navigation running, you need:

* **Odometry:** Wheel encoders, IMU, or Visual Odometry.
* **Obstacle Detection:** 2D LiDAR or a Depth Camera.
* **Global Pose:** A localization source (e.g., AMCL, SLAM, or GPS).

### {material-regular}`add_circle` Enhanced Features

Unlock advanced capabilities with additional sensors:

* **Depth Cameras:** Enable 3D obstacle avoidance and robust **RGB-D Vision Following**.
* **3D LiDAR:** High-fidelity perception in complex, multi-level environments.
* **RGB Cameras:** Enable semantic understanding and object-based navigation.
* **GPS/UWB:** Precision localization for outdoor or large-scale indoor deployment.
