

# Kompass

**Robust, event-driven navigation stacks for autonomous mobile robots.**

<p style="font-size: 1.1em; opacity: 0.8; max-width: 800px; margin: 0 auto;">
  Built to be customizable, extendable, and hardware-agnostic. Create sophisticated navigation capabilities within a <b>single Python script</b> without sacrificing performance.
</p>

[Get Started](tutorials/quick_start.md) • [Why Kompass?](./why.md) • [View on GitHub](https://github.com/automatika-robotics/kompass)

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">Hardware-Agnostic GPU Acceleration - </span> Kompass includes **highly optimized, GPU powered, versions of the most cutting edge navigation algorithms** in C++ that make full use of available hardware resources. It supports multi-threaded execution on CPUs and can run on <span class="text-red-strong">ANY GPU</span> (Nvidia, AMD, etc.) without vendor lock-in, making it suitable for both development and deployment across diverse hardware setups.


- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">Universal Recipes (Apps) - </span> Kompass provides an **intuitive Python API** making it straightforward to create and deploy sophisticated navigation capabilities within a **single Python script**, to run across different robots without sacrificing performance or flexibility.


- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">Event-Driven Architecture - </span> Built on top of [Sugarcoat](https://automatika-robotics.github.io/sugarcoat/), Kompass components are adaptive and resilient. The system can dynamically reconfigure itself and perform custom actions based on environmental changes or internal status events.

## Explore Kompass

::::{grid} 1 2 3 3
:gutter: 2

:::{grid-item-card} {material-regular}`download;1.2em;sd-text-primary` Installation
:link: install
:link-type: doc
:class-card: sugar-card

Install Kompass on your system
:::

:::{grid-item-card} {material-regular}`rocket_launch;1.2em;sd-text-primary` Quick Start
:link: tutorials/quick_start
:link-type: doc
:class-card: sugar-card

Get your robot moving in minutes
:::

:::{grid-item-card} {material-regular}`lightbulb;1.2em;sd-text-primary` Motivation
:link: why
:link-type: doc
:class-card: sugar-card

Why we built Kompass
:::

:::{grid-item-card} {material-regular}`menu_book;1.2em;sd-text-primary` Tutorials
:link: tutorials/index
:link-type: doc
:class-card: sugar-card

Learn how to create custom navigation capabilities
:::

:::{grid-item-card} {material-regular}`bolt;1.2em;sd-text-primary` Benchmarks
:link: advanced/benchmark
:link-type: doc
:class-card: sugar-card

See performance across hardware (CPUs & GPUs)
:::


:::{grid-item-card} {material-regular}`desktop_windows;1.2em;sd-text-primary` Web UI
:link: https://sugarcoat.automatikarobotics.com/features/web_ui.html
:class-card: sugar-card

Explore the Zero-Code Dynamic Web UI
:::

:::{grid-item-card} {material-regular}`power;1.2em;sd-text-primary` Universal Robot Plugins
:link: https://sugarcoat.automatikarobotics.com/features/robot_plugins.html
:class-card: sugar-card

Port automation recipes across different hardware
:::

:::{grid-item-card} {material-regular}`extension;1.2em;sd-text-primary` Design Concepts
:link: advanced/design
:link-type: doc
:class-card: sugar-card

Explore the core design concepts
:::

:::{grid-item-card} {material-regular}`smart_toy;1.2em;sd-text-primary` AI-Assisted Coding
:link: llms.txt
:link-type: url

Get the `llms.txt` for your coding-agent and let it write the recipes
:::

::::


## Architecture

Kompass has a <span class="text-red-strong">Modular Event-Driven Architecture</span>. It is divided into several interacting components each responsible for one of the navigation subtasks.

Learn more on Kompass [design concepts](advanced/design.md)

::::{tab-set}

:::{tab-item} Event-Driven Design
:sync: events
<br/>
Kompass has a dynamic orchestration layer that monitors data streams in real-time. Using the Pythonic API you can define Events to injects logic into the components without changing their code, allowing you to define complex and dynamic behaviors directly in your recipes.


```{figure} _static/images/diagrams/events_examples_dark.png
:class: dark-only
:alt: Events Examples
:align: center
:width: 80%

```

```{figure} _static/images/diagrams/events_examples_light.png
:class: light-only
:alt: Events Examples
:align: center
:width: 80%

```

:::


:::{tab-item} High-Level Components
:sync: components
<br/>
Each component in Kompass is responsible of one of the main navigation sub-tasks. Unlike a standard ROS2 node, a Component manages its own lifecycle, validates its own configuration, and reports its own health status to the central system monitor.

```{figure} _static/images/diagrams/system_components_dark.png
:class: dark-only
:alt: Components
:align: center

```

```{figure} _static/images/diagrams/system_components_light.png
:class: light-only
:alt: Components
:align: center

The main pillars of Kompass navigation stack.
```

:::

:::{tab-item} System Graph (ROS2)
:sync: graph
<br/>
Each of the previous components runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers

```{figure} /_static/images/diagrams/system_graph_dark.png
:class: dark-only
:alt: Components in a point-navigation system
:align: center

```

```{figure} /_static/images/diagrams/system_graph_light.png
:class: light-only
:alt: Components in a point-navigation system
:align: center

Components in a point-navigation system
```

:::

::::



## Components Reference

Learn more about configuring your robot and explore the functionalities and configuration of each component in the dedicated documentation page:

::::{grid} 1 2 2 4
:gutter: 2

:::{grid-item-card} {material-regular}`precision_manufacturing;1.2em;sd-text-primary` Robot Config
:link: navigation/robot
:link-type: doc

:::

:::{grid-item-card} {material-regular}`alt_route;1.2em;sd-text-primary` Planner
:link: navigation/path_planning
:link-type: doc

:::

:::{grid-item-card} {material-regular}`control_camera;1.2em;sd-text-primary` Controller
:link: navigation/control
:link-type: doc

:::

:::{grid-item-card} {material-regular}`radar;1.2em;sd-text-primary` Local Mapper
:link: navigation/mapper
:link-type: doc

:::

:::{grid-item-card} {material-regular}`directions_car;1.2em;sd-text-primary` Drive Manager
:link: navigation/driver
:link-type: doc

:::

:::{grid-item-card} {material-regular}`map;1.2em;sd-text-primary` Map Server
:link: navigation/map_server
:link-type: doc

:::

:::{grid-item-card} {material-regular}`traffic;1.2em;sd-text-primary` Motion Server
:link: navigation/motion_server
:link-type: doc

:::
::::

## Contributions

Kompass has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.
