---
title: Kompass Developer Documentation
---

# Kompass Developer Docs

Kompass is the navigation layer of the [EMOS](https://github.com/automatika-robotics/emos) ecosystem. It provides GPU-accelerated planning, control, and drive management for autonomous mobile robot navigation on ROS 2.

This site contains **developer documentation** for contributors extending the framework with new components, algorithms, or integrations.

:::{admonition} Looking for usage documentation?
:class: tip

Tutorials, installation guides, and usage documentation are on the
**[EMOS Documentation](https://emos.automatikarobotics.com)** site.
:::

---

## Extend & Customize

::::{grid} 1 2 2 3
:gutter: 3

:::{grid-item-card} {material-regular}`extension;1.5em;sd-text-primary` Control Algorithms
:link: development/adding_algorithms
:link-type: doc
:class-card: sugar-card

Add new path-following or vision-based control algorithms in Python or C++.
:::

:::{grid-item-card} {material-regular}`widgets;1.5em;sd-text-primary` Custom Components
:link: development/custom_component
:link-type: doc
:class-card: sugar-card

Build new ROS 2 lifecycle components with managed I/O, events, and health monitoring.
:::

:::{grid-item-card} {material-regular}`tune;1.5em;sd-text-primary` Advanced Components
:link: development/advanced_component
:link-type: doc
:class-card: sugar-card

Health status reporting, fallback triggers, event/action system, and component lifecycle hooks.
:::

:::{grid-item-card} {material-regular}`cable;1.5em;sd-text-primary` Data Types & I/O
:link: development/custom_callbacks_publishers
:link-type: doc
:class-card: sugar-card

Custom callbacks, publishers, processing pipelines, and the `SupportedType` registration system.
:::

:::{grid-item-card} {material-regular}`terminal;1.5em;sd-text-primary` CLI Reference
:link: development/cli_reference
:link-type: doc
:class-card: sugar-card

Query available algorithms, inspect default parameters, and check GPU accelerator support.
:::

::::

## Architecture & Internals

::::{grid} 1 2 2 3
:gutter: 3

:::{grid-item-card} {material-regular}`architecture;1.5em;sd-text-primary` System Design
:link: advanced/design
:link-type: doc
:class-card: sugar-card

Component architecture, data flow, and the three-layer stack (kompass / kompass-core / kompass_cpp).
:::

:::{grid-item-card} {material-regular}`functions;1.5em;sd-text-primary` Algorithm Details
:link: advanced/algorithms/index
:link-type: doc
:class-card: sugar-card

Mathematical formulations, cost functions, and implementation details for each built-in algorithm.
:::

:::{grid-item-card} {material-regular}`speed;1.5em;sd-text-primary` Benchmarks
:link: advanced/benchmark
:link-type: doc
:class-card: sugar-card

Performance benchmarks for CPU vs. GPU execution across algorithms and map resolutions.
:::

::::

## Integrations

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`route;1.5em;sd-text-primary` OMPL
:link: integrations/ompl
:link-type: doc
:class-card: sugar-card

Path planning with the Open Motion Planning Library via `omplpy` bindings.
:::

:::{grid-item-card} {material-regular}`view_in_ar;1.5em;sd-text-primary` FCL
:link: integrations/fcl
:link-type: doc
:class-card: sugar-card

Collision checking with the Flexible Collision Library for geometric robot models.
:::

::::

---

```{toctree}
:maxdepth: 2
:caption: Developer Guide
:hidden:

development/adding_algorithms
development/custom_component
development/advanced_component
development/custom_callbacks_publishers
development/cli_reference
```

```{toctree}
:maxdepth: 2
:caption: Architecture & Algorithms
:hidden:

advanced/design
advanced/algorithms/index
advanced/benchmark
```

```{toctree}
:maxdepth: 1
:caption: Integrations
:hidden:

integrations/ompl
integrations/fcl
```

```{toctree}
:maxdepth: 2
:caption: API Reference
:hidden:

apidocs/index
```
