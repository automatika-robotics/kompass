# Design Concepts

Kompass is designed using [ROS Sugar](https://automatika-robotics.github.io/ros-sugar). A [Component](https://automatika-robotics.github.io/ros-sugar/design/component.html) is the main execution unit in Kompass, each component is configured with [Inputs/Outputs](https://automatika-robotics.github.io/ros-sugar/design/topics.html) and [Fallback](https://automatika-robotics.github.io/ros-sugar/design/fallbacks.html) behaviors. Additionally, each component updates its own [Health Status](https://automatika-robotics.github.io/ros-sugar/design/status.html). Components can be handled and reconfigured dynamically at runtime using [Events](https://automatika-robotics.github.io/ros-sugar/design/events.html) and [Actions](https://automatika-robotics.github.io/ros-sugar/design/actions.html). Events, Actions and Components are passed to the [Launcher](https://automatika-robotics.github.io/ros-sugar/design/launcher.html) which runs the set of components as using multi-threaded or multi-process execution. The Launcher also uses an internal [Monitor](https://automatika-robotics.github.io/ros-sugar/design/monitor.html) to keep track of the components and monitor events.

```{seealso}
Check the design concepts in detail by referring to ROS Sugar [documentation](https://automatika-robotics.github.io/ros-sugar)
```


```{figure} ../_static/images/diagrams/component_dark.png
:class: only-dark
:alt: Kompass Component
:align: center

Component Structure
```

```{figure} ../_static/images/diagrams/component_light.png
:class: only-light
:alt: Kompass Component
:align: center

Component Structure
```

```{figure} ../_static/images/diagrams/multi_threaded_dark.png
:class: only-dark
:alt: Kompass Multi-threaded execution
:align: center

Multi-threaded execution
```

```{figure} ../_static/images/diagrams/multi_threaded_light.png
:class: only-light
:alt: Kompass Multi-threaded execution
:align: center

Multi-threaded execution
```


```{figure} ../_static/images/diagrams/multi_process_dark.png
:class: only-dark
:alt: Kompass Multi-process execution
:align: center

Multi-process execution
```

```{figure} ../_static/images/diagrams/multi_process_light.png
:class: only-light
:alt: Kompass Multi-process execution
:align: center

Multi-threaded execution
```
