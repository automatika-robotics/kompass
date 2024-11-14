![Logo](_static/Kompass_dark.png)

# Kompass

Kompass is a framework for building robust and comprehensive event-driven navigation stacks using an easy-to-use and intuitive Python API. Kompass is built to be customizable, extendable and hardware-agnostic. It aims to implement the most cutting edge algorithms for all parts of the navigation stack. And most importantly, it allows users to create very sophisticated navigation capabilities for autonomous mobile robots, within a single python script.

- Find out more about our [**motivation**](why.md) to create Kompass ✨
- [**Install**](install.md) Kompass on your robot 🛠️
- To get started with Kompass, check the [**quick start**](quick_start.md) tutorial 🚀
- Do a deep dive into Kompass [**components**](navigation/index.md) 🤖
- Learn more about the [**design concepts**](design/index.md) of Kompass 📚


Kompass is divided into several interacting components each responsible for one of the navigation subtasks:


```{figure} _static/images/diagrams/system_components_dark.png
:class: only-dark
:alt: Kompass Components and Main Tasks
:align: center

Kompass Components and Main Tasks
```

```{figure} _static/images/diagrams/system_components_light.png
:class: only-light
:alt: Kompass Components and Main Tasks
:align: center

Kompass Components and Main Tasks
```

Each of the previous components runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:



```{figure} /_static/images/diagrams/system_graph_dark.png
:class: only-dark
:alt: Kompass Full System
:align: center

System Diagram for Point Navigation
```

```{figure} /_static/images/diagrams/system_graph_light.png
:class: only-light
:alt: Kompass Full System
:align: center

System Diagram for Point Navigation
```


To learn more about the functionalities and configuration of each component check the component dedicated documentation page:

- [Planner](navigation/path_planning.md)
- [Controller](navigation/control.md)
- [Drive Manager](navigation/driver.md)
- [Motion Server](navigation/motion_server.md)
