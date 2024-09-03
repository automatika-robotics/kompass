![Logo](_static/Kompass_dark.png)

# Kompass

Kompass is an event-driven navigation system designed with an easy-to-use and intuitive Python API. Kompass is built to be customizable, extendable and hardware-agnostic. It aims to implement the most cutting edge algorithms for all parts of the navigation stack. And most importantly, it allows users to create very sophisticated navigation capabilities for autonomous mobile robots, within a single python script.

- Find out more about our [**motivation**](why.md) to create Kompass ✨
- [**Install**](install.md) Kompass on your robot 🛠️
- To get started with Kompass, check the [**quick start**](quick_start.md) tutorial 🚀
- Do a deep dive into Kompass [**components**](navigation/index.md) 🤖
- Learn more about the [**design concepts**](design/index.md) of Kompass 📚

```{note}
This is an alpha release of Kompass. Breaking changes are to be expected.
```

Kompass is divided into several interacting components each responsible for one of the navigation subtasks:

:::{figure-md} fig-components-tasks

<img src="_static/images/diagrams/system_components.jpg" alt="Kompass Components Tasks" width="1000px">

Kompass Components and Main Tasks
:::

Each of the previous components runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:

:::{figure-md} fig-components-system

<img src="_static/images/diagrams/system_graph.jpg" alt="Kompass Full System" width="1000px">

Kompass Full System for Autonomous Navigation
:::

To learn more about the functionalities and configuration of each component check the component dedicated documentation page:

- [Planner](navigation/path_planning.md)
- [Controller](navigation/control.md)
- [Drive Manager](navigation/driver.md)
- [Motion Server](navigation/motion_server.md)
