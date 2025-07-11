<div>
  <img src="_static/Kompass_light.png" class="only-light" />
  <img src="_static/Kompass_dark.png" class="only-dark" />
</div>

# Kompass

Kompass is a framework for building **robust**, **event-driven** navigation stacks for autonomous mobile robots. Kompass is built to be customizable, extendable and hardware-agnostic. It provides an **intuitive Python API** designed to be easy to integrate, extend, and adapt to a wide range of use cases.

Kompass includes **highly optimized, GPU powered, versions of the most cutting edge navigation algorithms in C++** that make full use of available hardware resources. It supports **multi-threaded execution on CPUs** and can run on <span class="text-red-strong">ANY GPU</span> (Nvidia, AMD, etc.) without vendor lock-in. This makes it suitable for both development and deployment across diverse hardware setups. And most importantly, Kompass makes it straightforward to create and deploy sophisticated navigation capabilities for any mobile robot within **a single Python script**, without sacrificing performance or flexibility.

- Find out more about our [**motivation**](why.md) to create Kompass ✨
- [**Install**](install.md) Kompass on your robot 🛠️
- To get started with Kompass, check the [**quick start**](tutorials/quick_start.md) tutorial 🚀
- Do a deep dive with one of the [**tutorials**](tutorials/index.md) 🤖
- Learn more about the [**design concepts**](advanced/design.md) of Kompass 📚


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
