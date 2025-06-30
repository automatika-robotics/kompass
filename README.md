<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/Kompass_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/Kompass_light.png">
  <img alt="Kompass Logo." src="docs/_static/Kompass_light.png"  width="50%">
</picture>
<br/>

🇨🇳  [简体中文](docs/README.zh.md) | 🇯🇵  [日本語](docs/README.ja.md)

Welcome to Kompass! The fastest and most intuitive navigation stack known to man! Kompass is a framework for building **robust**, **event-driven** navigation stacks for autonomous mobile robots. Kompass is built to be customizable, extendable and hardware-agnostic. It provides an **intuitive Python API** designed to be easy to integrate, extend, and adapt to a wide range of use cases.

Kompass includes **highly optimized, GPU powered, versions of the most cutting edge navigation algorithms in C++** that make full use of available hardware resources. It supports **multi-threaded execution on CPUs** and can run on **ANY GPU** (Nvidia, AMD, etc.) without vendor lock-in. This makes it suitable for both development and deployment across diverse hardware setups. And most importantly, Kompass makes it straightforward to create and deploy sophisticated navigation capabilities for any mobile robot within **a single Python script**, without sacrificing performance or flexibility.

- [**Install**](#installation) Kompass on your robot 🛠️
- [**Why Kompass?**](#why-kompass)
- See an overview of Kompass [**Components**](#components)
- To get started with Kompass, check the [**quick start**](https://automatika-robotics.github.io/kompass/quick_start.html) tutorial 🚀
- Do a deep dive with one of the [**tutorials**](https://automatika-robotics.github.io/kompass/tutorials/point_navigation.html) 🤖
- Learn more about the [**design concepts**](https://automatika-robotics.github.io/kompass/advanced/design.html) of Kompass 📚

# Why Kompass?

- **Adaptive Event-Driven Design**: Kompass is built to be responsive to real-world events, robot state changes, and task updates. Its event-driven architecture makes it easy to define event-action pairs to reconfigure the navigation stack during runtime, or to seamlessly switch planning or control strategies based on environment context.

- **Engineered for Speed - C++, Multi-Threading, and Cross-GPU Support**: All core algorithms are written in modern C++ for fast and safe execution ([kompass-core](https://github.com/automatika-robotics/kompass-core)). Kompass is the first navigation framework to explicitly support GPU based execution of primary navigation components. Furthermore its built with GPGPU support, unlocking high performance on any CPUs, GPUs, or even FPGAs, breaking the hardware vendor lock-in of traditional frameworks.

- **Machine learning models as first class citizens**: External events in Kompass can be driven by outputs of machine learning models interpreting sensor data or user commands, which means the entire stack becomes reconfigurable based on ML model outputs. This goes beyond well established scenarios of visual navigation.

- **Pythonic API with Native Speed**: While the heavy lifting is done in optimized C++, Kompass provides an intuitive Python API, letting developers prototype quickly and deploy high-performance systems without rewriting code.

- **Modular architecture and easy extensibility**: Kompass is built on ROS2 using [Sugarcoat🍬](https://github.com/automatika-robotics/sugarcoat). It decouples core algorithms from the ROS2 interface, Kompass ensures compatibility across ROS2 versions and simplifies core upgrades and community extensions.

Find out more about our [**motivation**](https://automatika-robotics.github.io/kompass/why.html) to create Kompass.

# Components

Kompass is divided into several interacting components each responsible for one of the navigation subtasks:

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/system_components_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/system_components_light.png">
  <img alt="Kompass Components Tasks" src="docs/_static/images/diagrams/system_components_dark.png"  width="100%">
</picture>

Each of the previous components runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/system_graph_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/system_graph_light.png">
  <img alt="System Diagram for Point Navigation" src="docs/_static/images/diagrams/system_graph_dark.png"  width="100%">
</picture>

To learn more about the functionalities and configuration of each component check the component dedicated documentation page:

- [Planner](https://automatika-robotics.github.io/kompass/navigation/path_planning.html)
- [Controller](https://automatika-robotics.github.io/kompass/navigation/control.html)
- [Drive Manager](https://automatika-robotics.github.io/kompass/navigation/driver.html)
- [Motion Server](https://automatika-robotics.github.io/kompass/navigation/motion_server.html)

# Installation

## Prerequisites

Kompass is built to be used with ROS2. All ROS2 distributions starting from _Foxy_ upto _Rolling_ are supported. Install ROS2 version of your choice by following the instructions on the [official site](https://docs.ros.org/).

## Install kompass-core

kompass-core is a python package that provides highly optimized implementations of planning and control algorithms for Kompass. You can install it in the following ways:

### With GPU support (Recommended):

On any Ubuntu (including Jetpack) based machine, you can simply run the following:

```bash
curl https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash
```

This script will install all relevant dependencies, including [AdaptiveCPP](https://github.com/AdaptiveCpp/AdaptiveCpp) and install the latest version of kompass-core from source. It is good practice to read the [script](https://github.com/automatika-robotics/kompass-core/blob/main/build_dependencies/install_gpu.sh) first.

### Installing with pip

On Ubuntu versions >= 22.04, install dependencies by running the following:

```bash
sudo apt-get install libompl-dev libfcl-dev libpcl-dev
```

Then install kompass-core as follows:

```bash
pip install kompass-core
```

## Install Kompass (available for `humble` and any distribution >= `jazzy` )

Install pre-build Kompass binary as follows:

```bash
sudo apt install ros-$ROS_DISTRO-kompass
```


## Build Kompass from source

You can build Kompass from source as follows:

```shell
mkdir -p kompass_ws/src
cd kompass_ws/src
git clone https://github.com/automatika-robotics/sugarcoat
git clone https://github.com/automatika-robotics/kompass
rosdep update
rosdep install -y --from-paths . --ignore-src
cd ..
colcon build
```

# Copyright

The code in this distribution is Copyright (c) 2024 Automatika Robotics unless explicitly indicated otherwise.

Kompass is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.

# Contributions

Kompass has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.
