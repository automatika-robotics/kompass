<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/Kompass_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/Kompass_light.png">
  <img alt="Kompass Logo" src="docs/_static/Kompass_light.png" width="600">
</picture>

<br/>

Part of the [EMOS](https://github.com/automatika-robotics/emos) ecosystem

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS2](https://img.shields.io/badge/ROS2-Foxy%2B-green)](https://docs.ros.org/en/humble/index.html)
[![Discord](https://img.shields.io/badge/Discord-%235865F2.svg?logo=discord&logoColor=white)](https://discord.gg/B9ZU6qjzND)

**High-Performance, Event-Driven Navigation Stack for ROS2**

[**EMOS Documentation**](https://emos.automatikarobotics.com) | [**Developer Docs**](https://automatika-robotics.github.io/kompass/) | [**Discord**](https://discord.gg/B9ZU6qjzND)

🇨🇳 [简体中文](docs/README.zh.md) | 🇯🇵 [日本語](docs/README.ja.md)

</div>

---

## What is Kompass?

**Kompass** is the navigation layer of [EMOS](https://github.com/automatika-robotics/emos) (Embodied Operating System) and a framework for building **robust**, **event-driven** navigation stacks for autonomous mobile robots. It is built to be customizable, extendable and hardware-agnostic.

Kompass includes **highly optimized, GPU-powered navigation algorithms in C++** that make full use of available hardware resources. It supports **multi-process parallelism on CPUs** and highly parallelized execution on **any GPU** (Nvidia, AMD, etc.) without vendor lock-in. And most importantly, Kompass makes it straightforward to create and deploy sophisticated navigation capabilities for any mobile robot within **a single Python script**, without sacrificing performance or flexibility.

For full documentation, tutorials, and recipes, visit [emos.automatikarobotics.com](https://emos.automatikarobotics.com).

- [**Key Features**](#key-features)
- [**Components**](#components)
- [**Installation**](#installation)
- [**Benchmarking Results**](#benchmarking-results)
- [**Dynamic Web UI**](#dynamic-web-ui)
- [**EMOS Ecosystem**](#part-of-the-emos-ecosystem)

---

## Key Features

- **Adaptive Event-Driven Design**: Responsive to real-world events, robot state changes, and task updates. Define event-action pairs to reconfigure the navigation stack at runtime, or seamlessly switch planning or control strategies based on environment context.

- **Engineered for Speed**: All core algorithms are written in modern C++ ([kompass-core](https://github.com/automatika-robotics/kompass-core)). First navigation framework to explicitly support GPU-based execution via GPGPU, unlocking high performance on any hardware without vendor lock-in.

- **ML Models as First-Class Citizens**: External events can be driven by ML model outputs interpreting sensor data or user commands. The entire stack becomes reconfigurable based on ML inference, going beyond well-established visual navigation scenarios.

- **Pythonic API with Native Speed**: While the heavy lifting is done in optimized C++, Kompass provides an intuitive Python API. Prototype quickly and deploy high-performance systems without rewriting code.

---

## Components

Kompass is divided into several interacting components, each responsible for a navigation subtask:

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/system_components_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/system_components_light.png">
  <img alt="Kompass Components" src="docs/_static/images/diagrams/system_components_dark.png" width="100%">
</picture>
</div>

Each component runs as a ROS 2 lifecycle node and communicates using topics, services, or action servers:

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/images/diagrams/system_graph_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/images/diagrams/system_graph_light.png">
  <img alt="System Diagram" src="docs/_static/images/diagrams/system_graph_dark.png" width="100%">
</picture>
</div>

Learn more about each component in the [EMOS Documentation](https://emos.automatikarobotics.com):
[Planner](https://emos.automatikarobotics.com/kompass/navigation/path_planning.html) |
[Controller](https://emos.automatikarobotics.com/kompass/navigation/control.html) |
[Local Mapper](https://emos.automatikarobotics.com/kompass/navigation/mapper.html) |
[Map Server](https://emos.automatikarobotics.com/kompass/navigation/map_server.html) |
[Drive Manager](https://emos.automatikarobotics.com/kompass/navigation/driver.html) |
[Motion Server](https://emos.automatikarobotics.com/kompass/navigation/motion_server.html)

---

## Installation

For detailed installation instructions, see the [EMOS Documentation](https://emos.automatikarobotics.com).

### Install kompass-core

**With GPU support (recommended):**

```bash
curl -sSL https://raw.githubusercontent.com/automatika-robotics/kompass-core/refs/heads/main/build_dependencies/install_gpu.sh | bash
```

**With pip (CPU only):**

```bash
sudo apt-get install -y libompl-dev libfcl-dev && pip install kompass-core
```

### Install Kompass

**Quick install (Ubuntu/Debian, ROS 2 Humble+ or Jazzy+):**

```bash
sudo apt install ros-$ROS_DISTRO-kompass
```

**From source (for contributors):**

```bash
mkdir -p kompass_ws/src && cd kompass_ws/src
git clone https://github.com/automatika-robotics/sugarcoat
git clone https://github.com/automatika-robotics/kompass
rosdep update && rosdep install -y --from-paths . --ignore-src
cd .. && colcon build
```

---

## Benchmarking Results

The plots below compare CPU vs. GPU performance across navigation components. See the [benchmarking details](https://github.com/automatika-robotics/kompass-core/blob/main/src/kompass_cpp/benchmarks/README.md) for methodology.

### Logarithmic Scale (CPU vs GPU)

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_light.png">
  <img alt="Logarithmic Benchmark Results" src="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_log_light.png" width="60%">
</picture>
</div>

### Power Consumption & Efficiency

_Efficiency = Operations per Joule (Throughput / Watts). Higher is better._

<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_dark.png">
  <source media="(prefers-color-scheme: light)" srcset="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_light.png">
  <img alt="Power Benchmark Results" src="https://raw.githubusercontent.com/automatika-robotics/kompass-core/main/docs/benchmark_power_light.png" width="60%">
</picture>
</div>

---

## Dynamic Web UI

Every Kompass recipe generates a fully dynamic Web UI automatically. Built with FastHTML on the underlying [Sugarcoat](https://github.com/automatika-robotics/sugarcoat) framework, it provides instant control and visualization without writing frontend code.

<div align="center">
<picture>
  <img alt="Kompass UI" src="./docs/_static/gif/ui_navigation.gif" width="60%">
</picture>
</div>

---

## Part of the EMOS Ecosystem

Kompass is one of three core open-source components in [EMOS](https://github.com/automatika-robotics/emos) (Embodied Operating System) the unified orchestration layer for Physical AI:

- **[EmbodiedAgents](https://github.com/automatika-robotics/embodied-agents)**: Intelligence and manipulation. ML model graphs with semantic memory and adaptive reconfiguration.
- **[Kompass](https://github.com/automatika-robotics/kompass)**: Navigation. GPU-accelerated planning and control.
- **[Sugarcoat](https://github.com/automatika-robotics/sugarcoat)**: Robust & Event-driven system design for ROS 2.

Write a recipe once. Deploy it on any robot. No code changes.

---

## Resources

- [EMOS Documentation](https://emos.automatikarobotics.com): Tutorials, recipes, and usage guides
- [Developer Docs](https://automatika-robotics.github.io/kompass/): Extending, custom components, API reference
- [Discord](https://discord.gg/B9ZU6qjzND): Community and support

## Copyright & Contributions

**Kompass** is a collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/).

The code is available under the **MIT License**. See [LICENSE](LICENSE) for details.
Copyright (c) 2024 Automatika Robotics unless explicitly indicated otherwise.
