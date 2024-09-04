<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/Kompass.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/Kompass_dark.png">
  <img alt="Kompass Logo." src="docs/_static/Kompass_dark.png"  width="50%">
</picture>

Kompass is an event-driven navigation system designed with an easy-to-use and intuitive Python API. Kompass is built to be customizable, extendable and hardware-agnostic. It aims to implement the most cutting edge algorithms for all parts of the navigation stack. And most importantly, it allows users to create very sophisticated navigation capabilities for autonomous mobile robots, within a single python script.

- Find out more about our [**motivation**](docs/why.md) to create Kompass ✨
- [**Install**](docs/install.md) Kompass on your robot 🛠️
- To get started with Kompass, check the [**quick start**](docs/quick_start.md) tutorial 🚀
- Do a deep dive into Kompass [**components**](docs/navigation/index.md) 🤖
- Learn more about the [**design concepts**](docs/design/index.md) of Kompass 📚

> [!NOTE]
> This is an alpha release of Kompass. Breaking changes are to be expected.

Kompass is divided into several interacting components each responsible for one of the navigation subtasks:

![Kompass Components Tasks](docs/_static/images/diagrams/system_components.jpg)

Each of the previous components runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:

![Kompass Full System](docs/_static/images/diagrams/system_graph.jpg)

To learn more about the functionalities and configuration of each component check the component dedicated documentation page:

- [Planner](docs/navigation/path_planning.md)
- [Controller](docs/navigation/control.md)
- [Drive Manager](docs/navigation/driver.md)
- [Motion Server](docs/navigation/motion_server.md)

## Copyright

The code in this distribution is Copyright (c) 2024 Automatika Robotics unless explicitly indicated otherwise.

Kompass is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.

## Contributions

Kompass has been developed in collaboration betweeen [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.
