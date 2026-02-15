# Why Kompass?

Designing robust robot behavior isn't just about perfecting individual components, it's about architecting an integrated system. This is especially true for autonomous navigation, where multiple subsystems must work in harmony to handle complex, dynamic environments and real-world scenarios.

<span class="sd-text-primary" style="font-weight: bold; font-size: 1em;">Getting each part right is important, but what really matters is how everything works together.</span> A capable navigation agent must adapt in real time, deploying different behaviors based on changing conditions. But even in static settings, this remains a nontrivial challenge, as highlighted by years of experience from the [BARN challenge](https://cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge23.html) at ICRA:

```{epigraph}

_"... while it is worthwhile to extend navigation research in directions orthogonal to metric navigation, the community should also not overlook the problems that still remain in this space, especially when robots are expected to be extensively and reliably deployed in the real world."_

-- Lessons learned from The BARN Challenge at ICRA 2022, *[full article](https://www.researchgate.net/publication/362858861_Autonomous_Ground_Navigation_in_Highly_Constrained_Spaces_Lessons_learned_from_The_BARN_Challenge_at_ICRA_2022)*
```

```{epigraph}
_"All teams adopted a hybrid paradigm in terms of a finite-state-machine setup, which requires different components to address different situations in the obstacle courses, ... Such a pragmatic practice suggests that a single stand-alone approach that is able to address all variety of obstacle configurations all together is still out of our reach."_

-- Lessons learned from The 3rd BARN Challenge at ICRA 2024, *[full article](https://arxiv.org/html/2407.01862v1)*
```

**Currently, the only other full-system navigation solution in the open-source community is Nav2, so why did we decide to create Kompass?**

## Adaptive event-driven design to the core

Kompass is designed as per the specification of open event-driven software standard to dynamically respond to real-time changes in the environment, the robots internal state, or the task assigned. Mobile robots working in interaction with messy world dynamics require to adapt on the fly and ensure stable performance in unpredictably changing conditions. Handling external events is built in the core of Kompass so the user can easily design a complete system capable of reconfiguring itself on the fly when an event is perceived in the world or a task is issued to the robot, making it more adaptive and robust.

For example, with Kompass the user can easily design a system that utilizes one planning system when the robot is out on the road or another controller when its inside the building or yet another when its close to its docking station, while easily configuring fallback conditions for each for those components. This approach of providing event driven control over the stack itself, makes it much more flexible to implement a comprehensive autonomous navigating agent which can operate in multiple scenarios. This is in contrast to other approaches like defining _behaviour trees_ where the changes in the stack are based on internal state of the robot alone (and that too with a rather complicated API).


## Engineered for Speed: C++, Multi-Threading, and SYCL GPU Support

Kompass core algorithms are implemented in modern C++ for maximum performance and efficiency. Designed with real-time robotics in mind, it makes full use of multi-threaded CPU execution and GPU acceleration to parallelize compute-heavy tasks like planning, control and map updates.

The GPU support in Kompass is built using SYCL. Unlike other solutions that rely on vendor-specific GPU APIs (e.g. CUDA for Nvidia), Kompass is the <span class="text-red-strong" style="text-align: center;">first navigation framework to support cross-GPU acceleration</span>. This means it can target any SYCL-compliant GPU, including those from Nvidia, AMD, Intel, and others—without requiring device-specific modifications.

While the performance-critical core runs in C++, Kompass offers a clean Python API combining the speed of native code with the ease of Python development.

## Machine learning models as first class citizens

The event driven stack control allows Kompass to utilize machine learning models unlike any other navigation stack. External events in Kompass can be driven by outputs of machine learning models interpreting sensor data or user commands, which means the entire stack becomes reconfigurable based on ML model outputs. This goes beyond well established scenarios of visual navigation.

As an example consider the scenario where the robot observes a certain number of humans in its environment and switches from a path following controller like _Pure Pursuit_ to a predictive controller like _Timed Elastic Bands_ or even better a human-aware controller like _Human Aware TEB_. The same goes for utilizing outputs of VLMs, which can answer more abstract aggregate perception questions like 'Whether the robot is inside or outside?', to change control behaviour on the fly.

To get more ideas about utilizing machine learning models with Kompass and create intelligent embodied agents, check out the example tutorial on using Kompass vision follower with **EmbodiedAgents**' Vision Component [here](./tutorials/vision_tracking.md).


## Ease of use and intuitive API while remaining within the ROS ecosystem

Kompass provides an approachable and intuitive interface for creating navigation systems. This is made possible with a pythonic API using [Sugarcoat🍬](https://www.github.com/automatika-robotics/sugarcoat). A fairly sophisticated navigation system can be configured in one simple python script where the user can configure the stack component by component and configure the system level behavior by defining all the required events, their consequent actions and per component fallback behaviors. The user also has a choice to provide component level parameters in YAML files. Furthermore, the project is structured so that core algorithms are implemented in the Kompass Core package, which provides a pure python interface while the underlying algorithms can be implemented in both python for quick prototyping and  in C/C++ for performance optimization in production environments.


## Modular architecture and easy extensibility

The architecture of Kompass to separate the core algorithms in Kompass Core package, away from its ROS primitives also serves to simplify upgrades and minimizes the risk of breaking changes between different ROS versions and distributions. It also allows for seamless integration of additional planning and control algorithms, as well as managing integration with specialized third party libraries. Most importantly, Kompass has been designed to be extendable by the community as a unified place to contribute additional plan and control algorithms that can be useful for mobile robots.
