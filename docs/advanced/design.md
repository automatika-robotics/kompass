# 🧩 Design Concepts

Kompass is built on [**Sugarcoat🍬**](https://automatika-robotics.github.io/sugarcoat), inheriting a lightweight, expressive, and event-driven architecture for designing ROS2-based systems.

At the core of Kompass is the **Component**—a "super-sweetened" version of the standard ROS2 lifecycle node. By standardizing execution, health monitoring, and data flow, Kompass allows you to build navigation stacks that are not just modular, but **reactive** and **self-healing**.


The following sections outline the five pillars of the Kompass design.

## 1. The Component: Smart Execution unit

The Component is the atomic unit of logic in Kompass. Unlike a standard node, a Component manages its own lifecycle, validates its own configuration, and reports its own health.

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

## 2. Standardized Inputs & Outputs

Components communicate through defined [**Inputs/Outputs**](https://automatika-robotics.github.io/sugarcoat/design/topics.html), allowing you to easily specify the ROS2 topics that connect different parts of your system.

To ensure that different navigation components (planners, controllers, mappers) snap together effortlessly, Kompass communicates through **Standardized I/O Keys**. Crucially, each key supports **multiple message types**, providing extra flexibility and out-of-the-box configuration for your data pipelines.

Each Input/Output key is governed by:

- **Allowed Types:** Ensures data compatibility by strictly defining which message classes are accepted (e.g., a map input only accepting `OccupancyGrid`, or a position input can accept both 'Pose' and `PoseStamped` messages).
- **Cardinality:** Defines the topology of the connection—specifically, **if a stream is mandatory** and **how many sources are allowed** (e.g., "Requires exactly 1 Odometry source" or "Accepts up to 3 PointCloud sources").


```{seealso}
See a complete list of the Inputs/Outputs keys in Kompass stack along with configuration examples [here](./advanced_conf/topics.md)
```

## 3. Active Resilience: Health & Fallbacks

Robots operate in unpredictable environments. Kompass components are designed to handle failures gracefully rather than crashing the whole stack.

### Self-Monitoring (Health Status)
Every component continuously introspects its state and broadcasts a [**Health Status**](https://automatika-robotics.github.io/sugarcoat/design/status.html). This allows the system to distinguish between a crashing driver and a path planning algorithm that simply can't find a route.

The following health codes help diagnose issues at different layers—from algorithmic faults to systemic problems—allowing fine-grained fault handling and robust navigation behavior:

| Status Code | Description              |
|-------------|--------------------------|
| 0           | Running - Healthy        |
| 1           | Failure: Algorithm Level |
| 2           | Failure: Component Level |
| 3           | Failure: System Level    |
| 4           | Failure: General         |

### Automatic Recovery (Fallbacks)
Why wake up a human when the robot can fix itself? Components are configured with [**Fallback**](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html) behaviors. When a specific Health Status level is reported, the component automatically triggers user-defined Actions—such as re-initializing a driver, clearing a costmap, or switching to a recovery behavior—without manual intervention.


## 4. Dynamic Orchestration: Events & Actions

Static navigation stacks are brittle. Kompass uses an Event-Driven Architecture to adapt to changing contexts in real-time.

- [**Events**](https://automatika-robotics.github.io/sugarcoat/design/events.html) (Triggers): These monitor data streams for specific conditions. For example, `OnEqual` (Battery == Low), `OnChange` (Terrain changed), `OnGreater` (Velocity > Limit).

- [**Actions**](https://automatika-robotics.github.io/sugarcoat/design/actions.html) (Responses): Routines executed when an Event fires (or a Fallback is triggered). for example you can execute a an action to switch the control algorithm or to stop the robot.

This decouples the "monitoring" logic from the "execution" logic, allowing you to compose complex behaviors (e.g., "If Terrain changes to Stairs, trigger SwitchGait action") purely through configuration.


## 5. Flexible Execution with Performance in Mind

Kompass respects that different robots have different compute constraints. The [**Launcher**](https://automatika-robotics.github.io/sugarcoat/design/launcher.html) allows you to orchestrate your entire stack with a simple Python API, choosing the execution model that fits your hardware:

- Multi-threaded: Components run as threads in a single process. Ideal for low-latency communication via shared memory.

- Multi-process: Components run in isolated processes. Ideal for stability (one crash doesn't kill the system) and distributing load.

An internal [**Monitor**](https://automatika-robotics.github.io/sugarcoat/design/monitor.html) runs alongside the stack, continuously supervising the state of components and the events being triggered—making the system self-aware and adaptable in real-time.

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

```{seealso}
Dive deeper into each of these architectural elements in [Sugarcoat🍬 documentation](https://automatika-robotics.github.io/sugarcoat)
```
