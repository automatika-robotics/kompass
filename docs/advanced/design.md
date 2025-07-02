# 🧩 Design Concepts

Kompass is built on top of [**Sugarcoat🍬**](https://automatika-robotics.github.io/sugarcoat), a lightweight and expressive framework for building modular, reactive systems in ROS 2. At the heart of this architecture is the **Component**, which is a super sweetened version of ROS2 lifecycle nodes, acting as the main execution unit of logic within Kompass.

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

## Easily Configurable Inputs and Outputs
 Components communicate through defined [**Inputs/Outputs**](https://automatika-robotics.github.io/sugarcoat/design/topics.html), allowing you to easily specify the ROS2 topics that connect different parts of your system seamlessly. While Sugarcoat🍬 components are generic and can be configured with any type of input or output, Inputs/Outputs in Kompass Components are defined with fixed **key names** across the stack, each containing:
 - Set of allowed types for the stream (equivalent to ROS2 messages)
 - The number of required streams for the key name
 - The maximum number of additional streams that can be assigned.


```{seealso}
See a complete list of the Inputs/Outputs keys in Kompass stack along with configuration examples [here](./advanced_conf/topics.md)
```

## Robust with Health Status Self-Monitoring & Fallbacks
Each component continuously maintains and updates a [**Health Status**](https://automatika-robotics.github.io/sugarcoat/design/status.html), providing runtime introspection for debugging and fault detection. Each component periodically publishes this to its respective `status` topic. This self-monitoring allows the system to assess component health in real time and take appropriate actions.

Components can also be configured with [**Fallback behaviors**](https://automatika-robotics.github.io/sugarcoat/design/fallbacks.html) that trigger user defined `Actions` automatically when failures occur. This mechanism enhances system resiliency by enabling on-line automatic recovery without manual intervention.

Kompass defines the following health status levels to categorize component states:

| Status Code | Description              |
|-------------|--------------------------|
| 0           | Running - Healthy        |
| 1           | Failure: Algorithm Level |
| 2           | Failure: Component Level |
| 3           | Failure: System Level    |
| 4           | Failure: General         |

These health codes help diagnose issues at different layers—from algorithmic faults to systemic problems—allowing fine-grained fault handling and robust navigation behavior.


## Runtime Control using Events and Actions

Kompass leverages [**Events**](https://automatika-robotics.github.io/sugarcoat/design/events.html) and [**Actions**](https://automatika-robotics.github.io/sugarcoat/design/actions.html) to enable dynamic, context-aware behavior orchestration during runtime. This design allows components to be reconfigured or triggered reactively in response to changing conditions, making the navigation system highly adaptable and robust.

Events serve as runtime signals that alert the robot software stack to dynamic changes. An Event is defined by a change in the value of a ROS2 message on a specific topic, representing a meaningful state or environmental update. Events are matched with corresponding Actions, which are executed once the Event occurs, allowing the system to react promptly and appropriately.

Kompass includes a variety of pre-defined Event types to help your system respond precisely when needed. For instance, `OnEqual` event activates when a message value matches a specified target. If you need to track changes into account, `OnChange` fires whenever a value changes, and ``OnChangeEqual triggers when it changes to a particular value. Threshold-based events like OnGreater and OnLess allow you to react when values cross set limits. See more on available events in [Sugarcoat🍬 docs](https://automatika-robotics.github.io/sugarcoat/design/events.html).

Actions, on the other hand, are routines or methods executed either by individual components or by the system monitor to respond to events (or failures). They can be paired with Events to execute the Action upon event detection. [More on Actions](https://automatika-robotics.github.io/sugarcoat/design/actions.html)

This flexible Event-Action framework empowers Kompass components with reactive capabilities essential for robust, adaptive autonomous navigation.


## Launched with Performance in Mind
The system is executed using the [**Launcher**](https://automatika-robotics.github.io/sugarcoat/design/launcher.html), which supports both multi-threaded and multi-process execution. It handles the orchestration of components, scheduling, and concurrent operation. An internal [**Monitor**](https://automatika-robotics.github.io/sugarcoat/design/monitor.html) continuously tracks the state of components and the events being triggered—making the system self-aware and adaptable in real-time.


```{seealso}
Dive deeper into each of these architectural elements in [Sugarcoat🍬 documentation](https://automatika-robotics.github.io/sugarcoat)
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
