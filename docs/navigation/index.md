# Navigation Components Overview

Kompass is designed to make autonomous navigation **easy to set up, flexible to customize, and powerful enough for robots working in changing real-world conditions**.
Think of it as a collection of building blocks: you choose the ones you need, configure them for your robot, and you're good to go!

```{figure} ../_static/images/diagrams/system_components_dark.png
:class: only-dark
:alt: Kompass Components and Main Tasks
:align: center

Kompass Components and Main Tasks
```

```{figure} ../_static/images/diagrams/system_components_light.png
:class: only-light
:alt: Kompass Components and Main Tasks
:align: center

Kompass Components and Main Tasks
```

Each component runs as a ROS2 lifecycle node and communicates with the other components using ROS2 topics, services or action servers:



```{figure} ../_static/images/diagrams/system_graph_dark.png
:class: only-dark
:alt: Kompass Full System
:align: center

System Diagram for Point Navigation
```

```{figure} ../_static/images/diagrams/system_graph_light.png
:class: only-light
:alt: Kompass Full System
:align: center

System Diagram for Point Navigation
```

In the following, we give a quick tour of the core navigation components, what each one does, and what sensors you actually need to get Kompass up and running.

---

## Core Components

### [**Global Planner**](path_planning.md)
The Global Planner is responsible for generating an optimal path in point navigation applications from the robot's current position to a specified goal location.
It uses a global map representation and path planning algorithms (e.g., Dijkstra, A* or sampling-based methods) to compute a collision-free route.

**Key Responsibilities:**
- Computes long-range paths across the map.
- Avoids known obstacles using global map data.
- Interfaces with the `Controller` to send planned paths.



### [**Controller**](control.md)
The Controller handles **local motion control** for the robot. It is designed to:
- Perform **path following** for point-to-point navigation.
- Handle **obstacle avoidance** for **unmapped obstacles** and **dynamic objects** that appear in the environment.
- Execute **target following**, allowing the robot to follow moving objects or people.

This makes the controller versatile, supporting both classic navigation tasks and reactive behaviors in dynamic environments.


### [**Local Mapper**](mapper.md)
The Local Mapper builds and maintains a real-time representation of the robot's immediate surroundings.
It integrates sensor data to create local occupancy maps or cost maps used by the Controller for reactive navigation.

**Key Responsibilities:**
- Generates local maps for obstacle detection and avoidance.
- Provides updated environment context to the Controller.



### [**Drive Manager**](driver.md)
The Drive Manager acts as the interface between navigation commands and the robot's actuators.
It handles safe motion execution and integrates emergency stop and safety-check features.

**Key Responsibilities:**
- Translates Controllers velocity commands to the robot.
- Ensures motion safety through emergency stop checks.


### [**Map Server**](map_server.md)
The Map Server is resposible for serving a static global map from a file and making it available to the navigation stack at runtime.

**Key Responsibilities:**
- Convert 2D and 3D maps to OccupancyGrid ROS maps.
- Serve the global map during runtime.
- Save custom maps to files.

---

## Minimum Sensor Requirements

Kompass is designed to be flexible in terms of sensor configurations.
However, at least the following sensors are required for basic autonomous navigation:

- **Odometry Source** (e.g., wheel encoders, IMU or visual odometry)
- **Obstacle Detection Sensor** (e.g., 2D LiDAR **or** Depth Camera)
- **Robot Pose Source** (e.g., localization system such as AMCL or visual SLAM)

These provide the minimal data necessary for localization, mapping, and safe path execution.



## Optional Sensors for Enhanced Features

Additional sensors can enhance navigation capabilities and unlock advanced features:

- **RGB Camera(s)**: Enables vision-based navigation, object tracking, and semantic navigation.
- **Depth Camera**: Improves obstacle avoidance in 3D environments and enables more accurate object tracking.
- **3D LiDAR**: Enhances perception in complex environments with full 3D obstacle detection.
- **GPS**: Enables outdoor navigation and geofenced planning.
- **Ultra-Wideband (UWB) / BLE Beacons**: Improves localization in GPS-denied environments.

---

Kompass supports dynamic configuration, allowing it to operate with minimal sensors and scale up for complex applications when additional sensing is available.
