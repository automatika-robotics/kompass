# Local Mapper

**Real-time, ego-centric occupancy grid generation.**

While the global map provides a static long-term view, the **Local Mapper** builds a dynamic, short-term map of the robot's immediate surroundings based on real-time sensor data.

It captures moving obstacles (people, other robots) and temporary changes, serving as the primary input for the [Controller](control.md) to enable fast reactive navigation.


## Core Algorithm

At its core, LocalMapper uses the Bresenham line drawing algorithm in C++ to efficiently update an occupancy grid from incoming LaserScan data. This approach ensures fast and accurate raycasting to determine free and occupied cells in the local grid.

To maximize performance and adaptability, the implementation **supports both CPU and GPU execution**:

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">SYCL GPU Acceleration - </span> GPU acceleration is implemented using SYCL, making it vendor-agnostic—compatible with Nvidia, AMD, Intel, and any other GPGPU-capable devices.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">Multi-Threaded CPU - </span>, If no GPU is available, it automatically falls back to a highly optimized multi-threaded CPU implementation capable of handling dense scan rates or high-frequency updates.

:::{admonition} Supported Data
:class: note
Currently supports `LaserScan` and `PointCloud2` data to create 2D Occupancy Grids. Support for **3D** local maps and semantic layers is planned for an upcoming releases.
:::


## Interface

### Inputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">sensor_data</span>
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html), `std_msgs.msg.Float64`, [`sensor_msgs.msg.PointCloud2`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1
  - `/scan` (`LaserScan`)

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">location</span>
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `/odom` (`Odometry`)

```

## Outputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">local_map</span>
  - `nav_msgs.msg.OccupancyGrid`
  - 1
  - `/local_map/occupancy_layer`
```


## Usage Example

```python
from kompass_core.mapping import LocalMapperConfig, MapperConfig
from kompass.components import LocalMapper

# 1. Define Map Dimensions
# Create a 5m x 5m rolling window around the robot with 20cm resolution
map_params = MapperConfig(
    width=5.0,
    height=5.0,
    resolution=0.2
)

# 2. Configure Component
my_config = LocalMapperConfig(
    loop_rate=10.0,       # Update at 10Hz
    map_params=map_params
)

# 3. Instantiate
my_mapper = LocalMapper(component_name="mapper", config=my_config)

```

## See Next

The Local Mapper feeds directly into the Controller for obstacle avoidance, while the Map Server provides the global context.

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} {material-regular}`gamepad;1.5em;sd-text-primary` Controller
:link: control
:link-type: doc
:class-card: sugar-card

**Real-Time Control**
Learn how to use the generated local map for dynamic obstacle avoidance.
:::

:::{grid-item-card} {material-regular}`map;1.5em;sd-text-primary` Map Server
:link: map_server
:link-type: doc
:class-card: sugar-card

**Global Context**
Learn how to manage and serve the static global map.
:::
::::
