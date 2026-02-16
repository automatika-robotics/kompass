# Map Server

**Static global map management and 3D-to-2D projection.**

The [MapServer](../apidocs/kompass/kompass.components.map_server.md) is the source of ground-truth for the navigation system. It reads static map files, processes them, and publishes the global `OccupancyGrid` required by the Planner and Localization components.

Unlike standard ROS2 map servers, the Kompass Map Server supports **native 3D Point Cloud (PCD)** files, automatically slicing and projecting them into 2D navigable grids based on configurable height limits.

## Key Features

The Map Server handles the lifecycle of map data from disk to network.


- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`layers;1.2em;sd-text-primary` Multi-Format Support - </span> **YAML & PCD**. Seamlessly reads standard 2D map files (`.yaml` + image) OR 3D point cloud files (`.pcd`).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`save;1.2em;sd-text-primary` Map Persistence - </span> **Save Services**. Supports saving current 2D or 3D map data to disk via `Save2dMapToFile` and `Save3dMapToFile` services.


- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`crop_free;1.2em;sd-text-primary` Auto Frame Handling - </span> **TF Compatibility**. Configurable reference frames ensuring the map aligns correctly with your robot's specific TF tree.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`graphic_eq;1.2em;sd-text-primary` Frequency Control - </span> The MapServer can be configured to control how often map data is read and converted. The rate of map updates can be controlled by the `map_file_read_rate` parameter, ensuring that map data is refreshed periodically or only when necessary.

```{seealso}
Check the full configuration parameters of the MapServer in the [MapServerConfig](../apidocs/kompass/kompass.components.map_server.md/#classes)
```

## Interface

### Outputs

The Map Server publishes the processed grid and optionally the raw cloud for visualization.


```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">map</span>
  - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
  - 1
  - `/map`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">sensor_data</span>
  - [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1, optional
  - `/row_point_cloud`
```


## Usage Example

```python
from kompass.components import MapServer, MapServerConfig

# 1. Configuration
my_config = MapServerConfig(
    # Path to a 3D Point Cloud file
    map_file_path="/path/to/environment.pcd",

    # Process at 5Hz (only needed if map changes or for initial load)
    map_file_read_rate=5.0,

    # Resolution for the generated 2D grid (meters/cell)
    grid_resolution=0.1,

    # Disable raw cloud publishing to save bandwidth
    pc_publish_row=False
)

# 2. Instantiate
my_map_server = MapServer(component_name="map_server", config=my_config)

```

## See Next

Once the map is served, the Planner uses it to calculate global paths.

:::{button-link} path_planning.html
:color: primary
:ref-type: doc
:outline:
Configure the Planner →
:::
