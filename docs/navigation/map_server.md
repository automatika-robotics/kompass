# Map Server

The [MapServer](../apidocs/kompass/kompass.components.map_server.md) component is responsible for serving the static global map data within the navigation system. It reads static map files, processes them, and serves global map data to other components in the system.

The MapServer component can perform the following functionalities:

```{list-table}
:widths: 20 70

* - **Map Data Conversion**
  - Reads map files in either 2D (YAML) or 3D (PCD) format and converts the data into usable global map formats (OccupancyGrid).

* - **Global Map Serving**
  - Once map data is loaded and processed, the MapServer publishes the global map as an `OccupancyGrid` message. This map is continuously available for other components to access for tasks like path planning, localization, and obstacle detection.

* - **Point Cloud to Grid Conversion**
  - If the map data is provided in the form of point cloud data (PCD file), the MapServer can generate an occupancy grid from the point cloud. It uses the provided grid resolution and ground limits to classify points and create an accurate occupancy map.

* - **Custom Frame Handling**
  - The MapServer allows configuration of a custom frame for the map. This frame can be applied to the global map when published, ensuring compatibility with other components that may use a different reference frame for their operations.

* - **Map Saving**
  - The MapServer can save the generated or modified maps to files. It supports saving both 2D and 3D maps using the `Save2dMapToFile` and `Save3dMapToFile` services. This allows map data to be preserved for later use or shared between different systems.

* - **Map Conversing Frequency Control**
  - The MapServer can be configured to control how often map data is read and converted. The rate of map updates can be controlled by the `map_file_read_rate` parameter, ensuring that map data is refreshed periodically or only when necessary.

```


## Outputs:

```{list-table}
:widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - **global_map**
  - [`nav_msgs.msg.OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)
  - 1
  - `Topic(name="/map", msg_type="OccupancyGrid")` - Global map generated from input data.

* - **spatial_sensor**
  - [`sensor_msgs.msg.PointCloud2`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)
  - 1, optional
  - `Topic(name="/row_point_cloud", msg_type="PointCloud2")` - Row point cloud data for visualization or further processing.
```

## Configuration Parameters:

See all available parameters in [MapServerConfig](../apidocs/kompass/kompass.components.map_server.md/#classes)

## Usage Example:
```python
    from kompass.components import MapServerConfig, MapServer
    from kompass.ros import Topic

    # Setup custom configuration
    my_config = MapServerConfig(
            map_file_read_rate=5.0,
            map_file_path="/path/to/your/map.pcd",      # Absolute path to the static map file
            grid_resolution=0.1,                        # Map resolution used for converting from 3D
            pc_publish_row=False,                       # Disable publishing point cloud data
        )

    # Init a MapServer object
    my_map_server = MapServer(component_name="map_server", config=my_config)
```
