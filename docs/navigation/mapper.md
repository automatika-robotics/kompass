# Local Mapper

A global map refers to a reference map containing the static space information. A local map, on the other hand, is a smaller ego-centric map around the robot that reflects the space dynamics using realtime sensor information. Local maps can reflect information fused from multiple sensors, providing a more stable, robust and complete source of information to the [Controller](control.md) rather than getting the direct sensor information.

[LocalMapper](../apidocs/kompass/kompass.components.mapper.md) component is responsible for generating this local map during the navigation.


```{note}
Current implementation supports LaserScan sensor data to create an Occupancy Grid local map. PointCloud and semantic information will be supported in an upcoming release
```


## Available Run Types
Set from ControllerConfig class or directly from Controller 'run_type' property.

```{list-table}
:widths: 10 80
* - **Timed**
  - Produces a local map periodically if all inputs are available
```

## Inputs

```{list-table}
::widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - sensor_data
  - [`sensor_msgs.msg.LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)
  - 1
  - ```Topic(name="/scan", msg_type="LaserScan")```

* - location
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - ```Topic(name="/odom", msg_type="Odometry")```

```

## Outputs

```{list-table}
::widths: 10 40 10 40
:header-rows: 1
* - Key Name
  - Allowed Types
  - Number
  - Default

* - local_map
  - `nav_msgs.msg.OccupancyGrid`
  - 1
  - ```Topic(name="/local_map/occupancy_layer", msg_type="OccupancyGrid")```
```


## Usage Example:
```python
    from kompass_core.mapping import LocalMapperConfig
    from kompass.components import LocalMapper, MapperConfig

    # Select map parameters
    map_params = MapperConfig(width=5.0, height=5.0, resolution=0.2) # 5mX5m map with 0.2m/cell resolution

    # Setup custom component configuration
    my_config = LocalMapperConfig(loop_rate=10.0, map_params=map_params)

    # Init a mapper
    my_mapper = LocalMapper(component_name="mapper", config=my_config)
```
