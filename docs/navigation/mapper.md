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
:widths: 10 30 15 20 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Number of Topics
  - Default Value

* - **location**
  - Robot current location
  - `Odometry, PoseWithCovariance, Pose`
  - 1
  - `Topic(name="/odom", msg_type="Odometry")`

* - **sensor_data**
  - Direct sensor input
  - `LaserScan`
  - 1
  - `Topic(name="/scan", msg_type="LaserScan")`

```

## Outputs

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Default Value

* - **local_map**
  - Local occupancy map
  - `OccupancyGrid`
  - `Topic(name="/scan", msg_type="LaserScan")`
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
