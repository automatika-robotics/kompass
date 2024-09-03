# {py:mod}`Kompass.data_types`

```{py:module} Kompass.data_types
```

```{autodoc2-docstring} Kompass.data_types
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`LaserScan <Kompass.data_types.LaserScan>`
  - ```{autodoc2-docstring} Kompass.data_types.LaserScan
    :summary:
    ```
* - {py:obj}`PointCloud2 <Kompass.data_types.PointCloud2>`
  -
* - {py:obj}`PointStamped <Kompass.data_types.PointStamped>`
  - ```{autodoc2-docstring} Kompass.data_types.PointStamped
    :summary:
    ```
* - {py:obj}`Point <Kompass.data_types.Point>`
  - ```{autodoc2-docstring} Kompass.data_types.Point
    :summary:
    ```
* - {py:obj}`Pose <Kompass.data_types.Pose>`
  - ```{autodoc2-docstring} Kompass.data_types.Pose
    :summary:
    ```
* - {py:obj}`PoseStamped <Kompass.data_types.PoseStamped>`
  - ```{autodoc2-docstring} Kompass.data_types.PoseStamped
    :summary:
    ```
* - {py:obj}`Odometry <Kompass.data_types.Odometry>`
  - ```{autodoc2-docstring} Kompass.data_types.Odometry
    :summary:
    ```
* - {py:obj}`TwistArray <Kompass.data_types.TwistArray>`
  - ```{autodoc2-docstring} Kompass.data_types.TwistArray
    :summary:
    ```
* - {py:obj}`Path <Kompass.data_types.Path>`
  - ```{autodoc2-docstring} Kompass.data_types.Path
    :summary:
    ```
````

### API

`````{py:class} LaserScan
:canonical: Kompass.data_types.LaserScan

Bases: {py:obj}`auto_ros.supported_types.LaserScan`

```{autodoc2-docstring} Kompass.data_types.LaserScan
```

````{py:method} convert(output: kompass_core.datatypes.laserscan.LaserScanData, frame_id: typing.Optional[str] = 'map', time_sec: typing.Optional[int] = 0, time_nanosec: typing.Optional[int] = 0, **_) -> sensor_msgs.msg.LaserScan
:canonical: Kompass.data_types.LaserScan.convert
:classmethod:

```{autodoc2-docstring} Kompass.data_types.LaserScan.convert
```

````

`````

`````{py:class} PointCloud2
:canonical: Kompass.data_types.PointCloud2

Bases: {py:obj}`auto_ros.supported_types.SupportedType`

````{py:method} convert(output, **__) -> typing.Any
:canonical: Kompass.data_types.PointCloud2.convert
:classmethod:

````

`````

`````{py:class} PointStamped
:canonical: Kompass.data_types.PointStamped

Bases: {py:obj}`auto_ros.supported_types.PointStamped`

```{autodoc2-docstring} Kompass.data_types.PointStamped
```

````{py:method} convert(output, **__) -> typing.Any
:canonical: Kompass.data_types.PointStamped.convert
:classmethod:

````

`````

`````{py:class} Point
:canonical: Kompass.data_types.Point

Bases: {py:obj}`auto_ros.supported_types.Point`

```{autodoc2-docstring} Kompass.data_types.Point
```

````{py:method} convert(output: typing.Union[numpy.ndarray, Navigation.models.RobotState], **_) -> geometry_msgs.msg.Point
:canonical: Kompass.data_types.Point.convert
:classmethod:

````

`````

`````{py:class} Pose
:canonical: Kompass.data_types.Pose

Bases: {py:obj}`auto_ros.supported_types.Pose`

```{autodoc2-docstring} Kompass.data_types.Pose
```

````{py:method} convert(output, **__) -> typing.Any
:canonical: Kompass.data_types.Pose.convert
:classmethod:

````

`````

`````{py:class} PoseStamped
:canonical: Kompass.data_types.PoseStamped

Bases: {py:obj}`auto_ros.supported_types.PoseStamped`

```{autodoc2-docstring} Kompass.data_types.PoseStamped
```

````{py:method} convert(output, **__) -> typing.Any
:canonical: Kompass.data_types.PoseStamped.convert
:classmethod:

````

`````

`````{py:class} Odometry
:canonical: Kompass.data_types.Odometry

Bases: {py:obj}`auto_ros.supported_types.Odometry`

```{autodoc2-docstring} Kompass.data_types.Odometry
```

````{py:method} convert(output: kompass_core.models.RobotState, frame_id: typing.Optional[str] = 'map', time_sec: typing.Optional[int] = 0, time_nanosec: typing.Optional[int] = 0, **_) -> nav_msgs.msg.Odometry
:canonical: Kompass.data_types.Odometry.convert
:classmethod:

```{autodoc2-docstring} Kompass.data_types.Odometry.convert
```

````

`````

`````{py:class} TwistArray
:canonical: Kompass.data_types.TwistArray

Bases: {py:obj}`auto_ros.supported_types.SupportedType`

```{autodoc2-docstring} Kompass.data_types.TwistArray
```

````{py:method} convert(output: kompass_interfaces.msg.TwistArray, **_)
:canonical: Kompass.data_types.TwistArray.convert
:classmethod:

```{autodoc2-docstring} Kompass.data_types.TwistArray.convert
```

````

`````

`````{py:class} Path
:canonical: Kompass.data_types.Path

Bases: {py:obj}`auto_ros.supported_types.Path`

```{autodoc2-docstring} Kompass.data_types.Path
```

````{py:method} to_json(path: nav_msgs.msg.Path, json_file: str)
:canonical: Kompass.data_types.Path.to_json
:classmethod:

```{autodoc2-docstring} Kompass.data_types.Path.to_json
```

````

````{py:method} from_json(json_file: str) -> typing.Union[nav_msgs.msg.Path, None]
:canonical: Kompass.data_types.Path.from_json
:classmethod:

```{autodoc2-docstring} Kompass.data_types.Path.from_json
```

````

````{py:method} convert(output, **__) -> typing.Any
:canonical: Kompass.data_types.Path.convert
:classmethod:

````

`````
