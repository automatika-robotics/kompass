# {py:mod}`kompass.callbacks`

```{py:module} kompass.callbacks
```

```{autodoc2-docstring} kompass.callbacks
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`OdomCallback <kompass.callbacks.OdomCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.OdomCallback
    :summary:
    ```
* - {py:obj}`PointCallback <kompass.callbacks.PointCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.PointCallback
    :summary:
    ```
* - {py:obj}`PointStampedCallback <kompass.callbacks.PointStampedCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.PointStampedCallback
    :summary:
    ```
* - {py:obj}`PoseCallback <kompass.callbacks.PoseCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.PoseCallback
    :summary:
    ```
* - {py:obj}`PoseStampedCallback <kompass.callbacks.PoseStampedCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.PoseStampedCallback
    :summary:
    ```
* - {py:obj}`LaserScanCallback <kompass.callbacks.LaserScanCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.LaserScanCallback
    :summary:
    ```
* - {py:obj}`PointCloudCallback <kompass.callbacks.PointCloudCallback>`
  - ```{autodoc2-docstring} kompass.callbacks.PointCloudCallback
    :summary:
    ```
````

### API

`````{py:class} OdomCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.OdomCallback

Bases: {py:obj}`ros_sugar.io.OdomCallback`

```{autodoc2-docstring} kompass.callbacks.OdomCallback
```

````{py:property} get_front
:canonical: kompass.callbacks.OdomCallback.get_front
:type: typing.Optional[bool]

```{autodoc2-docstring} kompass.callbacks.OdomCallback.get_front
```

````

`````

````{py:class} PointCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointCallback

Bases: {py:obj}`ros_sugar.io.PointCallback`

```{autodoc2-docstring} kompass.callbacks.PointCallback
```

````

````{py:class} PointStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointStampedCallback

Bases: {py:obj}`kompass.callbacks.PointCallback`

```{autodoc2-docstring} kompass.callbacks.PointStampedCallback
```

````

````{py:class} PoseCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PoseCallback

Bases: {py:obj}`ros_sugar.io.PoseCallback`

```{autodoc2-docstring} kompass.callbacks.PoseCallback
```

````

````{py:class} PoseStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PoseStampedCallback

Bases: {py:obj}`kompass.callbacks.PoseCallback`

```{autodoc2-docstring} kompass.callbacks.PoseStampedCallback
```

````

`````{py:class} LaserScanCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None)
:canonical: kompass.callbacks.LaserScanCallback

Bases: {py:obj}`ros_sugar.io.GenericCallback`

```{autodoc2-docstring} kompass.callbacks.LaserScanCallback
```

````{py:property} transformation
:canonical: kompass.callbacks.LaserScanCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} kompass.callbacks.LaserScanCallback.transformation
```

````

`````

`````{py:class} PointCloudCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None, max_range: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointCloudCallback

Bases: {py:obj}`ros_sugar.io.GenericCallback`

```{autodoc2-docstring} kompass.callbacks.PointCloudCallback
```

````{py:property} max_range
:canonical: kompass.callbacks.PointCloudCallback.max_range
:type: typing.Optional[float]

```{autodoc2-docstring} kompass.callbacks.PointCloudCallback.max_range
```

````

````{py:property} transformation
:canonical: kompass.callbacks.PointCloudCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} kompass.callbacks.PointCloudCallback.transformation
```

````

`````
