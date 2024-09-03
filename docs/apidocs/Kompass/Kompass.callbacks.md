# {py:mod}`Kompass.callbacks`

```{py:module} Kompass.callbacks
```

```{autodoc2-docstring} Kompass.callbacks
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`OdomCallback <Kompass.callbacks.OdomCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.OdomCallback
    :summary:
    ```
* - {py:obj}`PointCallback <Kompass.callbacks.PointCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.PointCallback
    :summary:
    ```
* - {py:obj}`PointStampedCallback <Kompass.callbacks.PointStampedCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.PointStampedCallback
    :summary:
    ```
* - {py:obj}`PoseCallback <Kompass.callbacks.PoseCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.PoseCallback
    :summary:
    ```
* - {py:obj}`PoseStampedCallback <Kompass.callbacks.PoseStampedCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.PoseStampedCallback
    :summary:
    ```
* - {py:obj}`LaserScanCallback <Kompass.callbacks.LaserScanCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.LaserScanCallback
    :summary:
    ```
* - {py:obj}`PointCloudCallback <Kompass.callbacks.PointCloudCallback>`
  - ```{autodoc2-docstring} Kompass.callbacks.PointCloudCallback
    :summary:
    ```
````

### API

`````{py:class} OdomCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: Kompass.callbacks.OdomCallback

Bases: {py:obj}`auto_ros.callbacks.OdomCallback`

```{autodoc2-docstring} Kompass.callbacks.OdomCallback
```

````{py:property} get_front
:canonical: Kompass.callbacks.OdomCallback.get_front
:type: typing.Optional[bool]

```{autodoc2-docstring} Kompass.callbacks.OdomCallback.get_front
```

````

`````

`````{py:class} PointCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: Kompass.callbacks.PointCallback

Bases: {py:obj}`auto_ros.callbacks.PointCallback`

```{autodoc2-docstring} Kompass.callbacks.PointCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.PointCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.PointCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.PointCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.PointCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.PointCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.PointCallback.got_msg

````

`````

`````{py:class} PointStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: Kompass.callbacks.PointStampedCallback

Bases: {py:obj}`Kompass.callbacks.PointCallback`

```{autodoc2-docstring} Kompass.callbacks.PointStampedCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.PointStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.PointStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.PointStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.PointStampedCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.PointStampedCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.PointStampedCallback.got_msg

````

`````

`````{py:class} PoseCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: Kompass.callbacks.PoseCallback

Bases: {py:obj}`auto_ros.callbacks.PoseCallback`

```{autodoc2-docstring} Kompass.callbacks.PoseCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.PoseCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.PoseCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.PoseCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.PoseCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.PoseCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.PoseCallback.got_msg

````

`````

`````{py:class} PoseStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: Kompass.callbacks.PoseStampedCallback

Bases: {py:obj}`Kompass.callbacks.PoseCallback`

```{autodoc2-docstring} Kompass.callbacks.PoseStampedCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.PoseStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.PoseStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.PoseStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.PoseStampedCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.PoseStampedCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.PoseStampedCallback.got_msg

````

`````

`````{py:class} LaserScanCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None)
:canonical: Kompass.callbacks.LaserScanCallback

Bases: {py:obj}`auto_ros.callbacks.GenericCallback`

```{autodoc2-docstring} Kompass.callbacks.LaserScanCallback
```

````{py:property} transformation
:canonical: Kompass.callbacks.LaserScanCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} Kompass.callbacks.LaserScanCallback.transformation
```

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.LaserScanCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.LaserScanCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.LaserScanCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.LaserScanCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.LaserScanCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.LaserScanCallback.got_msg

````

`````

`````{py:class} PointCloudCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None, max_range: typing.Optional[float] = None)
:canonical: Kompass.callbacks.PointCloudCallback

Bases: {py:obj}`auto_ros.callbacks.GenericCallback`

```{autodoc2-docstring} Kompass.callbacks.PointCloudCallback
```

````{py:property} max_range
:canonical: Kompass.callbacks.PointCloudCallback.max_range
:type: typing.Optional[float]

```{autodoc2-docstring} Kompass.callbacks.PointCloudCallback.max_range
```

````

````{py:property} transformation
:canonical: Kompass.callbacks.PointCloudCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} Kompass.callbacks.PointCloudCallback.transformation
```

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: Kompass.callbacks.PointCloudCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: Kompass.callbacks.PointCloudCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: Kompass.callbacks.PointCloudCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: Kompass.callbacks.PointCloudCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: Kompass.callbacks.PointCloudCallback.get_output

````

````{py:property} got_msg
:canonical: Kompass.callbacks.PointCloudCallback.got_msg

````

`````
