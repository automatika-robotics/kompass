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

Bases: {py:obj}`ros_sugar.callbacks.OdomCallback`

```{autodoc2-docstring} kompass.callbacks.OdomCallback
```

````{py:property} get_front
:canonical: kompass.callbacks.OdomCallback.get_front
:type: typing.Optional[bool]

```{autodoc2-docstring} kompass.callbacks.OdomCallback.get_front
```

````

`````

`````{py:class} PointCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointCallback

Bases: {py:obj}`ros_sugar.callbacks.PointCallback`

```{autodoc2-docstring} kompass.callbacks.PointCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.PointCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.PointCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.PointCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.PointCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.PointCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.PointCallback.got_msg

````

`````

`````{py:class} PointStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointStampedCallback

Bases: {py:obj}`kompass.callbacks.PointCallback`

```{autodoc2-docstring} kompass.callbacks.PointStampedCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.PointStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.PointStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.PointStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.PointStampedCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.PointStampedCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.PointStampedCallback.got_msg

````

`````

`````{py:class} PoseCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PoseCallback

Bases: {py:obj}`ros_sugar.callbacks.PoseCallback`

```{autodoc2-docstring} kompass.callbacks.PoseCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.PoseCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.PoseCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.PoseCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.PoseCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.PoseCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.PoseCallback.got_msg

````

`````

`````{py:class} PoseStampedCallback(input_topic, node_name: typing.Optional[str] = None, get_front: typing.Optional[bool] = False, robot_radius: typing.Optional[float] = None)
:canonical: kompass.callbacks.PoseStampedCallback

Bases: {py:obj}`kompass.callbacks.PoseCallback`

```{autodoc2-docstring} kompass.callbacks.PoseStampedCallback
```

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.PoseStampedCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.PoseStampedCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.PoseStampedCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.PoseStampedCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.PoseStampedCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.PoseStampedCallback.got_msg

````

`````

`````{py:class} LaserScanCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None)
:canonical: kompass.callbacks.LaserScanCallback

Bases: {py:obj}`ros_sugar.callbacks.GenericCallback`

```{autodoc2-docstring} kompass.callbacks.LaserScanCallback
```

````{py:property} transformation
:canonical: kompass.callbacks.LaserScanCallback.transformation
:type: typing.Optional[tf2_ros.TransformStamped]

```{autodoc2-docstring} kompass.callbacks.LaserScanCallback.transformation
```

````

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.LaserScanCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.LaserScanCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.LaserScanCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.LaserScanCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.LaserScanCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.LaserScanCallback.got_msg

````

`````

`````{py:class} PointCloudCallback(input_topic, node_name: typing.Optional[str] = None, transformation: typing.Optional[tf2_ros.TransformStamped] = None, max_range: typing.Optional[float] = None)
:canonical: kompass.callbacks.PointCloudCallback

Bases: {py:obj}`ros_sugar.callbacks.GenericCallback`

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

````{py:method} set_node_name(node_name: str) -> None
:canonical: kompass.callbacks.PointCloudCallback.set_node_name

````

````{py:method} set_subscriber(subscriber: rclpy.subscription.Subscription) -> None
:canonical: kompass.callbacks.PointCloudCallback.set_subscriber

````

````{py:method} on_callback_execute(callback: typing.Callable) -> None
:canonical: kompass.callbacks.PointCloudCallback.on_callback_execute

````

````{py:method} callback(msg) -> None
:canonical: kompass.callbacks.PointCloudCallback.callback

````

````{py:method} get_output() -> typing.Any
:canonical: kompass.callbacks.PointCloudCallback.get_output

````

````{py:property} got_msg
:canonical: kompass.callbacks.PointCloudCallback.got_msg

````

`````
