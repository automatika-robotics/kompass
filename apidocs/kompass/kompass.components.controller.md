---
orphan: true
---

# {py:mod}`kompass.components.controller`

```{py:module} kompass.components.controller
```

```{autodoc2-docstring} kompass.components.controller
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`CmdPublishType <kompass.components.controller.CmdPublishType>`
  - ```{autodoc2-docstring} kompass.components.controller.CmdPublishType
    :summary:
    ```
* - {py:obj}`ControllerConfig <kompass.components.controller.ControllerConfig>`
  - ```{autodoc2-docstring} kompass.components.controller.ControllerConfig
    :summary:
    ```
* - {py:obj}`Controller <kompass.components.controller.Controller>`
  - ```{autodoc2-docstring} kompass.components.controller.Controller
    :summary:
    ```
````

### API

````{py:class} CmdPublishType
:canonical: kompass.components.controller.CmdPublishType

Bases: {py:obj}`kompass.utils.StrEnum`

```{autodoc2-docstring} kompass.components.controller.CmdPublishType
```

````

````{py:class} ControllerConfig
:canonical: kompass.components.controller.ControllerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.controller.ControllerConfig
```

````

`````{py:class} Controller(*, component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.controller.ControllerConfig] = None, inputs: typing.Optional[typing.Dict[kompass.components.defaults.TopicsKeys, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[kompass.components.defaults.TopicsKeys, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.controller.Controller

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.controller.Controller
```

````{py:method} custom_on_activate()
:canonical: kompass.components.controller.Controller.custom_on_activate

```{autodoc2-docstring} kompass.components.controller.Controller.custom_on_activate
```

````

````{py:method} custom_create_all_subscribers()
:canonical: kompass.components.controller.Controller.custom_create_all_subscribers

```{autodoc2-docstring} kompass.components.controller.Controller.custom_create_all_subscribers
```

````

````{py:method} create_all_timers()
:canonical: kompass.components.controller.Controller.create_all_timers

```{autodoc2-docstring} kompass.components.controller.Controller.create_all_timers
```

````

````{py:method} destroy_all_timers()
:canonical: kompass.components.controller.Controller.destroy_all_timers

```{autodoc2-docstring} kompass.components.controller.Controller.destroy_all_timers
```

````

````{py:method} create_all_services()
:canonical: kompass.components.controller.Controller.create_all_services

```{autodoc2-docstring} kompass.components.controller.Controller.create_all_services
```

````

````{py:method} destroy_all_services()
:canonical: kompass.components.controller.Controller.destroy_all_services

```{autodoc2-docstring} kompass.components.controller.Controller.destroy_all_services
```

````

````{py:property} tracked_point
:canonical: kompass.components.controller.Controller.tracked_point
:type: typing.Optional[numpy.ndarray]

```{autodoc2-docstring} kompass.components.controller.Controller.tracked_point
```

````

````{py:property} local_plan
:canonical: kompass.components.controller.Controller.local_plan
:type: typing.Optional[nav_msgs.msg.Path]

```{autodoc2-docstring} kompass.components.controller.Controller.local_plan
```

````

````{py:property} local_plan_debug
:canonical: kompass.components.controller.Controller.local_plan_debug
:type: typing.Optional[nav_msgs.msg.Path]

```{autodoc2-docstring} kompass.components.controller.Controller.local_plan_debug
```

````

````{py:property} interpolated_path
:canonical: kompass.components.controller.Controller.interpolated_path
:type: typing.Optional[nav_msgs.msg.Path]

```{autodoc2-docstring} kompass.components.controller.Controller.interpolated_path
```

````

````{py:property} direct_sensor
:canonical: kompass.components.controller.Controller.direct_sensor
:type: bool

```{autodoc2-docstring} kompass.components.controller.Controller.direct_sensor
```

````

````{py:property} algorithm
:canonical: kompass.components.controller.Controller.algorithm
:type: kompass_core.control.ControllersID

```{autodoc2-docstring} kompass.components.controller.Controller.algorithm
```

````

````{py:method} set_algorithm(algorithm_value: typing.Union[str, kompass_core.control.ControllersID], **_) -> bool
:canonical: kompass.components.controller.Controller.set_algorithm

```{autodoc2-docstring} kompass.components.controller.Controller.set_algorithm
```

````

````{py:method} init_variables()
:canonical: kompass.components.controller.Controller.init_variables

```{autodoc2-docstring} kompass.components.controller.Controller.init_variables
```

````

````{py:method} main_action_callback(goal_handle) -> typing.Union[kompass_interfaces.action.ControlPath.Result, kompass_interfaces.action.TrackVisionTarget.Result]
:canonical: kompass.components.controller.Controller.main_action_callback

```{autodoc2-docstring} kompass.components.controller.Controller.main_action_callback
```

````

````{py:method} reached_point(goal_point: typing.Optional[kompass_core.models.RobotState]) -> bool
:canonical: kompass.components.controller.Controller.reached_point

```{autodoc2-docstring} kompass.components.controller.Controller.reached_point
```

````

````{py:method} custom_on_configure()
:canonical: kompass.components.controller.Controller.custom_on_configure

````

````{py:property} robot
:canonical: kompass.components.controller.Controller.robot
:type: kompass.config.RobotConfig

````

````{py:property} run_type
:canonical: kompass.components.controller.Controller.run_type
:type: kompass.config.ComponentRunType

````

````{py:property} inputs_keys
:canonical: kompass.components.controller.Controller.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:property} outputs_keys
:canonical: kompass.components.controller.Controller.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.controller.Controller.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.controller.Controller.outputs

````

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.controller.Controller.config_from_file

````

````{py:property} odom_tf_listener
:canonical: kompass.components.controller.Controller.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: kompass.components.controller.Controller.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: kompass.components.controller.Controller.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} pc_tf_listener
:canonical: kompass.components.controller.Controller.pc_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.controller.Controller.get_transform_listener

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.controller.Controller.in_topic_name

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.controller.Controller.out_topic_name

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.controller.Controller.get_in_topic

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.controller.Controller.get_out_topic

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.controller.Controller.get_callback

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.controller.Controller.get_publisher

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.controller.Controller.callbacks_inputs_check

````

`````
