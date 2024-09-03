---
orphan: true
---

# {py:mod}`Kompass.components.planner`

```{py:module} Kompass.components.planner
```

```{autodoc2-docstring} Kompass.components.planner
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`PlannerInputs <Kompass.components.planner.PlannerInputs>`
  - ```{autodoc2-docstring} Kompass.components.planner.PlannerInputs
    :summary:
    ```
* - {py:obj}`PlannerOutputs <Kompass.components.planner.PlannerOutputs>`
  - ```{autodoc2-docstring} Kompass.components.planner.PlannerOutputs
    :summary:
    ```
* - {py:obj}`PlannerConfig <Kompass.components.planner.PlannerConfig>`
  - ```{autodoc2-docstring} Kompass.components.planner.PlannerConfig
    :summary:
    ```
* - {py:obj}`Planner <Kompass.components.planner.Planner>`
  - ```{autodoc2-docstring} Kompass.components.planner.Planner
    :summary:
    ```
````

### API

````{py:class} PlannerInputs
:canonical: Kompass.components.planner.PlannerInputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.planner.PlannerInputs
```

````

````{py:class} PlannerOutputs
:canonical: Kompass.components.planner.PlannerOutputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.planner.PlannerOutputs
```

````

````{py:class} PlannerConfig
:canonical: Kompass.components.planner.PlannerConfig

Bases: {py:obj}`Kompass.config.ComponentConfig`

```{autodoc2-docstring} Kompass.components.planner.PlannerConfig
```

````

`````{py:class} Planner(node_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[Kompass.components.planner.PlannerConfig] = None, inputs: typing.Optional[typing.Dict[str, Kompass.topic.Topic]] = None, outputs: typing.Optional[typing.Dict[str, Kompass.topic.Topic]] = None, **kwargs)
:canonical: Kompass.components.planner.Planner

Bases: {py:obj}`Kompass.components.component.Component`

```{autodoc2-docstring} Kompass.components.planner.Planner
```

````{py:method} configure(config_file: str)
:canonical: Kompass.components.planner.Planner.configure

```{autodoc2-docstring} Kompass.components.planner.Planner.configure
```

````

````{py:method} init_variables()
:canonical: Kompass.components.planner.Planner.init_variables

```{autodoc2-docstring} Kompass.components.planner.Planner.init_variables
```

````

````{py:method} create_all_services()
:canonical: Kompass.components.planner.Planner.create_all_services

```{autodoc2-docstring} Kompass.components.planner.Planner.create_all_services
```

````

````{py:method} destroy_all_services()
:canonical: Kompass.components.planner.Planner.destroy_all_services

```{autodoc2-docstring} Kompass.components.planner.Planner.destroy_all_services
```

````

````{py:method} attach_callbacks()
:canonical: Kompass.components.planner.Planner.attach_callbacks

```{autodoc2-docstring} Kompass.components.planner.Planner.attach_callbacks
```

````

````{py:method} main_service_callback(request: kompass_interfaces.srv.PlanPath.Request, response: kompass_interfaces.srv.PlanPath.Response)
:canonical: Kompass.components.planner.Planner.main_service_callback

```{autodoc2-docstring} Kompass.components.planner.Planner.main_service_callback
```

````

````{py:method} reached_point(goal_point: kompass_core.models.RobotState, tolerance: kompass_interfaces.msg.PathTrackingError) -> bool
:canonical: Kompass.components.planner.Planner.reached_point

```{autodoc2-docstring} Kompass.components.planner.Planner.reached_point
```

````

````{py:method} main_action_callback(goal_handle: kompass_interfaces.action.PlanPath.Goal)
:canonical: Kompass.components.planner.Planner.main_action_callback

```{autodoc2-docstring} Kompass.components.planner.Planner.main_action_callback
```

````

````{py:property} robot
:canonical: Kompass.components.planner.Planner.robot
:type: Kompass.config.RobotConfig

````

````{py:method} create_all_subscribers()
:canonical: Kompass.components.planner.Planner.create_all_subscribers

````

````{py:method} create_all_publishers()
:canonical: Kompass.components.planner.Planner.create_all_publishers

````

````{py:method} destroy_all_subscribers()
:canonical: Kompass.components.planner.Planner.destroy_all_subscribers

````

````{py:method} destroy_all_publishers()
:canonical: Kompass.components.planner.Planner.destroy_all_publishers

````

````{py:method} inputs(**kwargs)
:canonical: Kompass.components.planner.Planner.inputs

````

````{py:method} outputs(**kwargs)
:canonical: Kompass.components.planner.Planner.outputs

````

````{py:method} update_cmd_args_list()
:canonical: Kompass.components.planner.Planner.update_cmd_args_list

````

````{py:method} add_ros_subscriber(callback: Kompass.callbacks.GenericCallback)
:canonical: Kompass.components.planner.Planner.add_ros_subscriber

````

````{py:method} add_ros_publisher(publisher: Kompass.topic.Publisher)
:canonical: Kompass.components.planner.Planner.add_ros_publisher

````

````{py:property} odom_tf_listener
:canonical: Kompass.components.planner.Planner.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: Kompass.components.planner.Planner.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: Kompass.components.planner.Planner.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: Kompass.components.planner.Planner.get_transform_listener

````

````{py:method} got_all_inputs(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.planner.Planner.got_all_inputs

````

````{py:method} get_missing_inputs() -> list[str]
:canonical: Kompass.components.planner.Planner.get_missing_inputs

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.planner.Planner.callbacks_inputs_check

````

````{py:property} inputs_json
:canonical: Kompass.components.planner.Planner.inputs_json
:type: typing.Union[str, bytes, bytearray]

````

````{py:property} outputs_json
:canonical: Kompass.components.planner.Planner.outputs_json
:type: typing.Union[str, bytes, bytearray]

````

````{py:property} events_json
:canonical: Kompass.components.planner.Planner.events_json
:type: typing.Union[str, bytes]

````

`````
