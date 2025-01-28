---
orphan: true
---

# {py:mod}`kompass.components.planner`

```{py:module} kompass.components.planner
```

```{autodoc2-docstring} kompass.components.planner
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`PlannerConfig <kompass.components.planner.PlannerConfig>`
  - ```{autodoc2-docstring} kompass.components.planner.PlannerConfig
    :summary:
    ```
* - {py:obj}`Planner <kompass.components.planner.Planner>`
  - ```{autodoc2-docstring} kompass.components.planner.Planner
    :summary:
    ```
````

### API

````{py:class} PlannerConfig
:canonical: kompass.components.planner.PlannerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.planner.PlannerConfig
```

````

`````{py:class} Planner(component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.planner.PlannerConfig] = None, inputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.planner.Planner

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.planner.Planner
```

````{py:method} config_from_yaml(config_file: str)
:canonical: kompass.components.planner.Planner.config_from_yaml

```{autodoc2-docstring} kompass.components.planner.Planner.config_from_yaml
```

````

````{py:method} init_variables()
:canonical: kompass.components.planner.Planner.init_variables

```{autodoc2-docstring} kompass.components.planner.Planner.init_variables
```

````

````{py:method} create_all_services()
:canonical: kompass.components.planner.Planner.create_all_services

```{autodoc2-docstring} kompass.components.planner.Planner.create_all_services
```

````

````{py:method} destroy_all_services()
:canonical: kompass.components.planner.Planner.destroy_all_services

```{autodoc2-docstring} kompass.components.planner.Planner.destroy_all_services
```

````

````{py:method} main_service_callback(request: kompass_interfaces.srv.PlanPath.Request, response: kompass_interfaces.srv.PlanPath.Response)
:canonical: kompass.components.planner.Planner.main_service_callback

```{autodoc2-docstring} kompass.components.planner.Planner.main_service_callback
```

````

````{py:method} reached_point(goal_point: kompass_core.models.RobotState, tolerance: kompass_interfaces.msg.PathTrackingError) -> bool
:canonical: kompass.components.planner.Planner.reached_point

```{autodoc2-docstring} kompass.components.planner.Planner.reached_point
```

````

````{py:method} main_action_callback(goal_handle: kompass_interfaces.action.PlanPath.Goal)
:canonical: kompass.components.planner.Planner.main_action_callback

```{autodoc2-docstring} kompass.components.planner.Planner.main_action_callback
```

````

````{py:property} robot
:canonical: kompass.components.planner.Planner.robot
:type: kompass.config.RobotConfig

````

````{py:property} run_type
:canonical: kompass.components.planner.Planner.run_type
:type: kompass.config.ComponentRunType

````

````{py:property} inputs_keys
:canonical: kompass.components.planner.Planner.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:property} outputs_keys
:canonical: kompass.components.planner.Planner.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.planner.Planner.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.planner.Planner.outputs

````

````{py:property} odom_tf_listener
:canonical: kompass.components.planner.Planner.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: kompass.components.planner.Planner.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: kompass.components.planner.Planner.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.planner.Planner.get_transform_listener

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.planner.Planner.in_topic_name

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.planner.Planner.out_topic_name

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.planner.Planner.get_in_topic

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.planner.Planner.get_out_topic

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.planner.Planner.get_callback

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.planner.Planner.get_publisher

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.planner.Planner.callbacks_inputs_check

````

`````
