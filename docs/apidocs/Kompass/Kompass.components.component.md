---
orphan: true
---

# {py:mod}`Kompass.components.component`

```{py:module} Kompass.components.component
```

```{autodoc2-docstring} Kompass.components.component
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Component <Kompass.components.component.Component>`
  - ```{autodoc2-docstring} Kompass.components.component.Component
    :summary:
    ```
````

### API

`````{py:class} Component(node_name: str, config: typing.Optional[Kompass.config.ComponentConfig] = None, config_file: typing.Optional[str] = None, inputs: typing.Optional[Kompass.config.BaseAttrs] = None, outputs: typing.Optional[Kompass.config.BaseAttrs] = None, fallbacks: typing.Optional[ros_sugar.fallbacks.ComponentFallbacks] = None, allowed_inputs: typing.Optional[type[Kompass.topic.RestrictedTopicsConfig]] = None, allowed_outputs: typing.Optional[type[Kompass.topic.RestrictedTopicsConfig]] = None, callback_group=None, **kwargs)
:canonical: Kompass.components.component.Component

Bases: {py:obj}`ros_sugar.component.BaseComponent`

```{autodoc2-docstring} Kompass.components.component.Component
```

````{py:property} robot
:canonical: Kompass.components.component.Component.robot
:type: Kompass.config.RobotConfig

```{autodoc2-docstring} Kompass.components.component.Component.robot
```

````

````{py:method} create_all_subscribers()
:canonical: Kompass.components.component.Component.create_all_subscribers

```{autodoc2-docstring} Kompass.components.component.Component.create_all_subscribers
```

````

````{py:method} create_all_publishers()
:canonical: Kompass.components.component.Component.create_all_publishers

```{autodoc2-docstring} Kompass.components.component.Component.create_all_publishers
```

````

````{py:method} destroy_all_subscribers()
:canonical: Kompass.components.component.Component.destroy_all_subscribers

```{autodoc2-docstring} Kompass.components.component.Component.destroy_all_subscribers
```

````

````{py:method} destroy_all_publishers()
:canonical: Kompass.components.component.Component.destroy_all_publishers

```{autodoc2-docstring} Kompass.components.component.Component.destroy_all_publishers
```

````

````{py:method} inputs(**kwargs)
:canonical: Kompass.components.component.Component.inputs

```{autodoc2-docstring} Kompass.components.component.Component.inputs
```

````

````{py:method} outputs(**kwargs)
:canonical: Kompass.components.component.Component.outputs

```{autodoc2-docstring} Kompass.components.component.Component.outputs
```

````

````{py:method} configure(config_file: str)
:canonical: Kompass.components.component.Component.configure

```{autodoc2-docstring} Kompass.components.component.Component.configure
```

````

````{py:method} update_cmd_args_list()
:canonical: Kompass.components.component.Component.update_cmd_args_list

```{autodoc2-docstring} Kompass.components.component.Component.update_cmd_args_list
```

````

````{py:method} add_ros_subscriber(callback: Kompass.callbacks.GenericCallback)
:canonical: Kompass.components.component.Component.add_ros_subscriber

```{autodoc2-docstring} Kompass.components.component.Component.add_ros_subscriber
```

````

````{py:method} add_ros_publisher(publisher: Kompass.topic.Publisher)
:canonical: Kompass.components.component.Component.add_ros_publisher

```{autodoc2-docstring} Kompass.components.component.Component.add_ros_publisher
```

````

````{py:property} odom_tf_listener
:canonical: Kompass.components.component.Component.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

```{autodoc2-docstring} Kompass.components.component.Component.odom_tf_listener
```

````

````{py:property} scan_tf_listener
:canonical: Kompass.components.component.Component.scan_tf_listener
:type: ros_sugar.tf.TFListener

```{autodoc2-docstring} Kompass.components.component.Component.scan_tf_listener
```

````

````{py:property} depth_tf_listener
:canonical: Kompass.components.component.Component.depth_tf_listener
:type: ros_sugar.tf.TFListener

```{autodoc2-docstring} Kompass.components.component.Component.depth_tf_listener
```

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: Kompass.components.component.Component.get_transform_listener

```{autodoc2-docstring} Kompass.components.component.Component.get_transform_listener
```

````

````{py:method} got_all_inputs(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.component.Component.got_all_inputs

```{autodoc2-docstring} Kompass.components.component.Component.got_all_inputs
```

````

````{py:method} get_missing_inputs() -> list[str]
:canonical: Kompass.components.component.Component.get_missing_inputs

```{autodoc2-docstring} Kompass.components.component.Component.get_missing_inputs
```

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.component.Component.callbacks_inputs_check

```{autodoc2-docstring} Kompass.components.component.Component.callbacks_inputs_check
```

````

````{py:property} inputs_json
:canonical: Kompass.components.component.Component.inputs_json
:type: typing.Union[str, bytes, bytearray]

```{autodoc2-docstring} Kompass.components.component.Component.inputs_json
```

````

````{py:property} outputs_json
:canonical: Kompass.components.component.Component.outputs_json
:type: typing.Union[str, bytes, bytearray]

```{autodoc2-docstring} Kompass.components.component.Component.outputs_json
```

````

````{py:property} events_json
:canonical: Kompass.components.component.Component.events_json
:type: typing.Union[str, bytes]

```{autodoc2-docstring} Kompass.components.component.Component.events_json
```

````

`````
