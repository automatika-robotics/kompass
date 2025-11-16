---
orphan: true
---

# {py:mod}`kompass.components.component`

```{py:module} kompass.components.component
```

```{autodoc2-docstring} kompass.components.component
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Component <kompass.components.component.Component>`
  - ```{autodoc2-docstring} kompass.components.component.Component
    :summary:
    ```
````

### API

`````{py:class} Component(component_name: str, config: typing.Optional[kompass.config.ComponentConfig] = None, config_file: typing.Optional[str] = None, inputs: typing.Optional[typing.Dict[kompass.components.defaults.TopicsKeys, typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]]] = None, outputs: typing.Optional[typing.Dict[kompass.components.defaults.TopicsKeys, typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]]] = None, fallbacks: typing.Optional[ros_sugar.core.ComponentFallbacks] = None, allowed_inputs: typing.Optional[typing.Dict[str, ros_sugar.io.AllowedTopics]] = None, allowed_outputs: typing.Optional[typing.Dict[str, ros_sugar.io.AllowedTopics]] = None, allowed_run_types: typing.Optional[typing.List[kompass.config.ComponentRunType]] = None, callback_group=None, **kwargs)
:canonical: kompass.components.component.Component

Bases: {py:obj}`ros_sugar.core.BaseComponent`

```{autodoc2-docstring} kompass.components.component.Component
```

````{py:method} custom_on_configure()
:canonical: kompass.components.component.Component.custom_on_configure

```{autodoc2-docstring} kompass.components.component.Component.custom_on_configure
```

````

````{py:property} robot
:canonical: kompass.components.component.Component.robot
:type: kompass.config.RobotConfig

```{autodoc2-docstring} kompass.components.component.Component.robot
```

````

````{py:property} run_type
:canonical: kompass.components.component.Component.run_type
:type: kompass.config.ComponentRunType

```{autodoc2-docstring} kompass.components.component.Component.run_type
```

````

````{py:property} inputs_keys
:canonical: kompass.components.component.Component.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

```{autodoc2-docstring} kompass.components.component.Component.inputs_keys
```

````

````{py:property} outputs_keys
:canonical: kompass.components.component.Component.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

```{autodoc2-docstring} kompass.components.component.Component.outputs_keys
```

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.component.Component.inputs

```{autodoc2-docstring} kompass.components.component.Component.inputs
```

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.component.Component.outputs

```{autodoc2-docstring} kompass.components.component.Component.outputs
```

````

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.component.Component.config_from_file

```{autodoc2-docstring} kompass.components.component.Component.config_from_file
```

````

````{py:property} odom_tf_listener
:canonical: kompass.components.component.Component.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

```{autodoc2-docstring} kompass.components.component.Component.odom_tf_listener
```

````

````{py:property} scan_tf_listener
:canonical: kompass.components.component.Component.scan_tf_listener
:type: ros_sugar.tf.TFListener

```{autodoc2-docstring} kompass.components.component.Component.scan_tf_listener
```

````

````{py:property} depth_tf_listener
:canonical: kompass.components.component.Component.depth_tf_listener
:type: ros_sugar.tf.TFListener

```{autodoc2-docstring} kompass.components.component.Component.depth_tf_listener
```

````

````{py:property} pc_tf_listener
:canonical: kompass.components.component.Component.pc_tf_listener
:type: ros_sugar.tf.TFListener

```{autodoc2-docstring} kompass.components.component.Component.pc_tf_listener
```

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.component.Component.get_transform_listener

```{autodoc2-docstring} kompass.components.component.Component.get_transform_listener
```

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.component.Component.in_topic_name

```{autodoc2-docstring} kompass.components.component.Component.in_topic_name
```

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.component.Component.out_topic_name

```{autodoc2-docstring} kompass.components.component.Component.out_topic_name
```

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.component.Component.get_in_topic

```{autodoc2-docstring} kompass.components.component.Component.get_in_topic
```

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.component.Component.get_out_topic

```{autodoc2-docstring} kompass.components.component.Component.get_out_topic
```

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.component.Component.get_callback

```{autodoc2-docstring} kompass.components.component.Component.get_callback
```

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.component.Component.get_publisher

```{autodoc2-docstring} kompass.components.component.Component.get_publisher
```

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.component.Component.callbacks_inputs_check

```{autodoc2-docstring} kompass.components.component.Component.callbacks_inputs_check
```

````

`````
