---
orphan: true
---

# {py:mod}`kompass.components.mapper`

```{py:module} kompass.components.mapper
```

```{autodoc2-docstring} kompass.components.mapper
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`LocalMapperConfig <kompass.components.mapper.LocalMapperConfig>`
  - ```{autodoc2-docstring} kompass.components.mapper.LocalMapperConfig
    :summary:
    ```
* - {py:obj}`LocalMapper <kompass.components.mapper.LocalMapper>`
  - ```{autodoc2-docstring} kompass.components.mapper.LocalMapper
    :summary:
    ```
````

### API

````{py:class} LocalMapperConfig
:canonical: kompass.components.mapper.LocalMapperConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.mapper.LocalMapperConfig
```

````

`````{py:class} LocalMapper(*, component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.mapper.LocalMapperConfig] = None, inputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.mapper.LocalMapper

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.mapper.LocalMapper
```

````{py:method} init_variables()
:canonical: kompass.components.mapper.LocalMapper.init_variables

```{autodoc2-docstring} kompass.components.mapper.LocalMapper.init_variables
```

````

````{py:method} publish_data()
:canonical: kompass.components.mapper.LocalMapper.publish_data

```{autodoc2-docstring} kompass.components.mapper.LocalMapper.publish_data
```

````

````{py:method} custom_on_configure()
:canonical: kompass.components.mapper.LocalMapper.custom_on_configure

````

````{py:property} robot
:canonical: kompass.components.mapper.LocalMapper.robot
:type: kompass.config.RobotConfig

````

````{py:property} run_type
:canonical: kompass.components.mapper.LocalMapper.run_type
:type: kompass.config.ComponentRunType

````

````{py:property} inputs_keys
:canonical: kompass.components.mapper.LocalMapper.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:property} outputs_keys
:canonical: kompass.components.mapper.LocalMapper.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.mapper.LocalMapper.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.mapper.LocalMapper.outputs

````

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.mapper.LocalMapper.config_from_file

````

````{py:property} odom_tf_listener
:canonical: kompass.components.mapper.LocalMapper.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: kompass.components.mapper.LocalMapper.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: kompass.components.mapper.LocalMapper.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} pc_tf_listener
:canonical: kompass.components.mapper.LocalMapper.pc_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.mapper.LocalMapper.get_transform_listener

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.mapper.LocalMapper.in_topic_name

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.mapper.LocalMapper.out_topic_name

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.mapper.LocalMapper.get_in_topic

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.mapper.LocalMapper.get_out_topic

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.mapper.LocalMapper.get_callback

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.mapper.LocalMapper.get_publisher

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.mapper.LocalMapper.callbacks_inputs_check

````

`````
