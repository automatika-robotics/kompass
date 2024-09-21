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
  -
````

### API

````{py:class} LocalMapperConfig
:canonical: kompass.components.mapper.LocalMapperConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.mapper.LocalMapperConfig
```

````

`````{py:class} LocalMapper(*, component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.mapper.LocalMapperConfig] = None, inputs=None, outputs=None, **kwargs)
:canonical: kompass.components.mapper.LocalMapper

Bases: {py:obj}`kompass.components.component.Component`

````{py:method} attach_callbacks() -> None
:canonical: kompass.components.mapper.LocalMapper.attach_callbacks

```{autodoc2-docstring} kompass.components.mapper.LocalMapper.attach_callbacks
```

````

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

````{py:property} robot
:canonical: kompass.components.mapper.LocalMapper.robot
:type: kompass.config.RobotConfig

````

````{py:method} create_all_subscribers()
:canonical: kompass.components.mapper.LocalMapper.create_all_subscribers

````

````{py:method} create_all_publishers()
:canonical: kompass.components.mapper.LocalMapper.create_all_publishers

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.mapper.LocalMapper.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.mapper.LocalMapper.outputs

````

````{py:method} configure(config_file: str)
:canonical: kompass.components.mapper.LocalMapper.configure

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

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.mapper.LocalMapper.get_transform_listener

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.mapper.LocalMapper.callbacks_inputs_check

````

`````
