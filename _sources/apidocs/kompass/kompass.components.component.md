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

`````{py:class} Component(component_name: str, config: typing.Optional[kompass.config.ComponentConfig] = None, config_file: typing.Optional[str] = None, inputs: typing.Optional[kompass.config.BaseAttrs] = None, outputs: typing.Optional[kompass.config.BaseAttrs] = None, fallbacks: typing.Optional[ros_sugar.core.ComponentFallbacks] = None, allowed_inputs: typing.Optional[type[kompass.topic.RestrictedTopicsConfig]] = None, allowed_outputs: typing.Optional[type[kompass.topic.RestrictedTopicsConfig]] = None, callback_group=None, **kwargs)
:canonical: kompass.components.component.Component

Bases: {py:obj}`ros_sugar.core.BaseComponent`

```{autodoc2-docstring} kompass.components.component.Component
```

````{py:property} robot
:canonical: kompass.components.component.Component.robot
:type: kompass.config.RobotConfig

```{autodoc2-docstring} kompass.components.component.Component.robot
```

````

````{py:method} create_all_subscribers()
:canonical: kompass.components.component.Component.create_all_subscribers

```{autodoc2-docstring} kompass.components.component.Component.create_all_subscribers
```

````

````{py:method} create_all_publishers()
:canonical: kompass.components.component.Component.create_all_publishers

```{autodoc2-docstring} kompass.components.component.Component.create_all_publishers
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

````{py:method} configure(config_file: str)
:canonical: kompass.components.component.Component.configure

```{autodoc2-docstring} kompass.components.component.Component.configure
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

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.component.Component.get_transform_listener

```{autodoc2-docstring} kompass.components.component.Component.get_transform_listener
```

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.component.Component.callbacks_inputs_check

```{autodoc2-docstring} kompass.components.component.Component.callbacks_inputs_check
```

````

`````
