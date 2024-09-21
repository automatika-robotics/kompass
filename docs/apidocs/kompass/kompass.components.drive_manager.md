---
orphan: true
---

# {py:mod}`kompass.components.drive_manager`

```{py:module} kompass.components.drive_manager
```

```{autodoc2-docstring} kompass.components.drive_manager
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`DriveManagerConfig <kompass.components.drive_manager.DriveManagerConfig>`
  - ```{autodoc2-docstring} kompass.components.drive_manager.DriveManagerConfig
    :summary:
    ```
* - {py:obj}`DriveManager <kompass.components.drive_manager.DriveManager>`
  - ```{autodoc2-docstring} kompass.components.drive_manager.DriveManager
    :summary:
    ```
````

### API

````{py:class} DriveManagerConfig
:canonical: kompass.components.drive_manager.DriveManagerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.drive_manager.DriveManagerConfig
```

````

`````{py:class} DriveManager(component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.drive_manager.DriveManagerConfig] = None, inputs=None, outputs=None, **kwargs)
:canonical: kompass.components.drive_manager.DriveManager

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager
```

````{py:property} run_type
:canonical: kompass.components.drive_manager.DriveManager.run_type
:type: kompass.config.ComponentRunType

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.run_type
```

````

````{py:method} init_flags()
:canonical: kompass.components.drive_manager.DriveManager.init_flags

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.init_flags
```

````

````{py:method} init_variables()
:canonical: kompass.components.drive_manager.DriveManager.init_variables

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.init_variables
```

````

````{py:method} attach_callbacks()
:canonical: kompass.components.drive_manager.DriveManager.attach_callbacks

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.attach_callbacks
```

````

````{py:method} move_to_unblock(max_distance: float = 0.2)
:canonical: kompass.components.drive_manager.DriveManager.move_to_unblock

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.move_to_unblock
```

````

````{py:method} check_limits()
:canonical: kompass.components.drive_manager.DriveManager.check_limits

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.check_limits
```

````

````{py:property} robot
:canonical: kompass.components.drive_manager.DriveManager.robot
:type: kompass.config.RobotConfig

````

````{py:method} create_all_subscribers()
:canonical: kompass.components.drive_manager.DriveManager.create_all_subscribers

````

````{py:method} create_all_publishers()
:canonical: kompass.components.drive_manager.DriveManager.create_all_publishers

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.drive_manager.DriveManager.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.drive_manager.DriveManager.outputs

````

````{py:method} configure(config_file: str)
:canonical: kompass.components.drive_manager.DriveManager.configure

````

````{py:property} odom_tf_listener
:canonical: kompass.components.drive_manager.DriveManager.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: kompass.components.drive_manager.DriveManager.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: kompass.components.drive_manager.DriveManager.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.drive_manager.DriveManager.get_transform_listener

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.drive_manager.DriveManager.callbacks_inputs_check

````

`````
