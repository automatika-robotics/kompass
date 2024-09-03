---
orphan: true
---

# {py:mod}`Kompass.components.drive_manager`

```{py:module} Kompass.components.drive_manager
```

```{autodoc2-docstring} Kompass.components.drive_manager
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`DriverInputs <Kompass.components.drive_manager.DriverInputs>`
  - ```{autodoc2-docstring} Kompass.components.drive_manager.DriverInputs
    :summary:
    ```
* - {py:obj}`DriverOutputs <Kompass.components.drive_manager.DriverOutputs>`
  - ```{autodoc2-docstring} Kompass.components.drive_manager.DriverOutputs
    :summary:
    ```
* - {py:obj}`DriveManagerConfig <Kompass.components.drive_manager.DriveManagerConfig>`
  - ```{autodoc2-docstring} Kompass.components.drive_manager.DriveManagerConfig
    :summary:
    ```
* - {py:obj}`DriveManager <Kompass.components.drive_manager.DriveManager>`
  - ```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager
    :summary:
    ```
````

### API

````{py:class} DriverInputs
:canonical: Kompass.components.drive_manager.DriverInputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.drive_manager.DriverInputs
```

````

````{py:class} DriverOutputs
:canonical: Kompass.components.drive_manager.DriverOutputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.drive_manager.DriverOutputs
```

````

````{py:class} DriveManagerConfig
:canonical: Kompass.components.drive_manager.DriveManagerConfig

Bases: {py:obj}`Kompass.config.ComponentConfig`

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManagerConfig
```

````

`````{py:class} DriveManager(node_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[Kompass.components.drive_manager.DriveManagerConfig] = None, inputs=None, outputs=None, **kwargs)
:canonical: Kompass.components.drive_manager.DriveManager

Bases: {py:obj}`Kompass.components.component.Component`

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager
```

````{py:property} run_type
:canonical: Kompass.components.drive_manager.DriveManager.run_type
:type: Kompass.config.ComponentRunType

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.run_type
```

````

````{py:method} init_flags()
:canonical: Kompass.components.drive_manager.DriveManager.init_flags

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.init_flags
```

````

````{py:method} init_variables()
:canonical: Kompass.components.drive_manager.DriveManager.init_variables

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.init_variables
```

````

````{py:method} attach_callbacks()
:canonical: Kompass.components.drive_manager.DriveManager.attach_callbacks

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.attach_callbacks
```

````

````{py:method} move_to_unblock(max_distance: float = 0.2)
:canonical: Kompass.components.drive_manager.DriveManager.move_to_unblock

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.move_to_unblock
```

````

````{py:method} check_limits()
:canonical: Kompass.components.drive_manager.DriveManager.check_limits

```{autodoc2-docstring} Kompass.components.drive_manager.DriveManager.check_limits
```

````

````{py:property} robot
:canonical: Kompass.components.drive_manager.DriveManager.robot
:type: Kompass.config.RobotConfig

````

````{py:method} create_all_subscribers()
:canonical: Kompass.components.drive_manager.DriveManager.create_all_subscribers

````

````{py:method} create_all_publishers()
:canonical: Kompass.components.drive_manager.DriveManager.create_all_publishers

````

````{py:method} destroy_all_subscribers()
:canonical: Kompass.components.drive_manager.DriveManager.destroy_all_subscribers

````

````{py:method} destroy_all_publishers()
:canonical: Kompass.components.drive_manager.DriveManager.destroy_all_publishers

````

````{py:method} inputs(**kwargs)
:canonical: Kompass.components.drive_manager.DriveManager.inputs

````

````{py:method} outputs(**kwargs)
:canonical: Kompass.components.drive_manager.DriveManager.outputs

````

````{py:method} configure(config_file: str)
:canonical: Kompass.components.drive_manager.DriveManager.configure

````

````{py:method} update_cmd_args_list()
:canonical: Kompass.components.drive_manager.DriveManager.update_cmd_args_list

````

````{py:method} add_ros_subscriber(callback: Kompass.callbacks.GenericCallback)
:canonical: Kompass.components.drive_manager.DriveManager.add_ros_subscriber

````

````{py:method} add_ros_publisher(publisher: Kompass.topic.Publisher)
:canonical: Kompass.components.drive_manager.DriveManager.add_ros_publisher

````

````{py:property} odom_tf_listener
:canonical: Kompass.components.drive_manager.DriveManager.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: Kompass.components.drive_manager.DriveManager.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: Kompass.components.drive_manager.DriveManager.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: Kompass.components.drive_manager.DriveManager.get_transform_listener

````

````{py:method} got_all_inputs(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.drive_manager.DriveManager.got_all_inputs

````

````{py:method} get_missing_inputs() -> list[str]
:canonical: Kompass.components.drive_manager.DriveManager.get_missing_inputs

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_execlude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: Kompass.components.drive_manager.DriveManager.callbacks_inputs_check

````

````{py:property} inputs_json
:canonical: Kompass.components.drive_manager.DriveManager.inputs_json
:type: typing.Union[str, bytes, bytearray]

````

````{py:property} outputs_json
:canonical: Kompass.components.drive_manager.DriveManager.outputs_json
:type: typing.Union[str, bytes, bytearray]

````

````{py:property} events_json
:canonical: Kompass.components.drive_manager.DriveManager.events_json
:type: typing.Union[str, bytes]

````

`````
