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

`````{py:class} DriveManager(component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.drive_manager.DriveManagerConfig] = None, inputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.drive_manager.DriveManager

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager
```

````{py:method} init_variables()
:canonical: kompass.components.drive_manager.DriveManager.init_variables

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.init_variables
```

````

````{py:method} execute_cmd_closed_loop(output: geometry_msgs.msg.Twist, max_time: float)
:canonical: kompass.components.drive_manager.DriveManager.execute_cmd_closed_loop

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.execute_cmd_closed_loop
```

````

````{py:method} move_forward(max_distance: float) -> bool
:canonical: kompass.components.drive_manager.DriveManager.move_forward

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.move_forward
```

````

````{py:method} move_backward(max_distance: float) -> bool
:canonical: kompass.components.drive_manager.DriveManager.move_backward

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.move_backward
```

````

````{py:method} rotate_in_place(max_rotation: float, safety_margin: typing.Optional[float] = None) -> bool
:canonical: kompass.components.drive_manager.DriveManager.rotate_in_place

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.rotate_in_place
```

````

````{py:method} move_to_unblock(max_distance_forward: typing.Optional[float] = None, max_distance_backwards: typing.Optional[float] = None, max_rotation: float = np.pi / 2, rotation_safety_margin: typing.Optional[float] = None) -> bool
:canonical: kompass.components.drive_manager.DriveManager.move_to_unblock

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.move_to_unblock
```

````

````{py:method} custom_on_configure()
:canonical: kompass.components.drive_manager.DriveManager.custom_on_configure

````

````{py:property} robot
:canonical: kompass.components.drive_manager.DriveManager.robot
:type: kompass.config.RobotConfig

````

````{py:property} run_type
:canonical: kompass.components.drive_manager.DriveManager.run_type
:type: kompass.config.ComponentRunType

````

````{py:property} inputs_keys
:canonical: kompass.components.drive_manager.DriveManager.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:property} outputs_keys
:canonical: kompass.components.drive_manager.DriveManager.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.drive_manager.DriveManager.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.drive_manager.DriveManager.outputs

````

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.drive_manager.DriveManager.config_from_file

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

````{py:property} pc_tf_listener
:canonical: kompass.components.drive_manager.DriveManager.pc_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.drive_manager.DriveManager.get_transform_listener

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.drive_manager.DriveManager.in_topic_name

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.drive_manager.DriveManager.out_topic_name

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.drive_manager.DriveManager.get_in_topic

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.drive_manager.DriveManager.get_out_topic

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.drive_manager.DriveManager.get_callback

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.drive_manager.DriveManager.get_publisher

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.drive_manager.DriveManager.callbacks_inputs_check

````

`````
