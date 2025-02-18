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

````{py:method} execute_cmd_open_loop(cmd: geometry_msgs.msg.Twist, max_time: float)
:canonical: kompass.components.drive_manager.DriveManager.execute_cmd_open_loop

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.execute_cmd_open_loop
```

````

````{py:method} execute_cmd_closed_loop(cmd: geometry_msgs.msg.Twist, max_time: float)
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

````{py:method} move_to_unblock(max_distance_forward: typing.Optional[float] = None, max_distance_backwards: typing.Optional[float] = None, max_rotation: float = np.pi / 4, rotation_safety_margin: typing.Optional[float] = None) -> bool
:canonical: kompass.components.drive_manager.DriveManager.move_to_unblock

```{autodoc2-docstring} kompass.components.drive_manager.DriveManager.move_to_unblock
```

````

`````
