# {py:mod}`kompass.launcher`

```{py:module} kompass.launcher
```

```{autodoc2-docstring} kompass.launcher
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Launcher <kompass.launcher.Launcher>`
  - ```{autodoc2-docstring} kompass.launcher.Launcher
    :summary:
    ```
````

### API

`````{py:class} Launcher(namespace: str = '', config_file: typing.Optional[str] = None, enable_monitoring: bool = True, activation_timeout: typing.Optional[float] = None, robot_plugin: typing.Optional[str] = None, **kwargs)
:canonical: kompass.launcher.Launcher

Bases: {py:obj}`ros_sugar.Launcher`

```{autodoc2-docstring} kompass.launcher.Launcher
```

````{py:method} kompass(components: typing.List[kompass.components.component.Component], events_actions: typing.Optional[typing.Dict[kompass.event.Event, typing.Union[kompass.actions.Action, launch.action.Action, typing.List[typing.Union[kompass.actions.Action, launch.action.Action]]]]] = None, multiprocessing: bool = True, activate_all_components_on_start: bool = True, components_to_activate_on_start: typing.Optional[typing.List[kompass.components.component.Component]] = None, ros_log_level: typing.Optional[str] = None, rclpy_log_level: typing.Optional[str] = None)
:canonical: kompass.launcher.Launcher.kompass

```{autodoc2-docstring} kompass.launcher.Launcher.kompass
```

````

````{py:property} robot
:canonical: kompass.launcher.Launcher.robot
:type: typing.Dict[str, kompass.config.RobotConfig]

```{autodoc2-docstring} kompass.launcher.Launcher.robot
```

````

````{py:property} frames
:canonical: kompass.launcher.Launcher.frames
:type: typing.Dict[str, kompass.config.RobotFrames]

```{autodoc2-docstring} kompass.launcher.Launcher.frames
```

````

````{py:method} inputs(**kwargs)
:canonical: kompass.launcher.Launcher.inputs

```{autodoc2-docstring} kompass.launcher.Launcher.inputs
```

````

````{py:method} outputs(**kwargs)
:canonical: kompass.launcher.Launcher.outputs

```{autodoc2-docstring} kompass.launcher.Launcher.outputs
```

````

`````
