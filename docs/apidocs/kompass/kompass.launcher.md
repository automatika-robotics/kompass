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

`````{py:class} Launcher(components: typing.List[kompass.components.component.Component], events_actions: typing.Dict[kompass.event.Event, kompass.actions.Action] | None = None, namespace: str = '', config_file: str | None = None, enable_monitoring: bool = True, multi_processing: bool = True, activate_all_components_on_start: bool = False, components_to_activate_on_start: typing.Optional[typing.List[kompass.components.component.Component]] = None)
:canonical: kompass.launcher.Launcher

Bases: {py:obj}`ros_sugar.launcher.Launcher`

```{autodoc2-docstring} kompass.launcher.Launcher
```

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

````{py:method} start(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: kompass.launcher.Launcher.start

````

````{py:method} stop(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: kompass.launcher.Launcher.stop

````

````{py:method} restart(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: kompass.launcher.Launcher.restart

````

````{py:method} add_event(new_event: ros_sugar.events.Event)
:canonical: kompass.launcher.Launcher.add_event

````

````{py:method} update_event(new_event: ros_sugar.events.Event)
:canonical: kompass.launcher.Launcher.update_event

````

````{py:method} configure(config_file: str, keep_alive: bool = False, component_name: str | None = None)
:canonical: kompass.launcher.Launcher.configure

````

````{py:method} add_py_executable(path_to_executable: str, name: str = 'python3')
:canonical: kompass.launcher.Launcher.add_py_executable

````

````{py:method} add_method(method: typing.Callable | typing.Awaitable, args: typing.Iterable | None = None, kwargs: typing.Dict | None = None)
:canonical: kompass.launcher.Launcher.add_method

````

````{py:method} bringup(config_file: str | None = None, introspect: bool = True, launch_debug: bool = False, ros_log_level: str = 'info')
:canonical: kompass.launcher.Launcher.bringup

````

`````
