# {py:mod}`Kompass.launcher`

```{py:module} Kompass.launcher
```

```{autodoc2-docstring} Kompass.launcher
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Launcher <Kompass.launcher.Launcher>`
  - ```{autodoc2-docstring} Kompass.launcher.Launcher
    :summary:
    ```
````

### API

`````{py:class} Launcher(components: typing.List[Kompass.components.component.Component], events_actions: typing.Dict[Kompass.event.Event, Kompass.actions.Action] | None = None, namespace: str = '', config_file: str | None = None, enable_monitoring: bool = True, multi_processing: bool = True, activate_all_components_on_start: bool = False, components_to_activate_on_start: typing.Optional[typing.List[Kompass.components.component.Component]] = None)
:canonical: Kompass.launcher.Launcher

Bases: {py:obj}`ros_sugar.launcher.Launcher`

```{autodoc2-docstring} Kompass.launcher.Launcher
```

````{py:property} robot
:canonical: Kompass.launcher.Launcher.robot
:type: typing.Dict[str, Kompass.config.RobotConfig]

```{autodoc2-docstring} Kompass.launcher.Launcher.robot
```

````

````{py:property} frames
:canonical: Kompass.launcher.Launcher.frames
:type: typing.Dict[str, Kompass.config.RobotFrames]

```{autodoc2-docstring} Kompass.launcher.Launcher.frames
```

````

````{py:method} inputs(**kwargs)
:canonical: Kompass.launcher.Launcher.inputs

```{autodoc2-docstring} Kompass.launcher.Launcher.inputs
```

````

````{py:method} outputs(**kwargs)
:canonical: Kompass.launcher.Launcher.outputs

```{autodoc2-docstring} Kompass.launcher.Launcher.outputs
```

````

````{py:method} start(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: Kompass.launcher.Launcher.start

````

````{py:method} stop(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: Kompass.launcher.Launcher.stop

````

````{py:method} restart(node_name: str, **kwargs) -> typing.List[launch.some_entities_type.SomeEntitiesType]
:canonical: Kompass.launcher.Launcher.restart

````

````{py:method} add_event(new_event: ros_sugar.events.Event)
:canonical: Kompass.launcher.Launcher.add_event

````

````{py:method} update_event(new_event: ros_sugar.events.Event)
:canonical: Kompass.launcher.Launcher.update_event

````

````{py:method} configure(config_file: str, keep_alive: bool = False, component_name: str | None = None)
:canonical: Kompass.launcher.Launcher.configure

````

````{py:method} add_py_executable(path_to_executable: str, name: str = 'python3')
:canonical: Kompass.launcher.Launcher.add_py_executable

````

````{py:method} add_method(method: typing.Callable | typing.Awaitable, args: typing.Iterable | None = None, kwargs: typing.Dict | None = None)
:canonical: Kompass.launcher.Launcher.add_method

````

````{py:method} bringup(config_file: str | None = None, introspect: bool = True, launch_debug: bool = False, ros_log_level: str = 'info')
:canonical: Kompass.launcher.Launcher.bringup

````

`````
