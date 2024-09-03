---
orphan: true
---

# {py:mod}`Kompass.monitor`

```{py:module} Kompass.monitor
```

```{autodoc2-docstring} Kompass.monitor
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`MotionStatus <Kompass.monitor.MotionStatus>`
  - ```{autodoc2-docstring} Kompass.monitor.MotionStatus
    :summary:
    ```
* - {py:obj}`Monitor <Kompass.monitor.Monitor>`
  - ```{autodoc2-docstring} Kompass.monitor.Monitor
    :summary:
    ```
````

### API

`````{py:class} MotionStatus
:canonical: Kompass.monitor.MotionStatus

Bases: {py:obj}`enum.Enum`

```{autodoc2-docstring} Kompass.monitor.MotionStatus
```

````{py:method} name()
:canonical: Kompass.monitor.MotionStatus.name

````

````{py:method} value()
:canonical: Kompass.monitor.MotionStatus.value

````

`````

`````{py:class} Monitor(componenets_names: typing.List[str], enable_health_status_monitoring: bool = True, events: typing.Optional[typing.List[Kompass.event.Event]] = None, actions: typing.Optional[typing.Dict[str, Kompass.actions.Action]] = None, config: typing.Optional[Kompass.config.BaseConfig] = None, services_components: typing.Optional[typing.List[Kompass.components.component.Component]] = None, action_servers_components: typing.Optional[typing.List[Kompass.components.component.Component]] = None, activate_on_start: typing.Optional[typing.List[Kompass.components.component.Component]] = None, start_on_init: bool = False, component_name: str = 'monitor', callback_group: typing.Optional[typing.Union[rclpy.callback_groups.MutuallyExclusiveCallbackGroup, rclpy.callback_groups.ReentrantCallbackGroup]] = None, **kwargs)
:canonical: Kompass.monitor.Monitor

Bases: {py:obj}`ros_sugar.monitor.Monitor`

```{autodoc2-docstring} Kompass.monitor.Monitor
```

````{py:method} init_flags()
:canonical: Kompass.monitor.Monitor.init_flags

```{autodoc2-docstring} Kompass.monitor.Monitor.init_flags
```

````

````{py:method} init_variables()
:canonical: Kompass.monitor.Monitor.init_variables

```{autodoc2-docstring} Kompass.monitor.Monitor.init_variables
```

````

`````
