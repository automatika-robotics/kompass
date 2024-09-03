---
orphan: true
---

# {py:mod}`kompass.monitor`

```{py:module} kompass.monitor
```

```{autodoc2-docstring} kompass.monitor
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`MotionStatus <kompass.monitor.MotionStatus>`
  - ```{autodoc2-docstring} kompass.monitor.MotionStatus
    :summary:
    ```
* - {py:obj}`Monitor <kompass.monitor.Monitor>`
  - ```{autodoc2-docstring} kompass.monitor.Monitor
    :summary:
    ```
````

### API

`````{py:class} MotionStatus
:canonical: kompass.monitor.MotionStatus

Bases: {py:obj}`enum.Enum`

```{autodoc2-docstring} kompass.monitor.MotionStatus
```

````{py:method} name()
:canonical: kompass.monitor.MotionStatus.name

````

````{py:method} value()
:canonical: kompass.monitor.MotionStatus.value

````

`````

`````{py:class} Monitor(componenets_names: typing.List[str], enable_health_status_monitoring: bool = True, events: typing.Optional[typing.List[kompass.event.Event]] = None, actions: typing.Optional[typing.Dict[str, kompass.actions.Action]] = None, config: typing.Optional[kompass.config.BaseConfig] = None, services_components: typing.Optional[typing.List[kompass.components.component.Component]] = None, action_servers_components: typing.Optional[typing.List[kompass.components.component.Component]] = None, activate_on_start: typing.Optional[typing.List[kompass.components.component.Component]] = None, start_on_init: bool = False, component_name: str = 'monitor', callback_group: typing.Optional[typing.Union[rclpy.callback_groups.MutuallyExclusiveCallbackGroup, rclpy.callback_groups.ReentrantCallbackGroup]] = None, **kwargs)
:canonical: kompass.monitor.Monitor

Bases: {py:obj}`ros_sugar.monitor.Monitor`

```{autodoc2-docstring} kompass.monitor.Monitor
```

````{py:method} init_flags()
:canonical: kompass.monitor.Monitor.init_flags

```{autodoc2-docstring} kompass.monitor.Monitor.init_flags
```

````

````{py:method} init_variables()
:canonical: kompass.monitor.Monitor.init_variables

```{autodoc2-docstring} kompass.monitor.Monitor.init_variables
```

````

`````
