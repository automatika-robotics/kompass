---
orphan: true
---

# {py:mod}`kompass.components.controller`

```{py:module} kompass.components.controller
```

```{autodoc2-docstring} kompass.components.controller
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ControllerConfig <kompass.components.controller.ControllerConfig>`
  - ```{autodoc2-docstring} kompass.components.controller.ControllerConfig
    :summary:
    ```
* - {py:obj}`Controller <kompass.components.controller.Controller>`
  - ```{autodoc2-docstring} kompass.components.controller.Controller
    :summary:
    ```
````

### API

````{py:class} ControllerConfig
:canonical: kompass.components.controller.ControllerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.controller.ControllerConfig
```

````

`````{py:class} Controller(*, node_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.controller.ControllerConfig] = None, inputs=None, outputs=None, **kwargs)
:canonical: kompass.components.controller.Controller

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.controller.Controller
```

````{py:method} create_all_timers()
:canonical: kompass.components.controller.Controller.create_all_timers

```{autodoc2-docstring} kompass.components.controller.Controller.create_all_timers
```

````

````{py:method} destroy_all_timers()
:canonical: kompass.components.controller.Controller.destroy_all_timers

```{autodoc2-docstring} kompass.components.controller.Controller.destroy_all_timers
```

````

````{py:property} run_type
:canonical: kompass.components.controller.Controller.run_type
:type: kompass.config.ComponentRunType

```{autodoc2-docstring} kompass.components.controller.Controller.run_type
```

````

````{py:property} tracked_point
:canonical: kompass.components.controller.Controller.tracked_point
:type: typing.Optional[geometry_msgs.msg.PoseStamped]

```{autodoc2-docstring} kompass.components.controller.Controller.tracked_point
```

````

````{py:property} local_plan
:canonical: kompass.components.controller.Controller.local_plan
:type: typing.Optional[nav_msgs.msg.Path]

```{autodoc2-docstring} kompass.components.controller.Controller.local_plan
```

````

````{py:property} interpolated_path
:canonical: kompass.components.controller.Controller.interpolated_path
:type: typing.Optional[nav_msgs.msg.Path]

```{autodoc2-docstring} kompass.components.controller.Controller.interpolated_path
```

````

````{py:property} direct_sensor
:canonical: kompass.components.controller.Controller.direct_sensor
:type: bool

```{autodoc2-docstring} kompass.components.controller.Controller.direct_sensor
```

````

````{py:property} algorithm
:canonical: kompass.components.controller.Controller.algorithm
:type: kompass_core.control.LocalPlannersID

```{autodoc2-docstring} kompass.components.controller.Controller.algorithm
```

````

````{py:method} set_algorithm(algorithm_value: typing.Union[str, kompass_core.control.LocalPlannersID], keep_alive: bool = True) -> bool
:canonical: kompass.components.controller.Controller.set_algorithm

```{autodoc2-docstring} kompass.components.controller.Controller.set_algorithm
```

````

````{py:method} attach_callbacks() -> None
:canonical: kompass.components.controller.Controller.attach_callbacks

```{autodoc2-docstring} kompass.components.controller.Controller.attach_callbacks
```

````

````{py:method} init_variables()
:canonical: kompass.components.controller.Controller.init_variables

```{autodoc2-docstring} kompass.components.controller.Controller.init_variables
```

````

````{py:method} init_flags()
:canonical: kompass.components.controller.Controller.init_flags

```{autodoc2-docstring} kompass.components.controller.Controller.init_flags
```

````

````{py:method} execute_cmd(vx: float, vy: float, omega: float) -> bool
:canonical: kompass.components.controller.Controller.execute_cmd

```{autodoc2-docstring} kompass.components.controller.Controller.execute_cmd
```

````

````{py:method} main_action_callback(goal_handle) -> kompass_interfaces.action.ControlPath.Result
:canonical: kompass.components.controller.Controller.main_action_callback

```{autodoc2-docstring} kompass.components.controller.Controller.main_action_callback
```

````

`````
