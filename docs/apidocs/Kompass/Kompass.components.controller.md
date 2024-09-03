# {py:mod}`Kompass.components.controller`

```{py:module} Kompass.components.controller
```

```{autodoc2-docstring} Kompass.components.controller
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ControllerInputs <Kompass.components.controller.ControllerInputs>`
  - ```{autodoc2-docstring} Kompass.components.controller.ControllerInputs
    :summary:
    ```
* - {py:obj}`ControllerOutputs <Kompass.components.controller.ControllerOutputs>`
  - ```{autodoc2-docstring} Kompass.components.controller.ControllerOutputs
    :summary:
    ```
* - {py:obj}`ControllerConfig <Kompass.components.controller.ControllerConfig>`
  - ```{autodoc2-docstring} Kompass.components.controller.ControllerConfig
    :summary:
    ```
* - {py:obj}`Controller <Kompass.components.controller.Controller>`
  - ```{autodoc2-docstring} Kompass.components.controller.Controller
    :summary:
    ```
````

### API

````{py:class} ControllerInputs
:canonical: Kompass.components.controller.ControllerInputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.controller.ControllerInputs
```

````

````{py:class} ControllerOutputs
:canonical: Kompass.components.controller.ControllerOutputs

Bases: {py:obj}`Kompass.topic.RestrictedTopicsConfig`

```{autodoc2-docstring} Kompass.components.controller.ControllerOutputs
```

````

````{py:class} ControllerConfig
:canonical: Kompass.components.controller.ControllerConfig

Bases: {py:obj}`Kompass.config.ComponentConfig`

```{autodoc2-docstring} Kompass.components.controller.ControllerConfig
```

````

`````{py:class} Controller(*, node_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[Kompass.components.controller.ControllerConfig] = None, inputs=None, outputs=None, **kwargs)
:canonical: Kompass.components.controller.Controller

Bases: {py:obj}`Kompass.components.component.Component`

```{autodoc2-docstring} Kompass.components.controller.Controller
```

````{py:property} run_type
:canonical: Kompass.components.controller.Controller.run_type
:type: Kompass.config.ComponentRunType

```{autodoc2-docstring} Kompass.components.controller.Controller.run_type
```

````

````{py:property} algorithm
:canonical: Kompass.components.controller.Controller.algorithm
:type: kompass_core.control.LocalPlannersID

```{autodoc2-docstring} Kompass.components.controller.Controller.algorithm
```

````

````{py:method} set_algorithm(algorithm_value: typing.Union[str, kompass_core.control.LocalPlannersID], keep_alive: bool = True) -> bool
:canonical: Kompass.components.controller.Controller.set_algorithm

```{autodoc2-docstring} Kompass.components.controller.Controller.set_algorithm
```

````

````{py:method} attach_callbacks() -> None
:canonical: Kompass.components.controller.Controller.attach_callbacks

```{autodoc2-docstring} Kompass.components.controller.Controller.attach_callbacks
```

````

````{py:method} init_variables()
:canonical: Kompass.components.controller.Controller.init_variables

```{autodoc2-docstring} Kompass.components.controller.Controller.init_variables
```

````

````{py:method} init_flags()
:canonical: Kompass.components.controller.Controller.init_flags

```{autodoc2-docstring} Kompass.components.controller.Controller.init_flags
```

````

````{py:method} execute_cmd(vx: float, vy: float, omega: float) -> bool
:canonical: Kompass.components.controller.Controller.execute_cmd

```{autodoc2-docstring} Kompass.components.controller.Controller.execute_cmd
```

````

````{py:method} main_action_callback(goal_handle) -> kompass_interfaces.action.ControlPath.Result
:canonical: Kompass.components.controller.Controller.main_action_callback

```{autodoc2-docstring} Kompass.components.controller.Controller.main_action_callback
```

````

`````
