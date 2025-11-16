---
orphan: true
---

# {py:mod}`kompass.components.planner`

```{py:module} kompass.components.planner
```

```{autodoc2-docstring} kompass.components.planner
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`PlannerConfig <kompass.components.planner.PlannerConfig>`
  - ```{autodoc2-docstring} kompass.components.planner.PlannerConfig
    :summary:
    ```
* - {py:obj}`Planner <kompass.components.planner.Planner>`
  - ```{autodoc2-docstring} kompass.components.planner.Planner
    :summary:
    ```
````

### API

````{py:class} PlannerConfig
:canonical: kompass.components.planner.PlannerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.planner.PlannerConfig
```

````

`````{py:class} Planner(component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.planner.PlannerConfig] = None, inputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.planner.Planner

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.planner.Planner
```

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.planner.Planner.config_from_file

```{autodoc2-docstring} kompass.components.planner.Planner.config_from_file
```

````

````{py:method} init_variables()
:canonical: kompass.components.planner.Planner.init_variables

```{autodoc2-docstring} kompass.components.planner.Planner.init_variables
```

````

````{py:method} create_all_services()
:canonical: kompass.components.planner.Planner.create_all_services

```{autodoc2-docstring} kompass.components.planner.Planner.create_all_services
```

````

````{py:method} destroy_all_services()
:canonical: kompass.components.planner.Planner.destroy_all_services

```{autodoc2-docstring} kompass.components.planner.Planner.destroy_all_services
```

````

````{py:method} main_service_callback(request: kompass_interfaces.srv.PlanPath.Request, response: kompass_interfaces.srv.PlanPath.Response)
:canonical: kompass.components.planner.Planner.main_service_callback

```{autodoc2-docstring} kompass.components.planner.Planner.main_service_callback
```

````

````{py:method} reached_point(goal_point: kompass_core.models.RobotState, tolerance: kompass_interfaces.msg.PathTrackingError) -> bool
:canonical: kompass.components.planner.Planner.reached_point

```{autodoc2-docstring} kompass.components.planner.Planner.reached_point
```

````

````{py:method} main_action_callback(goal_handle: kompass_interfaces.action.PlanPath.Goal)
:canonical: kompass.components.planner.Planner.main_action_callback

```{autodoc2-docstring} kompass.components.planner.Planner.main_action_callback
```

````

`````
