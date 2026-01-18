---
orphan: true
---

# {py:mod}`kompass.components.map_server`

```{py:module} kompass.components.map_server
```

```{autodoc2-docstring} kompass.components.map_server
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`MapServerConfig <kompass.components.map_server.MapServerConfig>`
  - ```{autodoc2-docstring} kompass.components.map_server.MapServerConfig
    :summary:
    ```
* - {py:obj}`MapServer <kompass.components.map_server.MapServer>`
  - ```{autodoc2-docstring} kompass.components.map_server.MapServer
    :summary:
    ```
````

### API

````{py:class} MapServerConfig
:canonical: kompass.components.map_server.MapServerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.map_server.MapServerConfig
```

````

`````{py:class} MapServer(component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.map_server.MapServerConfig] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.map_server.MapServer

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.map_server.MapServer
```

````{py:method} init_variables()
:canonical: kompass.components.map_server.MapServer.init_variables

```{autodoc2-docstring} kompass.components.map_server.MapServer.init_variables
```

````

````{py:method} create_all_timers()
:canonical: kompass.components.map_server.MapServer.create_all_timers

```{autodoc2-docstring} kompass.components.map_server.MapServer.create_all_timers
```

````

````{py:method} create_all_services()
:canonical: kompass.components.map_server.MapServer.create_all_services

```{autodoc2-docstring} kompass.components.map_server.MapServer.create_all_services
```

````

````{py:method} destroy_all_timers()
:canonical: kompass.components.map_server.MapServer.destroy_all_timers

```{autodoc2-docstring} kompass.components.map_server.MapServer.destroy_all_timers
```

````

````{py:method} destroy_all_services()
:canonical: kompass.components.map_server.MapServer.destroy_all_services

```{autodoc2-docstring} kompass.components.map_server.MapServer.destroy_all_services
```

````

````{py:method} convert_map_from_file() -> bool
:canonical: kompass.components.map_server.MapServer.convert_map_from_file

```{autodoc2-docstring} kompass.components.map_server.MapServer.convert_map_from_file
```

````

`````
