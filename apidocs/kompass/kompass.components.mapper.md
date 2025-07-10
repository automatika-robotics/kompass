---
orphan: true
---

# {py:mod}`kompass.components.mapper`

```{py:module} kompass.components.mapper
```

```{autodoc2-docstring} kompass.components.mapper
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`LocalMapperConfig <kompass.components.mapper.LocalMapperConfig>`
  - ```{autodoc2-docstring} kompass.components.mapper.LocalMapperConfig
    :summary:
    ```
* - {py:obj}`LocalMapper <kompass.components.mapper.LocalMapper>`
  - ```{autodoc2-docstring} kompass.components.mapper.LocalMapper
    :summary:
    ```
````

### API

````{py:class} LocalMapperConfig
:canonical: kompass.components.mapper.LocalMapperConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.mapper.LocalMapperConfig
```

````

`````{py:class} LocalMapper(*, component_name: str, config_file: typing.Optional[str] = None, config: typing.Optional[kompass.components.mapper.LocalMapperConfig] = None, inputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, outputs: typing.Optional[typing.Dict[str, kompass.components.ros.Topic]] = None, **kwargs)
:canonical: kompass.components.mapper.LocalMapper

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.mapper.LocalMapper
```

````{py:method} init_variables()
:canonical: kompass.components.mapper.LocalMapper.init_variables

```{autodoc2-docstring} kompass.components.mapper.LocalMapper.init_variables
```

````

````{py:method} publish_data()
:canonical: kompass.components.mapper.LocalMapper.publish_data

```{autodoc2-docstring} kompass.components.mapper.LocalMapper.publish_data
```

````

`````
