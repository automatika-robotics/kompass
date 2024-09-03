# {py:mod}`Kompass.config`

```{py:module} Kompass.config
```

```{autodoc2-docstring} Kompass.config
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`RobotFrames <Kompass.config.RobotFrames>`
  - ```{autodoc2-docstring} Kompass.config.RobotFrames
    :summary:
    ```
* - {py:obj}`RobotConfig <Kompass.config.RobotConfig>`
  - ```{autodoc2-docstring} Kompass.config.RobotConfig
    :summary:
    ```
* - {py:obj}`ComponentConfig <Kompass.config.ComponentConfig>`
  - ```{autodoc2-docstring} Kompass.config.ComponentConfig
    :summary:
    ```
````

### API

`````{py:class} RobotFrames
:canonical: Kompass.config.RobotFrames

Bases: {py:obj}`ros_sugar.base_attrs.BaseAttrs`

```{autodoc2-docstring} Kompass.config.RobotFrames
```

````{py:method} from_json(json_obj: str | bytes | bytearray)
:canonical: Kompass.config.RobotFrames.from_json

````

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: Kompass.config.RobotFrames.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict)
:canonical: Kompass.config.RobotFrames.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False)
:canonical: Kompass.config.RobotFrames.from_yaml

````

````{py:method} to_json()
:canonical: Kompass.config.RobotFrames.to_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: Kompass.config.RobotFrames.has_attribute

````

````{py:method} get_attribute_type(attr_name: str)
:canonical: Kompass.config.RobotFrames.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: Kompass.config.RobotFrames.update_value

````

`````

`````{py:class} RobotConfig
:canonical: Kompass.config.RobotConfig

Bases: {py:obj}`ros_sugar.base_attrs.BaseAttrs`

```{autodoc2-docstring} Kompass.config.RobotConfig
```

````{py:method} validate_params(_, value)
:canonical: Kompass.config.RobotConfig.validate_params

```{autodoc2-docstring} Kompass.config.RobotConfig.validate_params
```

````

````{py:method} from_json(json_obj: str | bytes | bytearray)
:canonical: Kompass.config.RobotConfig.from_json

````

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: Kompass.config.RobotConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict)
:canonical: Kompass.config.RobotConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False)
:canonical: Kompass.config.RobotConfig.from_yaml

````

````{py:method} to_json()
:canonical: Kompass.config.RobotConfig.to_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: Kompass.config.RobotConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str)
:canonical: Kompass.config.RobotConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: Kompass.config.RobotConfig.update_value

````

`````

````{py:class} ComponentConfig
:canonical: Kompass.config.ComponentConfig

Bases: {py:obj}`ros_sugar.config.BaseComponentConfig`

```{autodoc2-docstring} Kompass.config.ComponentConfig
```

````
