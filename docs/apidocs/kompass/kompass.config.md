# {py:mod}`kompass.config`

```{py:module} kompass.config
```

```{autodoc2-docstring} kompass.config
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`RobotFrames <kompass.config.RobotFrames>`
  - ```{autodoc2-docstring} kompass.config.RobotFrames
    :summary:
    ```
* - {py:obj}`RobotConfig <kompass.config.RobotConfig>`
  - ```{autodoc2-docstring} kompass.config.RobotConfig
    :summary:
    ```
* - {py:obj}`ComponentConfig <kompass.config.ComponentConfig>`
  - ```{autodoc2-docstring} kompass.config.ComponentConfig
    :summary:
    ```
````

### API

`````{py:class} RobotFrames
:canonical: kompass.config.RobotFrames

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} kompass.config.RobotFrames
```

````{py:method} from_json(json_obj: str | bytes | bytearray)
:canonical: kompass.config.RobotFrames.from_json

````

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: kompass.config.RobotFrames.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict)
:canonical: kompass.config.RobotFrames.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False)
:canonical: kompass.config.RobotFrames.from_yaml

````

````{py:method} to_json()
:canonical: kompass.config.RobotFrames.to_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: kompass.config.RobotFrames.has_attribute

````

````{py:method} get_attribute_type(attr_name: str)
:canonical: kompass.config.RobotFrames.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: kompass.config.RobotFrames.update_value

````

`````

`````{py:class} RobotConfig
:canonical: kompass.config.RobotConfig

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} kompass.config.RobotConfig
```

````{py:method} validate_params(_, value)
:canonical: kompass.config.RobotConfig.validate_params

```{autodoc2-docstring} kompass.config.RobotConfig.validate_params
```

````

````{py:method} from_json(json_obj: str | bytes | bytearray)
:canonical: kompass.config.RobotConfig.from_json

````

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: kompass.config.RobotConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict)
:canonical: kompass.config.RobotConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False)
:canonical: kompass.config.RobotConfig.from_yaml

````

````{py:method} to_json()
:canonical: kompass.config.RobotConfig.to_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: kompass.config.RobotConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str)
:canonical: kompass.config.RobotConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: kompass.config.RobotConfig.update_value

````

`````

````{py:class} ComponentConfig
:canonical: kompass.config.ComponentConfig

Bases: {py:obj}`ros_sugar.config.BaseComponentConfig`

```{autodoc2-docstring} kompass.config.ComponentConfig
```

````
