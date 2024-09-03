---
orphan: true
---

# {py:mod}`kompass.components.motion_server`

```{py:module} kompass.components.motion_server
```

```{autodoc2-docstring} kompass.components.motion_server
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`MotionServerConfig <kompass.components.motion_server.MotionServerConfig>`
  - ```{autodoc2-docstring} kompass.components.motion_server.MotionServerConfig
    :summary:
    ```
* - {py:obj}`MotionServer <kompass.components.motion_server.MotionServer>`
  - ```{autodoc2-docstring} kompass.components.motion_server.MotionServer
    :summary:
    ```
````

### API

````{py:class} MotionServerConfig
:canonical: kompass.components.motion_server.MotionServerConfig

Bases: {py:obj}`kompass.config.ComponentConfig`

```{autodoc2-docstring} kompass.components.motion_server.MotionServerConfig
```

````

`````{py:class} MotionServer(*, node_name: str, config: typing.Optional[kompass.components.motion_server.MotionServerConfig] = None, config_file: typing.Optional[str] = None, robot_cmd_topic: typing.Optional[kompass.topic.Topic] = None, robot_odom_topic: typing.Optional[kompass.topic.Topic] = None, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, **kwargs)
:canonical: kompass.components.motion_server.MotionServer

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.motion_server.MotionServer
```

````{py:property} run_type
:canonical: kompass.components.motion_server.MotionServer.run_type
:type: kompass.config.ComponentRunType

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.run_type
```

````

````{py:method} attach_callbacks() -> None
:canonical: kompass.components.motion_server.MotionServer.attach_callbacks

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.attach_callbacks
```

````

````{py:method} init_variables() -> None
:canonical: kompass.components.motion_server.MotionServer.init_variables

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.init_variables
```

````

````{py:method} main_action_callback(goal_handle: kompass_interfaces.action.MotionRecording.Goal) -> kompass_interfaces.action.MotionRecording.Result
:canonical: kompass.components.motion_server.MotionServer.main_action_callback

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.main_action_callback
```

````

````{py:method} run_motion_response_tests(msg, **_) -> None
:canonical: kompass.components.motion_server.MotionServer.run_motion_response_tests

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.run_motion_response_tests
```

````

````{py:method} generate_basic_ctr_tests(number_of_steps: int) -> typing.List[typing.Dict]
:canonical: kompass.components.motion_server.MotionServer.generate_basic_ctr_tests

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.generate_basic_ctr_tests
```

````

````{py:method} send_test(test: numpy.ndarray, test_name: str, number_steps: int) -> bool
:canonical: kompass.components.motion_server.MotionServer.send_test

```{autodoc2-docstring} kompass.components.motion_server.MotionServer.send_test
```

````

`````
