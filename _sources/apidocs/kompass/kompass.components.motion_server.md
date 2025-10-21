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

`````{py:class} MotionServer(*, component_name: str, config: typing.Optional[kompass.components.motion_server.MotionServerConfig] = None, config_file: typing.Optional[str] = None, robot_cmd_topic: typing.Optional[kompass.components.ros.Topic] = None, robot_odom_topic: typing.Optional[kompass.components.ros.Topic] = None, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, **kwargs)
:canonical: kompass.components.motion_server.MotionServer

Bases: {py:obj}`kompass.components.component.Component`

```{autodoc2-docstring} kompass.components.motion_server.MotionServer
```

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

````{py:method} custom_on_configure()
:canonical: kompass.components.motion_server.MotionServer.custom_on_configure

````

````{py:property} robot
:canonical: kompass.components.motion_server.MotionServer.robot
:type: kompass.config.RobotConfig

````

````{py:property} run_type
:canonical: kompass.components.motion_server.MotionServer.run_type
:type: kompass.config.ComponentRunType

````

````{py:property} inputs_keys
:canonical: kompass.components.motion_server.MotionServer.inputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:property} outputs_keys
:canonical: kompass.components.motion_server.MotionServer.outputs_keys
:type: typing.List[kompass.components.defaults.TopicsKeys]

````

````{py:method} inputs(**kwargs)
:canonical: kompass.components.motion_server.MotionServer.inputs

````

````{py:method} outputs(**kwargs)
:canonical: kompass.components.motion_server.MotionServer.outputs

````

````{py:method} config_from_file(config_file: str)
:canonical: kompass.components.motion_server.MotionServer.config_from_file

````

````{py:property} odom_tf_listener
:canonical: kompass.components.motion_server.MotionServer.odom_tf_listener
:type: typing.Optional[ros_sugar.tf.TFListener]

````

````{py:property} scan_tf_listener
:canonical: kompass.components.motion_server.MotionServer.scan_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} depth_tf_listener
:canonical: kompass.components.motion_server.MotionServer.depth_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:property} pc_tf_listener
:canonical: kompass.components.motion_server.MotionServer.pc_tf_listener
:type: ros_sugar.tf.TFListener

````

````{py:method} get_transform_listener(src_frame: str, goal_frame: str) -> ros_sugar.tf.TFListener
:canonical: kompass.components.motion_server.MotionServer.get_transform_listener

````

````{py:method} in_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.motion_server.MotionServer.in_topic_name

````

````{py:method} out_topic_name(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[str, typing.List[str], None]
:canonical: kompass.components.motion_server.MotionServer.out_topic_name

````

````{py:method} get_in_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.motion_server.MotionServer.get_in_topic

````

````{py:method} get_out_topic(key: typing.Union[str, kompass.components.defaults.TopicsKeys]) -> typing.Union[kompass.components.ros.Topic, typing.List[kompass.components.ros.Topic], None]
:canonical: kompass.components.motion_server.MotionServer.get_out_topic

````

````{py:method} get_callback(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> typing.Optional[kompass.callbacks.GenericCallback]
:canonical: kompass.components.motion_server.MotionServer.get_callback

````

````{py:method} get_publisher(key: typing.Union[str, kompass.components.defaults.TopicsKeys], idx: int = 0) -> ros_sugar.io.Publisher
:canonical: kompass.components.motion_server.MotionServer.get_publisher

````

````{py:method} callbacks_inputs_check(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: kompass.components.motion_server.MotionServer.callbacks_inputs_check

````

`````
