# Motion Server

[MotionServer](../apidocs/Kompass/Kompass.components.motion_server.md) offers additional motion related services for testing and recording. Unlike the rest of the navigation components, the Motion Server does not perform a core navigation task but offers complementary functionalities essential for validating and calibrating a navigation system.

Motion Server contains a set of basic motion tests. In each test, the MotionServer sends commands to the robot to perform the pre-configured motion and records both the sent command and the robot response (velocity and pose). This is used to test the motion response on new terrain or calibrate the robot motion model.

```{note}
The available motion tests include Step tests and Circle test and can be configured by adjusting the [MotionServerConfig](../apidocs/kompass/kompass.components.motion_server.md)
```

MotionServer offers a Motion Recording service as a ROS2 action which allows to record the control commands and the response of the robot during the navigation.

```{tip}
Launch the MotionServer as a **Timed** component to launch the basic motion tests automatically, or as a **Event** component to launch the tests when a trigger is received
```

```{tip}
Launch the MotionServer as an **ActionServer** component and send a request to record your robot's motion at any time during the navigation.
```

```{tip}
The Motion Recording action can also be configured to start based on an external event
```

## Available Run Types

```{list-table}
:widths: 10 80
* - **Timed**
  - Launches an automated test periodically after start

* - **Event**
  - Launches automated testing when a trigger is received on RUN input

* - **ActionServer**
  - Offers a MotionRecording ROS action to record motion for location and control commands topics for given recording period
```


## Inputs:

```{list-table}
:widths: 10 30 15 20 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Number of Topics
  - Default Value

* - **run**
  - Start running tests
  - `Bool`
  - 1
  - `Topic(name="/run_tests", msg_type="Bool")`

* - **robot_command**
  - One control command
  - `Twist`
  - 1
  - `Topic(name="/cmd_vel", msg_type="Twist")`

```

## Outputs:

```{list-table}
:widths: 10 30 15 20
:header-rows: 1
* - Key
  - Description
  - Accepted Types
  - Default Value

* - **command**
  - One control command
  - `Twist`
  - `Topic(name="/control", msg_type="Twist")`

```

```{note}
Topic for _Control Command_ is both in MotionServer inputs and outputs:
- The output is used when running automated testing (i.e. sending the commands directly from the MotionServer).
- The input is used to purly record motion and control from external sources (example: recording output from Controller).
- Different command topics can be configured for the input and the output. For example: to test the DriveManager, the control command from MotionServer output can be sent to the DriveManager, then the DriveManager output can be configured as the MotionServer input for recording.
```
