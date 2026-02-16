# Motion Server

**System validation, calibration, and motion data recording.**

Unlike the core navigation components, the [MotionServer](../apidocs/kompass/kompass.components.motion_server.md) does not plan paths or avoid obstacles. Instead, it provides essential utilities for validating your robot's physical performance and tuning its control parameters.

It serves two primary purposes:
1.  **Automated Motion Tests:** Executing pre-defined maneuvers (Step response, Circles) to calibrate the robot's motion model on new terrain.
2.  **Black Box Recording:** capturing synchronized control commands and robot responses (Pose/Velocity) during operation for post-analysis.

## Key Capabilities

The Motion Server is a versatile tool for system identification.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`science` Motion Calibration</span> - **Automated Tests.** Execute step inputs or circular paths automatically to measure the robot's real-world response vs. the theoretical model.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`radio_button_checked` Data Recording</span> - **"Black Box" Logging.** Record exact control inputs and odometry outputs synchronized in time. Essential for tuning controller gains or debugging tracking errors.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`loop` Closed-Loop Validation</span> - **Input/Output Compare.** Can act as both the *source* of commands (during tests) and the *sink* for recording, allowing you to validate the entire control pipeline (e.g., passing commands through the Drive Manager).

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`event` Event-Triggered</span> - **Dynamic Execution.** Start recording or launch a calibration sequence automatically based on external events (e.g., "Terrain Changed" or "Slip Detected").

## Run Types

Choose how you want to utilize the server:

```{list-table}
:widths: 20 80
* - **{material-regular}`schedule;1.2em;sd-text-primary` Timed**
  - **Auto-Start Tests.** Automatically launches the configured motion tests periodically after the component starts.

* - **{material-regular}`touch_app;1.2em;sd-text-primary` Event**
  - **Triggered Tests.** Waits for a `True` signal on the `run_tests` input topic to launch the calibration sequence.

* - **{material-regular}`hourglass_top;1.2em;sd-text-primary` Action Server**
  - **On-Demand Recording.** Offers a `MotionRecording` ROS2 Action. Allows you to start/stop recording specific topics for a set duration via an Action Goal.

```

```{note}
The available motion tests include Step tests and Circle test and can be configured by adjusting the [MotionServerConfig](../apidocs/kompass/kompass.components.motion_server.md)
```

## Interface

### Inputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1

* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">run_tests</span>
  - `std_msgs.msg.Bool`
  - 1
  - `/run_tests`

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">command</span>
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - 1
  - `/cmd_vel` (`Twist`)

* - location
  - [`nav_msgs.msg.Odometry`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html), [`geometry_msgs.msg.PoseStamped`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/PoseStamped.html), [`geometry_msgs.msg.Pose`](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Pose.html)
  - 1
  - `/odom` (`Odometry`)

```

### Outputs

```{list-table}
:widths: 10 40 10 40
:header-rows: 1

* - Key Name
  - Allowed Types
  - Number
  - Default

* - <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">robot_command</span>
  - [`geometry_msgs.msg.Twist`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html), [`geometry_msgs.msg.TwistStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TwistStamped.html)
  - 1
  - `/cmd_vel` (`Twist`)

```

:::{admonition} Dual Command Topics
:class: note
The **Command** topic appears in both Inputs and Outputs, but serves different roles:

1. **Output (`robot_command`):** Used when the Motion Server is *generating* commands (Running Tests).
2. **Input (`command`):** Used when the Motion Server is *listening* (Recording).

*Power User Tip:* You can wire these differently to test specific components. For example, connect the Motion Server **Output** to the Drive Manager's input, and connect the Drive Manager's output back to the Motion Server **Input**. This records exactly how the Drive Manager modifies your commands (e.g., smoothing or limiting).
:::

## Usage Example

```python
from kompass.components import MotionServer, MotionServerConfig
from kompass.ros import Topic

# 1. Configuration
# Define the test parameters (e.g., a 1.0m/s step input)
my_config = MotionServerConfig(
    step_test_velocity=1.0,
    step_test_duration=5.0
)

# 2. Instantiate
motion_server = MotionServer(component_name="motion_server", config=my_config)

# 3. Setup for Event-Based Testing
motion_server.run_type = "Event"
motion_server.inputs(run_tests=Topic(name="/start_calibration", msg_type="Bool"))

```

## See Next

Explore how you can use the MotionServer to run automated motions testing and recording with you navigation system.

:::{button-link} ../tutorials/automated_motion_test.html
:color: primary
:ref-type: doc
:outline:
Check the Automated Motion Tests Tutorial →
:::
