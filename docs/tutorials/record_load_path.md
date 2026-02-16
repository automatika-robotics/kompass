# Path Recording & Replay

**Save successful paths and re-execute them on demand.**

Sometimes you don't need a dynamic planner to calculate a new path every time. In scenarios like **routine patrols, warehousing, or repeatable docking**, it is often more reliable to record a "golden path" once and replay it exactly.

The **Planner** component facilitates this via three ROS 2 services:
1.  `save_plan_to_file`: Saves the currently active plan (or recorded history) to a CSV file.
2.  `load_plan_from_file`: Loads a CSV file and publishes it as the current global plan.
3.  `start_path_recording`: Starts recording the robot's actual odometry history to be saved later.



## The Recipe

This recipe sets up a basic navigation stack but exposes the **Save/Load/Record Services** to the Web UI instead of the standard "Click-to-Nav" action.

**Create a file named `path_recorder.py`:**

```python
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Kompass Imports
from kompass.robot import (
    AngularCtrlLimits, LinearCtrlLimits, RobotGeometry, RobotType, RobotConfig, RobotFrames
)
from kompass.components import (
    DriveManager, DriveManagerConfig, Planner, PlannerConfig,
    MapServer, MapServerConfig, TopicsKeys, Controller
)
from kompass.ros import Topic, Launcher, ServiceClientConfig
from kompass.control import ControllersID
from kompass_interfaces.srv import PathFromToFile, StartPathRecording

def run_path_recorder():
    kompass_sim_dir = get_package_share_directory(package_name="kompass_sim")

    # 1. Robot Configuration
    my_robot = RobotConfig(
        model_type=RobotType.DIFFERENTIAL_DRIVE,
        geometry_type=RobotGeometry.Type.CYLINDER,
        geometry_params=np.array([0.1, 0.3]),
        ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
        ctrl_omega_limits=AngularCtrlLimits(max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3),
    )

    # 2. Configure Components
    planner = Planner(component_name="planner", config=PlannerConfig(loop_rate=1.0))
    planner.run_type = "Timed"

    controller = Controller(component_name="controller")
    controller.algorithm = ControllersID.PURE_PURSUIT
    controller.direct_sensor = True # Use direct sensor for simple obstacle checks

    driver = DriveManager(
        component_name="drive_manager",
        config=DriveManagerConfig(critical_zone_distance=0.05)
    )

    # Handle message types (Twist vs TwistStamped)
    cmd_msg_type = "TwistStamped" if os.environ.get("ROS_DISTRO") in ["rolling", "jazzy", "kilted"] else "Twist"
    driver.outputs(robot_command=Topic(name="/cmd_vel", msg_type=cmd_msg_type))

    map_server = MapServer(
        component_name="global_map_server",
        config=MapServerConfig(
            map_file_path=os.path.join(kompass_sim_dir, "maps", "turtlebot3_webots.yaml"),
            grid_resolution=0.5
        )
    )

    # 3. Define Services for UI Interaction
    save_path_srv = ServiceClientConfig(
        name=f"{planner.node_name}/save_plan_to_file", srv_type=PathFromToFile
    )
    load_path_srv = ServiceClientConfig(
        name=f"{planner.node_name}/load_plan_from_file", srv_type=PathFromToFile
    )
    start_path_recording = ServiceClientConfig(
        name=f"{planner.node_name}/start_path_recording", srv_type=StartPathRecording
    )

    # 4. Launch
    launcher = Launcher()
    launcher.kompass(
        components=[map_server, planner, driver, controller],
        multiprocessing=True,
    )

    odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
    launcher.inputs(location=odom_topic)

    launcher.robot = my_robot
    launcher.frames = RobotFrames(world="map", odom="map", scan="LDS-01")

    # 5. Enable UI
    launcher.enable_ui(
        inputs=[save_path_srv, load_path_srv, start_path_recording],
        outputs=[
            map_server.get_out_topic(TopicsKeys.GLOBAL_MAP),
            odom_topic,
            planner.get_out_topic(TopicsKeys.GLOBAL_PLAN),
        ],
    )

    launcher.bringup()

if __name__ == "__main__":
    run_path_recorder()

```

---

## Workflow: Two Ways to Generate a Path

Once the recipe is running and you have the Kompass UI open (`http://0.0.0.0:5001`), you can generate a path using either the planner or by manually driving the robot.

### Option A: Save a Computed Plan

<span class="sd-text-primary" style="font-weight: bold; font-size: 1.2em;">Use this if you want to save the path produced by the global planner and you want to "freeze" that exact path for future use.</span>

1. **Generate Plan:** Trigger the planner (e.g., via the `/clicked_point` input on the UI).
2. **Verify:** Check if the generated path looks good...
3. **Save:** In the UI Inputs panel, go to the `planner/save_plan_to_file` in the  *Inputs* panel
* Enter the **file_location:** `/tmp/` and **file_name:** `computed_path.json`
* Click **Send**.


### Option B: Record a Driven Path (Teleop)

<span class="sd-text-primary" style="font-weight: bold; font-size: 1.2em;">Use this if you want the robot to follow a human-demonstrated path (e.g., a specific maneuver through a tight doorway).</span>

1. **Start Recording:** In the UI Inputs panel, select `planner/start_path_recording`.
* **recording_time_step:** `0.1` (Records a point every 0.1 seconds).
* Click **Call**.


2. **Drive:** Use your keyboard or joystick (e.g., `ros2 run teleop_twist_keyboard teleop_twist_keyboard`) to drive the robot along the desired route.
3. **Save:** When finished, select `planner/save_plan_to_file`.
* Enter the **file_location:** `/tmp/` and **file_name:** `computed_path.csv`
* Click **Send**.
*(Note: Calling save automatically stops the recording process).*


---

## Replay the Path

Now that you have your "Golden Path" saved (either computed or recorded), you can replay it anytime.

1. **Restart:** You can restart the stack or simply clear the current plan.
2. **Load:** In the UI **Inputs** panel, select `planner/load_plan_from_file`.
* **file_location:** `/tmp/`
* **file_name:** `driven_path.csv` (or `computed_path.csv`)
* Click **Send**.


**Result:**
The planner immediately loads the CSV file and publishes it as the **Global Plan**. The **Controller** receives this path and begins executing it immediately, retracing your steps exactly.

---

## Use Cases

* **Routine Patrols:** Record a perfect lap around a facility and replay it endlessly.
* **Complex Docking:** Manually drive a complex approach to a charging station, save the plan, and use it for reliable docking.
* **Multi-Robot Coordination:** Share a single "highway" path file among multiple robots to ensure they stick to verified lanes.

