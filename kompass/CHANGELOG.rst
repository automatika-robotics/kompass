^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kompass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2026-10-20)
------------------
* (chore) Updates sugarcoat minimum required version
* (fix) Fixes fetching latest robot state in planner and handles OMPL approximate solutions
* (fix) Adds timeout of robot state check in Planner reached_end method
* (fix) Adds a flag in the MissionManager to track if the routine cursor is lost
* (fix) Adds clearing for callback last message on Planner goal clear
* (fix) Forwards mission waypoint to the planner as they come as the planner already handles its own TFs
* (fix) Ensures the robot is stopped when controller core reports that end is reached
* (fix) Adds additional cleanup to previous goal points on planner action end
* (fix) Adds wait for robot state in planner reached_end check
* (chore) Fixes tolerances in turtlebot recipe
* (fix) Respects a TwistArray time_step and ensure safety margin when executing a rotate in place action
* (fix) Adds missing TF for PointStamped callback
* (fix) Adds missing TF to world frame for goal point Planner input required in TIMED/Event mode
* (fix) Checks for node health when waiting for an input, ends the loop on node shutting down
* (chore) Fixes setting algorithm string name and not class name in health status message
* (fix) Corrects path length calculation
* (chore) Removes the dead Monitor subclass
* (fix) Builds the planner's own action client once instead of per trigger
* (fix) Fixes minimum tolernce check in planner to honor the lowest default
* (fix) Fixes execution of return_to_start policy on mission timeout
* (fix) Correctly handles sending local vs global frame in rgbd follower, local by default unless requested in config
* (fix) Checks robot stopping using location change when velocity feedback is not available
* (fix) Refreshes the safety verdict every tick in DriveManager to avoid stale emergency_stop flags
* (fix) Reads a clicked waypoint from the UI node
* (fix) Clears Controller core path when tracking stops and ends the tracking goal
* (fix) Adds a reentrant threading lock to use on every touch of kompass-core controller
* (fix) Ends a mission whose routine cannot be read and bounds the Monitor calls
* (fix) Makes the latched map QoS a default that a recipe or config file can override
* (fix) Applied missing TF to the robot state
* (feature) Adds custom UI Task element for the MissionManager
* (feature) Adds pause/resume to MissionManager and accepts waypoints from the UI
* (fix) Publishes the final mission status once before cleanup, with completed, canceled and aborted status
* (fix) Skips the pause condition at the last waypoint of a mission
* (fix) Adds missing tf2 geometry messages dependency to package.xml
* (fix) Passes global plans to the core without length check, core handles the check
* (feature) Adds UI element for the mission status
* (fix) Re-evaluates the safety check every tick so a stale sensor no longer latches the emergency stop
* (fix) Returns the (bool, str) action contract from the planner trigger_main_action_server action
* (fix) Publishes the global map once with a latched QoS and sets it on the planner when received
* (fix) Adds float conversion for map metadata to avoid errors when int-like values are present
* (fix) Turns pointcloud filtering min_height into a config param in DriveManager
* (fix) Restrict values to float before publishing twist array
* (fix) Adds stop_timeout param to stop_robot action in DriveManager
* (feature) Adds MissionManager to executable types
* (feature) Adds mission status message
* (refactor) Changes return type of DriveManager component actions to ActionReturnType instead of bool
* (feature) Adds action to step the robot in DriveManager
* (feature) Adds Controller component action to stop path tracking
* (refactor) Fixes set_algorithm to return ActionReturnType instead of bool
* (feature) Adds MissionManager to Kompass to handle multi-goal missions execution
* (fix) Adds missing _img_size to trackings callback
* (fix) Fixes vision follower to avoid blocking an RGB follower for non-available depth info
* (fix) Handles -inf values from range sensors and treating them as nearby obstacles
* (fix) Blocks command publishing in DriveManager if collision checker is not initialized
* (fix) Fixes odom callback _process function for correct calculation of speed
* (refactor) Standardizes angular ctrl limit params with sugarcoat and core
* (refactor) Makes the name of min_vel param consistent with core and sugarcoat
* (feature) Adds handling range properly in drive manager and removes bare floats
* (fix) Waits for depth every time inputs are updated in vision follower
* (fix) Fixes getting boxes of the correct labels for vision following sorted by detection score when multiple labels are present
* (fix) Fixes depth age check wrt detections msg by caching stamp so it survives clear_last
* (refactor) Uses component level tf wait method to wait for camera intrinsics
* (feature) Adds external depth topics for aligned depth values to vision follower
* (feature) Adds external depth input to the controller
* (feature) Adds external depth topic registration to the planner (aligned depth and pointclouds)
* (feature) Adds external depth mixin for callbacks that were bringing their own depth to consume external depth topics
* (refactor) Makes drive manager use the native buffer layout for pointcloud and component level tf wait
* (refactor) Refactors mapper with component level sensor tf wait and native buffer layout
* (feature) Adds helper methods to get topic TF and return sensor config for spatial sensors
* (feature) Adds vision depth to default keys
* (fix) Adds refreshing frame mode on mission start to check if global frame is available
* (feature) Adds sending depth metadata directly to core for the zero copy path and fixes errors in trackings callback
* (refactor) Read depth image info only at setup for rgbd follower, cache otherwise
* (fix) Adds error logging and failure health status if inputs are not available to local mapper rather than silent fail
* (fix) Fixes refreshing inputs and outputs to components after init with a launch guard
* (refactor) Removes upstreamed camera info datatype and uses CameraIntrinsics from sugarcoat
* (feature) Adds multi sensor fusion for point cloud based local mapping
* (feature) Adds multiple pointcloud fused emergency stop checking
* (feature) Adds exception based safety stop in unblocking maneuver as well
* (feature) Adds handling in drive manager to stop the robot if an exception is thrown from inside critical zone checker
* (fix) Fixes resetting the sensor mount flag
* (fix) Fixes checking for global plan availability in TIMED mode
* (fix) Resets sensor mount flag when changing direct_sensor property
  Reset is needed when this property is changed during runtime
* (fix) Uses the new tf resolve utility from parent to check if global TF is available for plan, local map and pointcloud in Controller
* (feature) Adds utility method to resolve TF for a given input key
* (fix) Fix frame check when no global plan subscription is available
* (fix) Uses the explicit fields update to configure the controller's algorithm to correct the sensor pose mount setting
* (fix) Checks input data type using the callback type rather than the sensor_data to avoid checking against None value if the message is not published yet
* (fix) Fixes criticalzone checker init for failure case, when gpu is requested
* (fix) Changes imread input to string for Python 3.8 compatibility
* (fix) Updates the controller to respect kompass-core contract for sensor input frames based on sensor type
* (fix) Checks for global plan frame in Controller before execution
* (fix) Removes setting the transformation to the input callback to be applied only on kompass-core side
* Contributors: ahr, mkabtoul

0.7.0 (2026-08-05)
------------------
* (chore) Updates minimum required kompass-core version
* (feature) Adds import error message on broken imports for backward compatibility
* (fix) Handles multi-input case in failure source setting
* (fix) Fixes input check in planner
* (refactor) Import LaserScanData from Sugarcoat
* (fix) Fixes checking if input is available and removes call of deleted method
* (feature) Declares failure on non-valid spatial sensor data type in emergency stop check
* (refactor) Imports pointcloud utilities from the refactored kompass_core module
* (fix) Uses new RobotGeometryType refactored class in recipe
* (refactor) Uses upstreamed LaserScanData and PointCloudData from Sugarcoat
* (refactor) Passes ranges, angle and points to the core controller instead of removed LaserScanData and PointCloudData
* (refactor) Updates component to use refactored AugularCtrlLimits, LinearCtrlLimits and RobotType from kompass-core
* (refactor) Cleans up imports for refactored types in kompass-core
* (fix) Removes usage of deleted config params and fixes method name typo
* (fix) Makes update_topics non-mutating to avoid mutating stack defaults
* (fix) Adds missing allowed SERVER runtype to Planner
* (refactor) Updates all components to access RobotConfig from self.robot instead of Optional self.config.robot
* (refactor) Updates robot property to raise an error if None RobotConfig is provided to a Kompass component
* (refactor) Updates recipe and config files with new RobotFrames
* (feature) Adds property to get kompass_core geometry type from RobotConfig
* (refactor) Moves RobotConfig to Sugarcoat
* (fix) Fixes wait condition for robot location to handle non-stamped types
* (fix) Removes 'odom' as the backup frame for the map and uses the robot location message frame_id instead
* (refactor) Updates components to use the new transformation lookup method inside the callback
* (refactor) Removes specific tf listeners and updates to use a generic tf input listener
* (refactor) Uses LaserScan and PointCloud callbacks upstreamed to Sugarcoat
* (refactor) Upstreams RobotFrames to Sugarcoat
* (fix) Fixes None value check for ndarray type
* Contributors: ahr, mkabtoul

0.6.0 (2026-06-05)
------------------
* (chore) Bumps required sugarcoat version
* (fix) Fixes controller mode change to handle plugin-based callbacks
* Contributors: ahr, mkabtoul

0.5.0 (2026-05-02)
------------------
* (fix) Removes type hint from map server to fix for rolling api change
* (fix) Fixes sending algorithm name error in health status
* (fix) Returns robot state regardless of the TF since it will be blocked in the main update_state if required and not available
* (fix) Fixes reading robot state in update state
* (chore) Updates kompaass-core and sugarcoat minimum versions
* (chore) Update turtlebot test recipe
* (fix) Fixes state update in path control
* (refactor) Fixes vision follower execution using rgb images
* (refactor) Removes sensor data from vision follower loop step
* (refactor) Splits update_state into two methods for updating sensor data and updating robot state
* (fix) Reduces default value for topic subscription timeout and timestep
* (refactor) Updates tests to use new controller helper modules
* (refactor) Cleans up the controller component and uses new helper modules
* (refactor) Moves all vision following lifecycle methods to a helper module
* (refactor) Moves gathering obstacles method into utils
* (refactor) Moves Controller status/mode enums into a helper module
* (fix) Fixes method name typo
* (chore) Adds a user warning when providing a camera frame that seems like an 'optical' frame instead of the physical link
* (fix) Fixes setting the local map frame based on the available transformation data
* (refactor) Adds utility method to lookup and wait for a TF in component and uses it in the Controller
* (fix) Fixes all return paths for set_algorithm and adds tests for it
* (chore) Adds tests for planner and path controller
* (fix) Removes stale interface calls in controller
* (refactor) Adds path controller status enum to track status uniformly during control loops
* (fix) Fixes handling of detection and point of interest use in planning callback for EVENT mode
* (refactor) Removes blocking wait for odom world tf to not starve the callbacks
* (fix) Removes reached_end topic spamming from reached_point function in planner
* (refactor) Makes robot state wait loop non blocking with timeout in controller
* (fix) Fixes quaternion construction in planner for goal orientation
* (fix) Fixes path deviation error assignment in controller action feedback msg
* (fix) Fixes path tracking callback feedback construction
* (feature) Adds correctness pytests for controller vision tracking server
* (refactor) Reduces vision_tracking_callback complexity by refactoring steps into helper methods
* (feature) Adds block attribute to update_state to block on controller init and prevent blocking inside control loops
* (fix) Adds missing sleep to wait for incoming data
* (fix) Adds wait and timeout for incoming data on vision tracking setup
* (fix) Fixes state update in vision follower server callback
* (fix) Sets detection search default value to None in callback to avoid `next` exceptions
* (chore) Uses 'warning' instead of 'warn' in loggings for Rolling compatibility
* (fix) Fixes Detections2D and PointsOfInterest callbacks handeling of depth data
* (fix) Marks static transforms to reduce TF search overhead
* (fix) Fixes planner action feedback initialization
* (feature) Adds Trackings to Planner supported goal_point input types
* (fix) Adds action description to set_algorithm action
* (feature) Adds support for Detections as Planner goal_point input
* (fix) Updates Twist pubisher to match parent in sugarcoat
* (feature) Adds components introspection and component actions description
* (feature) Implements set_input and set_output method in component to be used by the base Launcher
* (fix) Removes launcher module from kompass and migrates to upstream base launcher in sugarcoat
* (fix) Uses _update_param method in controller algorithm change action
* (feature) Adds PointsOfInterest to supported external data types
* (feature) Allows multiple topics for Planner goal_point input
* (fix) Adds check for reaching the target goal in timed planner execution
* Contributors: ahr, aleph-ra, mkabtoul

0.4.1 (2026-02-16)
------------------
* (fix) Fixes TwistStamped callback
* (fix) Updates turtlebot3 recipe
* (chore) Adds missing config imports from kompass core
* (fix) Adds wait time for fetching inputs on acion execution
* (chore) Adds prompt for llms.txt context ingestion
* (chore) Adds minimum version check for sugarcoat to init
* (refactor) Fixes imports from sugarcoat
* (refactor) Removes events module and updates imports from sugarcoat
* Contributors: ahr, mkabtoul

0.4.0 (2026-01-07)
------------------
* (chore) Adds version check for kompass-core
* (fix) Uses new PointFieldType for emergency stop checking with point cloud data
* (fix) Removes assertion from pointcloud callback and gives a warning
* (fix) Adds pure pursuit config class to control module
* (fix) Adds warning message for unsupported configuration in drive manager
* (feature) Adds global map server to turtlebot3 recipe
* (fix) Adds missing header frame to global path publisher
* (feature) Enables using pure pursuit controller from kompass-core
* (fix) Fixes TwistStamped converter
* (fix) Updates drive manager to enable using point cloud data directly in critical zone check
* Contributors: ahr, aleph-ra, mkabtoul

0.3.3 (2025-11-30)
------------------
* (fix) Updates critical zone checker initialization to match kompass core
* (feature) Adds robot_plugin argument to launcher
* (fix) Fixes command publishing pre-processor
* (fix) Fixes topic update from file
* (feature) Enables using robot plugin in drive manager
* (fix) Fixes image size update in callback
* (fix) Moves external callbacks to new module and inherets from external callback classes
* (fix) Fixes update attribute name in BaseComponentConfig attrs class
* Contributors: ahr, mkabtoul

0.3.2 (2025-10-03)
------------------
* (fix) Removes Scipy dependency from DriveManager
* (docs) Adds MapServer docs
* (feature) Adds map server for serving static global map
* Contributors: ahr, mkabtoul

0.3.1 (2025-09-02)
------------------
* (fix) Adds 'reached_end' publishing in Planner
* (feature) Simplifies pointcloud callback as its processing is handled in kompass core and changes local mapper to take pointcloud data
* (fix) Adds missing inputs/outputs keys serialization in component
* (feature) Adds option to enable emergency stop usage in DriveManager without 360 scan
* (fix) Fixes list inputs parsing in component
* (feature) Adds PointCloud2 to DriveManager and Mapper allowed inputs
* Contributors: ahr, mkabtoul

0.3.0 (2025-07-02)
------------------
* (fix) Fixes image size check in callback
* (feature) Adds config parameter for core algorithm logging level in Component
* (fix) Moves vision controller setup to component start
* (fix) Removes redundant cmd_rate parameter and adds emergecy_checker warmup
* (docs) Updates docs, docstrings and add chinese and japanese README
* (feature) Adds config parsing from json, toml and yaml
* (fix) Fixes vision follower setup
* (feature) Adds CLI and updates docs
* (feature) Adds buffer size to detections callback
* (fix) Moves emergency stop checking from lidar callback to publish method to include command direction
* (feature) Updates Controller to use both RGB and RGBD based vision followers
* (fix) Adds wait for incoming detections in VisionFollower action server
* (fix) Adds logging info in unblocking action
* (feature) Updates controller to use new detections message from embodied-agents (includeing Depth)
* (feature) Adds tracking by label to vision tracking action server
* (fix) Adds timestamp to detections callback
* (feature) Adds slowdown factor to DriveManager
* (feature) Sets vision tracking initial target directly from label in action request
* (fix) Fixes minor errors in controller action server
* (fix) Fixes commands parallel publishing in controller and drive manager
* (fix) Adds inputs check to vision tracking action server
* (fix) Minor fix in controller's twist array publish mode
* (feature) Updates controller component to use VisionDWA from core
* (feature) Adds 'debug' mode to controller component
* (feature) Adds local map debug to controller
* (fix) Fixes rate in loop sleep time
* (fix) Fixes init usage of CriticalZoneCheckerGPU
* (feature) Uses critical zone checker directly from kompass_cpp
* (feature) Moves laserscan tf to be executed inside the emergency checker in Cpp
* (feature) Moves laserscan transformation from callback to kompass-core
* (fix) Fixes parsing local plan in controller
* (fix) Treats both numpy arrays and lists in init_twist_array
* (fix) Uses main_goal_lock in controller action callback and fixes activation based on mode
* (fix) Fixes action cencellation on new coming action
* (fix) Fixes emergency stop direction
* (fix) Handles empty global paths in controller
* (feature) Updates DriveManager to use EmergencyChecker from kompass_core
* (fix) Sets detections buffer size from config
* (chore) Adds error message for kompass-core installation when not found
* (fix) Fixes setting new buffer size in DetectionsCallback
* (feature) Adds map resolution set in controller from map callback metadata
* (fix) Executes emergency stop check only if enabled in config
* (fix) Minor fix for python3.8 compatibility
* (feature) Adds message buffer to Detections/Trackings callbacks
* Contributors: ahr, mkabtoul

0.2.1 (2025-01-28)
------------------
* (fix) Adds controller custom on activate method
* (fix) Updates controller activation method
* (fix) Adds ompl exception catch and log
* (fix) Recreates all subscribers at mode change
* (feature) Fixes set_algorithm for online mode switch
* (feature) Adds TwistStamped to supported types
* (fix) Fixes imports and error in webots_ros2 implementation
* (docs) Updates vision tracking tutorial
* (fix) Adds a list of input/output topics containing None defaults
* (feature) Adds support for 'None' default values of inputs/outputs
* (docs) Adds VisionFollower docs and updates docstrings
* (feature) Adds wait to vision target following callback
* (feature) Adds Detections callback
* (feature) Adds support for lists in topics dictionary and adds doc-strings
* (feature) Adds wait time to get vision tracking in callback loop
* (fix) Sets control callback rate to control time step
* (feature) Updates control classes and supported types
* (feature) Add wait time for vision tracking input in action callback
* (feature) Adds component for visualization
* (feature) Adds Detections to supported types
* (feature) Adds vision object tracking to controller
* (feature) Adds control publishing options to Controller (`#32 <https://github.com/automatika-robotics/kompass-ros/issues/32>`_)
  Adds parallel and array publish options to Controller and updates docs
* (docs) Adds default parameters for each control algorithm
* (fix) Exposes max_num_threads parameter for DWA controller in turtlebot example
* (fix) Updates path last cost in Planner and testing params
* (feature) Adds command execution in closed loop to DriveManager
* (feature) Adds option to select controller commands publishing type
  Select to publish the control commands in a new thread or to publish a TwistArray
* Contributors: ahr, mkabtoul

0.2.0 (2024-10-25)
------------------
* (feature) Minor updates (`#31 <https://github.com/automatika-robotics/kompass-ros/issues/31>`_)
* (feature) Adds minor updates/fixes for latest ros sugar release
* (feature) Updates drive manager to use new laserscan methods
* (feature) Adds unblocking actions to DriveManager (`#30 <https://github.com/automatika-robotics/kompass-ros/issues/30>`_)
* (fix) Adds return False when unblocking action is not possible
* (chore) Adds note to planner
* (fix) Fixes control command type hint in drive manager
* (fix) Fixes publishing action feedback in planner
* (feature) Adds RVIZ launch file and updates testing params
* (fix) Fixes tracked point publishing in controller
* (feature) Exposes robot motion actions in drive manager used for robot unblocking
* (feature) Adds unblocking actions forward/backward/rotate in drive manager based on laserscan data
* (fix) Adds debug logging and fixes loop rate in planner action server callback
* (fix) Fixes reached_end check in controller
* (feature) Adds local mapper component (`#29 <https://github.com/automatika-robotics/kompass-ros/issues/29>`_)
  * (feature) Adds LocalMapper component
  * (fix) Updates commands queue clear in controller
  * (fix) Fixes imports and launcher init to match latest ros_sugar update
  * (fix) Adds missing dependencies and updates rviz config file
  * (docs) Adds more details to events/action tutorial in docs and updates docstrings
  * (docs) Minor updates in docs tutorials
  * (docs) Updates mapper inputs and adds docs
  * (docs) Adds documentation for supported data types
  * (docs) Updates url links to ros_sugar docs
  * (docs) Fixes minor typo
  * (docs) Removes apidocs and updates gitignore
* (refactor) Updates imports from ros_sugar (`#28 <https://github.com/automatika-robotics/kompass-ros/issues/28>`_)
* (feature) Adds option to toggle parallel publishing of commands in controller (`#27 <https://github.com/automatika-robotics/kompass-ros/issues/27>`_)
  * Adds option in Controller to turn on/off publishing commands in parallel
  * Updates Laserscan processing in callback
  * Updates turtulebot3 test recipe
  * Removes unused import and fixes method return type hint
* (feature) Adds local map input to the controller and updates docs (`#26 <https://github.com/automatika-robotics/kompass-ros/issues/26>`_)
  * (feature) Sends controller commands to robot in parallel to control loop
  * (feature) Adds commands queue to handle sending commands to the robot from the controller
  * Updates test parameters
  * Imports callbacks/datatypes from ros_sugar
  * Updates DriveManager and api docs
  * Updates controller docs
* (fix) Adds correct paths for params in simulation launch files
  Changes version number according to ROS convention
* Initial release 0.1.1a
* Contributors: ahr, mkabtoul
