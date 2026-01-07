^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kompass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
