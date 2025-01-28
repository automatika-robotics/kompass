^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kompass
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
