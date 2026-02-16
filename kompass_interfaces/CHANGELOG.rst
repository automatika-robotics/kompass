^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kompass_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.4.0 (2026-01-07)
------------------
* (fix) Adds number of point to path recording/saving service response
* Contributors: ahr, mkabtoul

0.3.3 (2025-11-30)
------------------

0.3.2 (2025-10-03)
------------------
* (feature) Adds map server for serving static global map
* Contributors: ahr, mkabtoul

0.3.1 (2025-09-02)
------------------

0.3.0 (2025-07-02)
------------------
* (fix) Moves vision controller setup to component start
* (feature) Updates controller component to use VisionDWA from core
* Contributors: ahr, mkabtoul

0.2.1 (2025-01-28)
------------------
* (fix) Adds action_msgs dependency for kompass_interfaces
* (feature) Adds support for 'None' default values of inputs/outputs
* (feature) Updates control classes and supported types
* (feature) Adds vision tracking action/srv and updates interfaces
* (feature) Adds control publishing options to Controller (`#32 <https://github.com/automatika-robotics/kompass-ros/issues/32>`_)
  Adds parallel and array publish options to Controller and updates docs
* (feature) Adds time_step to TwistArray msg
* Contributors: ahr, mkabtoul

0.2.0 (2024-10-25)
------------------
* (fix) Adds correct paths for params in simulation launch files
  Changes version number according to ROS convention
* Initial release 0.1.1a
* Contributors: ahr, mkabtoul
