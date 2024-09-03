# This hold basic information about the characteristics of a grid
# grid has shape width x height   with origin_pose is at (0,0), the upper left grid cell to the robot_pose
# The time at which the map was loaded
builtin_interfaces/Time map_load_time

# The map resolution [m/cell]
float32 resolution

# Map width [cells]
uint32 width

# Map height [cells]
uint32 height


Pose3D origin_pose
Pose3D robot_pose
