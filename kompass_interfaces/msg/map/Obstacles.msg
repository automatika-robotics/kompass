# This represents a multi 2-D grid map
std_msgs/Header header

# MetaData for the map
nav_msgs/MapMetaData info


#occupancy info
int32[] obstacle_type #1 occupied by scan  #2 occupied by detections #3 both
float32[] x_global
float32[] y_global
float32[] x_local
float32[] y_local
int32[] i_grid
int32[] j_grid
float32[] occupied_zone

#semantic info
int32[] class_id
# string[] class_name

#tracking info
int32[] object_id
float32[] vx
float32[] vy
