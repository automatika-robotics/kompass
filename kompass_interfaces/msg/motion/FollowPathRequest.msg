# msg request to start following a path
int32 FOLLOW_PATH_FROM_ROS_TOPIC = 0
int32 FOLLOW_PATH_FROM_XML_FILE = 1

int32 path_source
string path_name        # ROS topic name or path file xml name
string path_file_location   # path file location
int32 path_follower_type
string robot_cmd_topic      # topic name for sending the control
