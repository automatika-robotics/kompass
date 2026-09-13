# Status of the MissionManager

string mission_id               # Ongoing mission, from the time it was received; empty when there is none

uint8 STATE_NAVIGATING=0
uint8 STATE_PAUSED_DWELL=1
uint8 STATE_PAUSED_CONDITION=2
uint8 STATE_RETURNING_TO_START=3
uint8 STATE_IDLE=4                  # No ongoing mission
uint8 state

uint32 total_goals              # Number of waypoints in the ongoing mission
uint32 current_goal_idx         # Waypoint being worked on

uint8 LEVEL_INFO=0
uint8 LEVEL_ERROR=1
uint8 message_level
string message                  # Latest message from the MissionManager
