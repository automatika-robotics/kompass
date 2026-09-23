# Status of the MissionManager

string mission_id               # Mission the status is about, from the time it was received; empty before the first one

uint8 STATE_NAVIGATING=0
uint8 STATE_PAUSED_DWELL=1
uint8 STATE_PAUSED_CONDITION=2
uint8 STATE_RETURNING_TO_START=3
uint8 STATE_IDLE=4                  # No mission has run yet
uint8 STATE_COMPLETED=5             # Final: the mission reached every waypoint
uint8 STATE_CANCELED=6              # Final: the mission was canceled by its client
uint8 STATE_ABORTED=7               # Final: the mission ended early, on a failure or a timed out pause, or was refused
uint8 STATE_PAUSED=8                # Paused on request: the robot holds position until the mission is resumed
uint8 state

uint32 total_goals              # Number of waypoints in the mission
uint32 current_goal_idx         # Waypoint being worked on

uint8 LEVEL_INFO=0
uint8 LEVEL_ERROR=1
uint8 message_level
string message                  # Latest message from the MissionManager
