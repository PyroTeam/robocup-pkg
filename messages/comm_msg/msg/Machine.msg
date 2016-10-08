uint8 CYAN = 0
uint8 MAGENTA = 1

uint8 RING_BLUE   = 1
uint8 RING_GREEN  = 2
uint8 RING_ORANGE = 3
uint8 RING_YELLOW = 4


string name
string type
string state
uint8 team_color


LightSpec[] lights
geometry_msgs/Pose2D pose
uint8 zone
bool correctly_reported
uint32 loaded_with
uint8[] ring_colors
