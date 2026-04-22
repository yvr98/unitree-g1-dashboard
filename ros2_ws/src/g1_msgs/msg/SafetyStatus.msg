std_msgs/Header header

bool is_safe
bool is_upright
bool estop_active
bool data_stale
bool joints_within_limits

float32 pitch
float32 roll
string current_alert

string[] violation_sources
int32[] violating_joint_indices
