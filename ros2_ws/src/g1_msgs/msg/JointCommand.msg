std_msgs/Header header

# Joint order matches G1_NUM_MOTOR / G1JointIndex in the Unitree SDK G1 example.
# joint_mask[i] == false means the bridge ignores q/dq/tau/kp/kd at index i.

uint8 mode_pr
uint8 mode_machine

float32[29] q
float32[29] dq
float32[29] tau
float32[29] kp
float32[29] kd
bool[29] joint_mask
