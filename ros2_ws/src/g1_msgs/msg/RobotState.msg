std_msgs/Header header

# Joint order matches G1_NUM_MOTOR / G1JointIndex in the Unitree SDK G1 example.
# This is a 29-joint ROS bridge API, not a 35-slot raw packet mirror.

bool connected
uint32[2] version
uint8 mode_pr
uint8 mode_machine
uint32 tick

float32[29] joint_position
float32[29] joint_velocity
float32[29] joint_acceleration
float32[29] joint_torque_estimate
uint8[29] joint_mode

float32[4] imu_quaternion_wxyz
float32[3] imu_gyroscope
float32[3] imu_accelerometer
float32[3] imu_rpy
int16 imu_temperature

uint8[40] wireless_remote
uint32 crc
