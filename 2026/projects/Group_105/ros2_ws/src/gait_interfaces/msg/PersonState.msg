# Where the walker is on the walking path, measured by the LiDAR of the parked robot.
# Published once per scan, after the empty room has been learned.

std_msgs/Header header  # time of the scan
bool detected           # at least one person is on the walking path
uint8 people_count      # position_m and speed_mps are valid only when this is 1
float32 position_m      # along the path: 0 is straight in front of the robot, positive to its left; NaN when not valid
float32 speed_mps       # along the path, positive towards the robot's left; NaN when not valid or not yet known
