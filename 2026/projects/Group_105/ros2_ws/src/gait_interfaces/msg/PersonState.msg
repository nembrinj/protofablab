# Where the walker is on the walking path, measured by the LiDAR; while the robot
# drives, with the odometry as well. Published once per scan, after the empty
# room has been learned.

std_msgs/Header header  # time of the scan
bool detected           # at least one person is on the walking path
uint8 people_count      # position_m and speed_mps are valid only when this is 1
float32 position_m      # along the path, from the robot: 0 is straight in front of it, positive to its left; NaN when not valid
float32 speed_mps       # along the path, over the floor, positive towards the robot's left; NaN when not valid or not yet known
uint32 steps            # steps counted since the empty room was learned; use the difference between two messages
