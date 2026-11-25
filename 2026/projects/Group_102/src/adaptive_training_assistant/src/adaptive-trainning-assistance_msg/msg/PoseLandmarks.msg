std_msgs/Header header

# True when MediaPipe found a pose in this frame.
bool pose_detected

# All 33 MediaPipe Pose landmarks, indexed as in MediaPipe (e.g. 11 = left
# shoulder, 15 = left wrist). Coordinates are normalised to the image (0..1,
# y grows downward). The arrays are empty when pose_detected is false.
float32[] x
float32[] y
float32[] visibility
