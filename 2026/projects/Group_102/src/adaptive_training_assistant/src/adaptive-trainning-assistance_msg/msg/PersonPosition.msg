std_msgs/Header header

bool person_detected
bool upper_body_detected

float32 tracking_x
float32 tracking_y

# Horizontal distance between the two shoulders as a fraction of the image
# width (0..1). Used to estimate how far away the person is. 0 when both
# shoulders are not visible.
float32 shoulder_width
