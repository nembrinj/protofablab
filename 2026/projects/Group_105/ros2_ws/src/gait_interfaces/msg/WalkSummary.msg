# The numbers of one walk: one way along the path, as far as the LiDAR saw it.
# Published by walk_session when the walk ends.

std_msgs/Header header  # time the end of the walk was noticed (scan time)
string session_id       # the session the walk belongs to
uint32 walk             # number of the walk in the session, from 1
bool measured           # false when two people were seen, or the walker was lost in the middle of the zone
float32 duration_s      # from the start of the walk to its end, stops included
uint32 steps            # steps counted during the walk
float32 cadence_spm     # steps per minute while walking, stops left out; NaN when unknown
float32 speed_mps       # distance along the path per second while walking, stops left out; NaN when unknown
uint32 stops            # how many times the walker stopped during the walk
