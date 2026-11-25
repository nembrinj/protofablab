'''
lidar_circle.py

LiDAR logic for circling a person. Used by web_control.py (this file is not a
ROS node itself, it just does the fun maths). The only reason it is a seperate file
is for ease of version control. I may end up creating a lidar_circle class but for
now I will just leave raw methods. 

The camera faces the robots RIGHT side. The LiDAR measures angles like this:
    0 = straight ahead, +90 = left -90 = right (where the camera looks)

Idea in 3 steps:
  1. Find the person: the closest object near -90 degrees (in front of the camera).
  2. Lock the radius: the person distance when the circle starts.
  3. Steer: drive slowly and turn so the person stays at that distance
     and stays straight in front of the camera (at -90 degrees).


https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html
'''

import math   # angle and distance maths


#   SETTINGS
CAMERA_ANGLE = -90     # where the camera looks from LiDAR perspective (-90 = right side)
CIRCLE_SPEED = 0.08    # speed along the circle in m/s (could speed up if need be but slow is likely better)
SEARCH_ANGLE = 30      # look for the person within +-30 degrees of the camera direction
GATE = 0.3             # once locked only accept points within +-0.3 m of the radius
SAFE_DIST = 0.25       # stop if anything is this close in the driving direction (m)
K_DIST = 1.0           # how strongly to fix the distance to the person
K_ANG = 1.0            # how strongly to keep the person in front of the camera


def scan_points(scan):
    '''
    Turns a LiDAR scan into a list of (distance, angle) points.
    Angles are in degrees from -180 to +180. Invalid readings are skipped.

     NOTE: I may end up turning this into NumPy arrays (one array of distances, one of
            angles) as the overhead of building a Python list of tuples here is suboptimal

    Args:
        scan (LaserScan): one full 360 degree scan from /scan

    Returns:
        list of (distance in m, angle in degrees)
    '''
    points = []
    for i in range(len(scan.ranges)):                          # i = index of each reading
        dist = scan.ranges[i]
        if scan.range_min < dist < scan.range_max:             # skip invalid readings
            angle = math.degrees(scan.angle_min + i * scan.angle_increment)   # angle of reading i

            # The angle conversion below is just for my ease as -180/180 is easier to work with than 0-359
            angle = (angle + 180) % 360 - 180                  # wrap to -180 +180

            points.append((dist, angle))
    return points



def find_person(points, radius):
    '''
    Step 1: finds the person = the closest point in front of the camera.

    Args:
        points (list): (distance, angle) points from scan_points()
        radius (float or None): locked circle radius, or None before locking

    Returns:
        (distance, angle) of the person, or None if nobody is found
    '''
    person = None
    for dist, angle in points:

        #check if the angle is within camera view
        if abs(angle - CAMERA_ANGLE) > SEARCH_ANGLE: 
            continue   # not in front of the camera
        if radius is not None and abs(dist - radius) > GATE: # NOTE: this may be thrown off by person/robot speed needs testing
            continue   # too far from the locked radius 
        if person is None or dist < person[0]:
            person = (dist, angle)
    return person


def path_blocked(points, direction):
    '''
    Safety: checks if something is too close in the driving direction.
        This may be finnicky and need tweaking, literally the easiest function here

    Args:
        points (list): (distance, angle) points from scan_points()
        direction (int): +1 driving forward, -1 driving backward

    Returns:
        True if anything is closer than SAFE_DIST within +-30 degrees
        of straight ahead (forward) or straight behind (backward)
    '''
    for dist, angle in points:
        if direction > 0:
            in_path = abs(angle) <= 30            # in front
        else:
            in_path = abs(angle) >= 150           # behind
        if in_path and dist < SAFE_DIST:
            return True
    return False


def circle_speeds(person, radius, direction):
    '''
    Step 3: works out the speeds to circle the person.

    The robot drives slowly forward (or backward) and turns:

        angular velocity (w) = linear velocity(v) / radius(r)
      - base turn  w = v / r         -> drives a circle of radius r
      - distance:  too far | too close -> turn toward | away from the person
      - angle:     person not straight in front of the camera -> turn to fix it

    Args:
        person (tuple): (distance, angle) of the person from find_person()
        radius (float): locked circle radius in m
        direction (int): +1 to circle driving forward, -1 driving backward

    Returns:
        (linear speed in m/s, turning speed in rad/s)
    '''
    dist, angle = person
    v = direction * CIRCLE_SPEED

    dist_error = dist - radius                            # + = too far away
    angle_error = math.radians(angle - CAMERA_ANGLE)      # + = person drifted toward the front

    # Person is on the RIGHT, so the base turn is to the right (negative).
    # The distance fix flips when driving backward, because reversing steers the other way.
    w = -v / radius
    w -= direction * K_DIST * dist_error
    w += K_ANG * angle_error
    return v, w