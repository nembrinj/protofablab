#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import time
import math
import cv2
import mediapipe as mp
import numpy as np
from pydub import AudioSegment
from pydub.playback import play

# MQTT broker details
broker_address = "192.168.120.86"

# Two fixed positions and yaws for initial movement (absolute positions in map frame)
positionA = (1.0, 1.0, 0.0)         # (x, y, yaw)
positionB = (2.0, 2.0, math.pi/2)   # Another absolute position in map frame

# Thresholds
goal_tolerance = 0.3      # meters
yaw_tolerance = 0.4       # radians
human_distance_threshold = 0.5  # meters

# Camera and human detection constants
FOV = 120  # degrees
RW = 1280  # resolution width
KNOWN_WIDTH = 35  # cm (shoulder width)

def calculate_focal_length_in_pixels(resolution_width, FOV_degrees):
    FOV_radians = FOV_degrees * (math.pi / 180)
    focal_length_pixels = resolution_width / (2 * math.tan(FOV_radians / 2))
    return focal_length_pixels

FOCAL_LENGTH = calculate_focal_length_in_pixels(RW, FOV)

mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.7, min_tracking_confidence=0.7)
cap = cv2.VideoCapture(0)

# Global variables to store current AMCL pose of robot
current_x = None
current_y = None
current_yaw = None

surveillance_mode = True

def estimate_position_and_distance(image_width, image_height, landmark1, landmark2, focal_length, known_width):
    pixel_distance = math.hypot((landmark1.x - landmark2.x) * image_width, (landmark1.y - landmark2.y) * image_height)
    distance = (known_width * focal_length) / pixel_distance
    avg_x = (landmark1.x + landmark2.x) / 2 * image_width
    center_x = image_width / 2
    pixel_offset = avg_x - center_x
    distance_per_pixel = math.tan(math.radians(FOV / 2)) * 2 * distance / image_width
    cm_offset = pixel_offset * distance_per_pixel
    return cm_offset, distance

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to MQTT broker with result code", rc)
    # Subscribe to robot pose updates
    client.subscribe("pmb2/amcl_pose")

def on_message(client, userdata, msg):
    global current_x, current_y, current_yaw
    if msg.topic == "pmb2/amcl_pose":
        data = json.loads(msg.payload.decode('utf-8'))
        current_x = data["x"]
        current_y = data["y"]
        current_yaw = data["yaw"]

def send_goal(client, gx, gy, gyaw):
    goal_msg = {"x": gx, "y": gy, "yaw": gyaw}
    client.publish("pmb2/goal", json.dumps(goal_msg))
    print(f"Published goal: x={gx}, y={gy}, yaw={gyaw}")

def goal_reached(gx, gy, gyaw):
    # Check if current_x, current_y, current_yaw are known
    if current_x is None or current_y is None or current_yaw is None:
        return False
    dist = math.sqrt((gx - current_x)**2 + (gy - current_y)**2)
    yaw_diff = abs((gyaw - current_yaw + math.pi) % (2*math.pi) - math.pi)
    return (dist < goal_tolerance) and (yaw_diff < yaw_tolerance)

def wait_until_goal_reached(gx, gy, gyaw, timeout=60):
    start_time = time.time()
    while time.time() - start_time < timeout:
        if goal_reached(gx, gy, gyaw):
            print("Goal reached!")
            return True
        time.sleep(0.5)
    print("Timeout waiting for goal!")
    return False

def check_for_human():
    """Check a few frames to see if a human is detected."""
    for _ in range(10):
        success, image = cap.read()
        if not success:
            continue
        ih, iw, ic = image.shape
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_image)
        if results.pose_landmarks:
            return True
    return False

def move_between_fixed_positions(client, positionA, positionB):
    positions = [positionA, positionB]
    index = 0
    while surveillance_mode:
        gx, gy, gyaw = positions[index]
        send_goal(client, gx, gy, gyaw)
        wait_until_goal_reached(gx, gy, gyaw, timeout=60)
        index = (index + 1) % len(positions)

        if check_for_human():
            return False
    return True


def rotate_towards_human(offset_cm, distance_cm):
    # Compute angle to human relative to robot's facing direction
    yaw_angle = math.atan(offset_cm / distance_cm)
    return yaw_angle

def approach_human(client):
    while True:
        success, image = cap.read()
        if not success or current_x is None or current_y is None or current_yaw is None:
            continue

        ih, iw, ic = image.shape
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_image)
        if not results.pose_landmarks:
            # If human not visible, wait and check again
            time.sleep(0.5)
            continue

        landmark1 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
        landmark2 = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        cm_offset, distance = estimate_position_and_distance(iw, ih, landmark1, landmark2, FOCAL_LENGTH, KNOWN_WIDTH)

        distance_m = distance / 100.0
        print(f"Detected human: offset={cm_offset:.2f}cm, distance={distance:.2f}cm")

        if distance_m < human_distance_threshold:
            # Close enough
            print("Reached human, playing alert sound...")
            song=AudioSegment.from_file('/home/pi/alert_sound.mp3')
            play(song)
            break

        # Calculate yaw towards human
        yaw_angle = rotate_towards_human(cm_offset, distance)
        # Absolute yaw the robot should face = current_yaw + yaw_angle
        goal_yaw = current_yaw + yaw_angle

        # Move closer by a step
        # Let's move half the distance or a capped amount so we don't overshoot
        step_distance = min(distance_m - 0.5, 0.5)  # Move closer by up to 0.5m
        if step_distance < 0.1:
            step_distance = 0.1  # move at least 10 cm

        # Compute absolute goal
        # Robot at (current_x, current_y, current_yaw)
        # Move in the direction of goal_yaw
        gx = current_x + step_distance * math.cos(goal_yaw)
        gy = current_y + step_distance * math.sin(goal_yaw)

        send_goal(client, gx, gy, goal_yaw)
        wait_until_goal_reached(gx, gy, goal_yaw, timeout=30)

# Wait for the robot's AMCL pose to become available
def wait_for_initial_pose(timeout=30):
    start_time = time.time()
    while (current_x is None or current_y is None or current_yaw is None) and time.time() - start_time < timeout:
        print("Waiting for AMCL pose...")
        time.sleep(1)
    if current_x is None or current_y is None or current_yaw is None:
        print("Could not get AMCL pose within timeout!")
        return False
    return True

def main():
    client = mqtt.Client(client_id="RaspberryPiClient")
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(broker_address, 1883)
    client.loop_start()

    # Wait until we have the current pose from AMCL
    if not wait_for_initial_pose():
        print("Exiting because we never got a pose.")
        client.loop_stop()
        return

    # Now that we have current_x, current_y, current_yaw:
    positionA = (current_x, current_y, current_yaw)
    positionB = (current_x, current_y + 2.0, math.pi/2)  # 2 meters ahead in the Y direction

    print("PositionA set to current robot pose:", positionA)
    print("PositionB set to 2m ahead in Y direction:", positionB)

    # Uses positionA and positionB for the surveillance_mode
    global surveillance_mode
    surveillance_mode = True
    ended_normally = move_between_fixed_positions(client, positionA, positionB)

    if not ended_normally:
        # Human detected logic...
        print("Human detected! Switching to human approach mode.")
        approach_human(client)

    client.loop_stop()
    cap.release()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == '__main__':
    main()