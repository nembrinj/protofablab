# 3. What runs on the robot and what runs on the laptop

- Status: accepted
- Date: 2026-11-04

## Context

Pose estimation on every camera frame is too heavy for the robot's Raspberry Pi, which also runs the LiDAR, motor control and ROS 2 drivers. The course setup already uses a laptop running ROS 2 Jazzy on the same network as the robot. The patient must also hear a steady beat: a beat sent over Wi-Fi as one message per beat would arrive unevenly.

## Decision

- **Robot (Raspberry Pi):** drivers only (TurtleBot3, camera), plus audio. A USB speaker plays the beat, generated locally from a target tempo, and short spoken cues with offline text-to-speech.
- **Laptop:** pose estimation, LiDAR person tracking, session logic, Nav2, and the dashboard containers. It sends the robot only the beat tempo and the cue texts.

The camera stream goes to the laptop as compressed frames (MJPEG) over ROS 2.

## Consequences

- The laptop can run the full MediaPipe model and reuse the `walk_analysis` code.
- Beat timing does not depend on the network, and the sound comes from the robot, where the patient is.
- Video crosses Wi-Fi, which adds delay and can drop frames. Resolution, frame rate and delay must be measured, and timestamps must come from the camera.
- If the camera stream stops, the session pauses and the robot says so, instead of reporting wrong numbers.
- The speaker draws power from the robot; check it does not cause brownouts on the LiDAR and camera. Fallback for the demo: play audio on the laptop.
- In a home product the laptop would be a small home hub.

## Alternatives considered

- **Pose model on the Raspberry Pi**: no network dependency, but too slow at a useful frame rate.
- **Cloud processing**: sends patient video off-site; rejected for privacy.
- **Laptop speaker only**: simplest, but the laptop is not where the patient walks.
