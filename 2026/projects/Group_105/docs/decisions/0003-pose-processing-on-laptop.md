# 3. Pose processing runs on a laptop, not on the robot

- Status: accepted
- Date: 2026-11-04

## Context

Pose estimation on every camera frame is too heavy for the robot's Raspberry Pi, which also runs the LiDAR, motor control and ROS 2 drivers. The course setup already uses a laptop running ROS 2 Jazzy on the same network as the robot, with the same `ROS_DOMAIN_ID`.

## Decision

The robot publishes compressed camera frames (MJPEG) over ROS 2. A laptop on the same network runs pose estimation, gait metrics, the session logic, Nav2, and the dashboard containers. The Raspberry Pi runs only drivers and audio playback.

## Consequences

- The laptop can use the full MediaPipe model and reuse the existing `walk_analysis` code.
- Video crosses Wi-Fi, which adds latency and can drop frames. Resolution, frame rate and latency must be measured, and timestamps must come from the camera, not from arrival time.
- The system depends on the network. If the camera stream stops, the session pauses and the robot says so, instead of reporting wrong metrics.
- In a home product the laptop would be a small home hub; for the proof of concept a laptop is enough.

## Alternatives considered

- **Pose model on the Raspberry Pi**: no network dependency, but too slow at a useful frame rate.
- **Cloud processing**: sends patient video off-site; rejected for privacy.
