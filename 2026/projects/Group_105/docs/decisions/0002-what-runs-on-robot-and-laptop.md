# 2. What runs on the robot and what runs on the laptop

- Status: accepted
- Date: 2026-11-04, updated 2026-11-19

## Context

Pose estimation on every camera frame is too heavy for the robot's Raspberry Pi, which also runs the LiDAR, the motors and the ROS 2 drivers. The course setup already uses a laptop with ROS 2 Jazzy on the same network as the robot.

## Decision

- **Robot (Raspberry Pi):** drivers only (TurtleBot3, camera), plus a USB speaker that speaks the feedback with offline text-to-speech.
- **Laptop:** everything else: pose estimation, LiDAR tracking, the driving control, the session logic, the recording and the clips, Nav2, and the dashboard containers. It sends the robot only speed commands and the texts to speak.

The camera stream goes to the laptop as compressed frames (MJPEG) over ROS 2.

## Consequences

- The laptop can run the full pose model and reuse the `walk_analysis` code.
- Video crosses Wi-Fi, which adds delay and can drop frames. Resolution, frame rate and delay must be measured, and timestamps must come from the camera.
- Clips are cut by time, so the robot's clock must match the laptop's. In the first lab test it was two months behind.
- The driving control runs on the laptop, so the robot must stop by itself when speed commands stop arriving, for example when Wi-Fi drops. This is checked on the real robot.
- If the camera stream stops, the session pauses and the robot says so.
- The speaker draws power from the robot; check that it does not cause brownouts on the LiDAR and camera. Fallback for the demo: play the audio on the laptop.
- If a beat to walk to is added later, it is generated on the robot: one message per beat over Wi-Fi would arrive unevenly.
- In a home product the laptop would be a small home hub.

## Alternatives considered

- **Pose model on the Raspberry Pi**: no network delay, but too slow at a useful frame rate.
- **Cloud processing**: sends video of the patient off-site.
- **Laptop speaker only**: the simplest, but the laptop is not where the patient walks.
