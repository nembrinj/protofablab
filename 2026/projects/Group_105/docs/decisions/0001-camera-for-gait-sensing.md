# 1. Camera and pose estimation for gait sensing

- Status: accepted
- Date: 2026-11-04

## Context

The initial idea measured gait with the TurtleBot3 LiDAR: one horizontal plane at shin height, 5 scans per second, 3.5 m range. At 5 Hz a step at 80 steps per minute is covered by about four scans, which makes step detection fragile. Telling the left leg from the right, which symmetry needs, is near the resolution limit. Dark trousers reflect poorly and show up as dropouts.

## Decision

Measure gait with a USB webcam mounted on the robot and a pose-estimation model (MediaPipe Pose Landmarker), which gives labelled left and right hip, ankle and heel positions at camera frame rate. The offline prototype in `walk_analysis_with_camera/` already counts steps and estimates cadence from these positions.

The LiDAR stays in use for what it does well: localisation on a map, obstacle avoidance while the robot drives to its station, and tracking where the walker is and how fast they walk. Distance in metres needs no camera calibration, and walking speed only needs the position of both legs together, not single steps.

## Consequences

- Left and right feet are labelled by the model, so step-time symmetry becomes reachable.
- The robot needs a camera mount at a height where the walker's legs are in frame (3D-printed mast).
- Video of a person at home is sensitive. Frames are processed live and never stored; only gait metrics leave the processing machine.
- Pose models need more compute than the robot's Raspberry Pi offers (see [ADR 3](0003-pose-processing-on-laptop.md)).
- The capture zone is set by the camera field of view and distance, not by LiDAR range.
- Camera and LiDAR give two independent views of the walker: steps from the camera, position and speed from the LiDAR.

## Alternatives considered

- **LiDAR leg tracking**: no privacy issue and already on the robot, but too coarse for cadence and symmetry at 5 Hz.
- **Wearable IMU on the ankle**: accurate, but the patient must put it on correctly, and it moves the project away from the robot.
