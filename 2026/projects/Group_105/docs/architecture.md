# Architecture

Status: target design for the final proof of concept, as presented at the mid-term. Only the offline gait prototype in [`walk_analysis_with_camera/`](../walk_analysis_with_camera/) exists so far. Decisions are recorded in [`decisions/`](decisions/).

## Overview

The robot is a self-positioning gait station. Before a session it drives to a station beside a walking path and points its camera across the walking path. The patient walks back and forth past the robot, through the camera's capture zone, while the robot plays a beat at the prescribed cadence. A laptop on the same network turns the video into gait metrics, decides on spoken cues, and sends a summary of each lap to a dashboard for the physiotherapist. A *lap* is one walk past the robot through the capture zone; each lap is one measurement.

![System architecture](img/architecture.svg)

## Physical setup

![Corridor setup](img/setup.svg)

- Flat walking path, about 10 m long, with room for the robot about 2.5 m to the side (lab floor or a wide hallway).
- Robot parked about 2.5 m from the walking line, camera on a 3D-printed mast at about knee height, looking across the path.
- Capture zone: the part of the walking line inside the camera's view, about 3–4 m depending on camera and distance. - The robot does not move while anyone is walking in the zone ([ADR 2](decisions/0002-robot-parks-beside-path.md)).

## Session flow

1. **Position**: the robot drives to the station pose on a map built beforehand with SLAM (Nav2 `NavigateToPose`).
2. **Wait**: the robot says it is ready and starts the beat at the target cadence once a person is in view.
3. **Lap**: the patient walks through the zone. The camera detects steps (cadence, stops); the LiDAR tracks position and walking speed.
4. **Between laps**: the robot gives a cue or the lap count, based on the last lap.
5. **Finish**: after the prescribed number of laps, the session summary is published and the robot returns to its start pose.

## Components

### Robot (TurtleBot3 Burger, Raspberry Pi)

| Component | Package | Role |
| --- | --- | --- |
| Robot drivers | `turtlebot3_bringup` (existing) | `/scan`, `/odom`, `/cmd_vel`, TF |
| Camera driver | `usb_cam` (existing) | Publishes compressed frames from the USB webcam |
| Audio player | `audio_cues` (ours) | Plays the beat locally at the target tempo; speaks cue texts with offline text-to-speech ([ADR 4](decisions/0004-audio-cues-on-robot.md)) |

### Laptop (ROS 2 Jazzy, same `ROS_DOMAIN_ID`)

| Component | Package | Role |
| --- | --- | --- |
| Gait monitor | `gait_monitor` (ours) | Pose estimation per frame and step detection, reusing `walk_analysis` ([ADR 1](decisions/0001-camera-for-gait-sensing.md), [ADR 3](decisions/0003-pose-processing-on-laptop.md)) |
| LiDAR person tracker | `lidar_person_tracker` (ours) | Finds the walker's legs in the scan: presence, position along the path, walking speed |
| Session manager | `walk_session` (ours) | State machine for the session flow, cue rules, lap summaries, Nav2 goals |
| Navigation | `turtlebot3_navigation2` (existing) | Map, AMCL localisation, path to the station and back |
| MQTT bridge | `session_reporter` (ours) | Converts lap and session summaries to JSON on MQTT |
| Dashboard stack | Docker Compose | Mosquitto, Telegraf, InfluxDB, Grafana ([ADR 5](decisions/0005-mqtt-and-grafana-dashboard.md)) |

Custom message types live in a separate `gait_interfaces` package.

## ROS 2 interfaces

| Topic / action | Type | From → to | Content |
| --- | --- | --- | --- |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | `usb_cam` → `gait_monitor` | MJPEG frames, camera timestamps |
| `/gait/steps` | `gait_interfaces/StepEvent` | `gait_monitor` → `walk_session` | Time and foot (left/right) of each detected step |
| `/gait/state` | `gait_interfaces/GaitState` | `gait_monitor` → `walk_session` | Person in view, rolling cadence (steps/min), time since last step |
| `/scan` | `sensor_msgs/LaserScan` | `turtlebot3_bringup` → `lidar_person_tracker`, Nav2 | LiDAR ranges |
| `/person/state` | `gait_interfaces/PersonState` | `lidar_person_tracker` → `walk_session` | Person detected, position along the path, walking speed |
| `/session/beat_bpm` | `std_msgs/Float32` | `walk_session` → `audio_cues` | Beat tempo; 0 stops the beat |
| `/session/cue` | `std_msgs/String` | `walk_session` → `audio_cues` | Text to speak |
| `/session/lap_summary` | `gait_interfaces/LapSummary` | `walk_session` → `session_reporter` | Per lap: duration, step count, mean cadence, stops, average speed |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `walk_session` → Nav2 | Station pose and return pose |

## MQTT interface

Broker: Mosquitto on the laptop, port 1883, local network only. Payloads are JSON.

| MQTT topic | When | Payload fields |
| --- | --- | --- |
| `rehab/<patient_id>/lap` | After each lap | `session_id`, `lap`, `duration_s`, `steps`, `cadence_spm`, `target_spm`, `stops`, `speed_mps` |
| `rehab/<patient_id>/session` | End of session | `session_id`, `laps`, `mean_cadence_spm`, `cadence_drift_spm` (last lap minus first), `total_stops` |

`patient_id` is an anonymised ID, never a name.

## Gait metrics

| Metric | Method | Goal |
| --- | --- | --- |
| Steps | A step is the moment a heel is furthest in front of the hips (heel strike, method of Zeni et al., 2008). Per frame: horizontal distance from the hip midpoint to each heel, divided by leg length in the image; each forward maximum is one step, labelled left or right. Walking direction comes from the hip movement | Core |
| Cadence | Steps per minute, over the last few seconds and per lap | Core |
| Stop | Person in view, no step for N seconds and LiDAR speed near zero (N set from recorded data) | Core |
| Walking speed | LiDAR leg clusters near the path; slope of a line fitted to the walker's position over the last second | Core |
| Lap time | Time between entering and leaving the zone (LiDAR position) | Core |
| Cadence drift | Last lap cadence minus first lap cadence (fatigue) | Core |
| Step-time symmetry | Ratio of left and right step intervals | Stretch |

The prototype in [`walk_analysis.cadence`](../walk_analysis_with_camera/src/walk_analysis/cadence.py) detects steps from ankle lift instead (a rise in ankle height in the image). Ankle lift is small and easy to miss in a shuffling gait, and it is sensitive to camera tilt. Both methods will be run on our own recordings and compared with hand-counted steps; the one closer to the hand count goes into `gait_monitor`. Thresholds are set from the same recordings, not chosen in advance.

## Cue rules

| Condition | Spoken cue |
| --- | --- |
| Person in view, not yet walking | "Ready when you are" |
| Stop detected during a lap | "Take your time, start again when ready" |
| Lap cadence below target band | "Try to step with the beat" |
| Lap cadence above target band | "A little slower" |
| Lap within the band | Lap count and encouragement |
| Camera stream lost | "Pausing, please wait" (session paused) |

The target cadence, tolerance band, number of laps and station pose come from a session parameter file set by the physiotherapist.

## Ownership

The work is split by sensor so that both team members can work independently. The `gait_interfaces` messages and the topics above are the contract between the two parts; change them only together.

| Part | Owner | Packages |
| --- | --- | --- |
| LiDAR, robot and session logic | Leonid Pomortsev | `lidar_person_tracker`, `walk_session`, Nav2 setup and map, `session_reporter`, dashboard stack |
| Camera and audio | Matin Mahmoudzadeh | camera mast, `usb_cam` setup, `gait_monitor`, `audio_cues` |

## Safety and privacy

- The robot moves only before and after the session, with the default TurtleBot3 speed limits and Nav2 obstacle avoidance. It is stationary while a person walks.
- The robot gives no physical support and is not a clinical device.
- Video frames are processed in memory and never stored. Only metrics are sent over MQTT, tagged with an anonymised ID.
- If the camera stream or pose tracking drops, the session pauses rather than reporting wrong metrics.

## Stretch goals

- Rotate in place to keep the walker in frame and lengthen the capture zone.
- Step-time symmetry per lap.
- Session start/stop button on an ESP32, over MQTT.
