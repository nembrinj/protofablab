# 2. The robot parks beside the walking path

- Status: accepted
- Date: 2026-11-04

## Context

The TurtleBot3 Burger has a top speed of 0.22 m/s. Comfortable walking is 1.2–1.4 m/s, and even slow clinical gait is 0.4–0.8 m/s, so the robot cannot lead or follow a walker. A knee-high robot in the walking line is also a trip hazard for someone with impaired mobility.

## Decision

The robot drives to a station beside the walking path before the session, using Nav2 on a map built with SLAM. It stays stationary while the patient walks back and forth past the robot, through the camera's capture zone, like the capture volume of a gait lab. The walking rhythm is set by an audible beat (rhythmic auditory cueing), not by the robot's position. After the session the robot drives back to its start pose.

## Consequences

- Measurements are taken from a still camera, so no motion compensation with drifting odometry is needed.
- A session is a set number of laps through the zone, not continuous measurement over the whole walk. Each lap is one sample, and comparing first and last laps shows fatigue.
- The robot never moves while a person is walking in the zone. It moves only before and after the session, with the default TurtleBot3 speed limits.
- The robot's value is that it places and orients the camera itself, so the patient does not have to set up equipment.

## Alternatives considered

- **Robot leads or follows the walker**: not possible at 0.22 m/s, and unsafe.
- **Robot in front of or behind the walker, or at the end of the path**: the LiDAR would see the legs better, since they no longer hide each other. But the camera would look along the walking direction, where it cannot see how far each foot moves forward, and the person would walk toward the robot. The side is the only position that suits the camera, which is the main step detector.
- **Robot rotates in place to keep the walker in frame**: extends the capture zone using the robot's own orientation. Kept as a stretch goal, because the image moves while the robot turns.
