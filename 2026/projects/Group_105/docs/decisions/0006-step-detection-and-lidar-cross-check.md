# 6. Steps from heel distance, cross-checked by the LiDAR

- Status: accepted
- Date: 2026-11-04

## Context

Cadence is the core measurement, so step detection must be reliable and must say when it is not. The camera sits low on the robot, beside the path ([ADR 2](0002-robot-parks-beside-path.md)), so the upper body and head may be out of frame. The prototype detects a step when the ankle rises in the image; that rise is small, easy to miss in a shuffling gait, and affected by camera tilt.

## Decision

**Camera, step timing.** A step is the moment the two heels are furthest apart along the path, which is when the front heel touches the floor. `gait_monitor` takes the horizontal distance between the heels in each frame, divided by shin length (knee to ankle) so that the distance to the camera does not matter. Each maximum is one step; the foot in front is the one that stepped. Walking direction comes from the movement of the feet in the image. Only the legs need to be in frame.

**LiDAR, metres.** `lidar_person_tracker` gives the walker's position and speed, and a rough step count per lap from the distance between the two leg clusters, which also swings once per step.

**Session logic, comparison.** `walk_session` combines the two:

- Step length = walking speed ÷ steps per second.
- A lap is marked uncertain when the camera and LiDAR step counts differ by more than a set margin. An uncertain lap is reported as such and never triggers a correction cue.
- A stop needs both sensors to agree: no steps from the camera and no movement from the LiDAR.

The two detectors do not use each other's data, so each can be built and tested alone.

## Consequences

- Works with full-body and legs-only framing.
- Adds step length, a clinically useful metric, without extra hardware.
- Wrong numbers are flagged instead of shown as facts.
- The current ankle-lift method stays as the baseline: both are run on our recordings against hand-counted steps before the heel-distance method replaces it.
- The comparison margin and the stop time are set from those recordings.

## Alternatives considered

- **Heel in front of the hips** (common gait-lab method): needs the hips in frame, which a low camera may not see.
- **Ankle lift** (current prototype): weak signal for shuffling steps.
- **LiDAR as the main step detector**: 5 scans per second gives about three samples per step, too few to time single steps.
- **LiDAR guiding the camera** (where to look, scale in metres): would tie the two detectors together; left out to keep them independent.
