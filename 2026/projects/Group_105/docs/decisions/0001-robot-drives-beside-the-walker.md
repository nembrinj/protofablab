# 1. The robot drives beside the walker and keeps the legs in the picture

- Status: accepted
- Date: 2026-11-19

Replaces the mid-term plan, where the robot stayed parked while the patient walked.

## Context

The physiotherapist wants to see how the patient walked, not only numbers: a short video of each moment that went wrong, with the numbers that show it. So the camera has to film the legs during the whole exercise, from the same good angle.

A parked camera sees only 3–4 m of a 10 m walking path. The rest of each walk is not filmed, and the angle changes as the person comes closer and moves away again.

The professors advised filming from the side and slightly from the front: the camera then sees both legs and how far each foot moves forward.

## Decision

The robot drives beside the walking path, parallel to it, at a set distance from the person and a little ahead of them. The camera points straight to the side, so it sees the legs slightly from the front.

On one walk the robot drives forward, on the walk back it drives backward. The camera stays on the person's side, and the robot never has to turn around or pass in front of the person. While the person turns around at the end of the path, the robot moves a short way, so that it is again a little ahead of them.

The robot follows the person with the LiDAR: it drives at the person's walking speed, and corrects when the gap to the person grows or shrinks.

We build it in two stages:

1. The robot stays parked beside the path. Measuring, feedback, clips and the dashboard are built and tested on a robot that does not move.
2. The robot drives beside the person.

## Consequences

- Every walk is filmed from the same angle, from start to end.
- The robot must drive as fast as the person walks. Our test walker walked at 0.6–0.95 m/s. The TurtleBot3 spec sheet gives 0.22 m/s as the top speed, but in a first lab check the robot drove faster. The real top speed is measured before stage 2 is built. When the person walks faster than the robot, the robot falls behind, and that part of the walk is marked as not filmed.
- The LiDAR tracker compares each scan with the empty room ([ADR 4](0004-find-the-walker-against-the-empty-room.md)). That only works while the robot stands still. While it drives, the tracker needs another reference, for example a SLAM map of the room. The map is also used to drive to the start of the path.
- The robot moves near a person with limited mobility. It stays beside the path, never enters the walking line, keeps its distance to the person, and stops when it loses them.
- The camera mount points straight to the side.

## Alternatives considered

- **Parked beside the path** (the mid-term plan): the simplest, and kept as stage 1. Films only 3–4 m of each walk, from a changing angle.
- **Turning in place to follow the person**: films more of the path, but the ends of the path are far away and seen at a steep angle. Kept as a fallback if driving beside the person is not ready in time.
- **Driving in front of or behind the person**: the robot would be in the walking line, a trip hazard, and the camera cannot see how far each foot moves forward.
- **Camera pointing back at a slant**: a better angle, but in one direction only. On the walk back the camera faces away from the person, unless the robot turns around and passes them.
