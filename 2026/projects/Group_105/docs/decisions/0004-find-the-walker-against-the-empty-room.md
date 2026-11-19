# 4. The LiDAR finds the walker by comparing with the empty room

- Status: accepted
- Date: 2026-11-12, updated 2026-11-19

## Context

`lidar_person_tracker` must report where the walker is along the path and how fast they move. The session logic uses this to flag moments, and in stage 2 the robot drives by it ([ADR 1](0001-robot-drives-beside-the-walker.md)). The robot's LDS-02 LiDAR gives 240 distances per turn, 1.5° apart, about 9 times per second. At 2.5 m from the robot, two neighbouring beams land about 6.5 cm apart, so a shin is only 1 or 2 points. That is too few to recognise a leg by its round shape.

In stage 1 the robot is parked while the person walks, so the room around it gives the same scan every time.

## Decision

After parking, the tracker remembers the scan of the empty room for about 2 seconds. From then on, a point belongs to a person when it is:

- clearly nearer than the empty room in that direction (by a margin, 0.2 m to start with), and
- on the walking path: within a strip of floor in front of the robot (0.5 m to 3.5 m away to start with) and within the zone the camera sees.

Points that are close together along the path are one person; a gap wider than a long step means two people. The position is the middle between the outermost points. The speed is the slope of a line fitted to the positions of the last second.

Each scan is first put on a fixed grid of one distance per degree, so the same code works for LiDAR models with a different number of beams.

## Consequences

- No model of a leg is needed, and furniture that stays in place is ignored.
- It works only while the robot stands still. When the robot drives (stage 2), the room moves in the scan and walls would look like people. The tracker then needs another reference, such as the SLAM map.
- The walking path must be clear while the empty room is learned. `walk_session` asks the tracker to learn again each time the robot has parked.
- Anything new on the path counts as a person: a pet, a moved chair, a second person. Two people standing closer together than the gap count as one.
- If the robot is moved or bumped, the remembered scan no longer fits, and the room must be learned again.
- The strip is wide on purpose, so the distance to the walking line does not have to be set for each room. In the first lab test a narrow strip around a fixed 2.5 m line missed a person who walked at 1.2 m. The price: anything new in the strip counts as a person, also off the walking line.
- The margin, the strip and the gap are parameters, to be set from recordings of the real LiDAR.
- Counting steps from the LiDAR is not part of this decision. It is added after comparing with a hand count.

## Alternatives considered

- **Recognise legs by shape** (round clusters of a leg's width, as the common leg detectors do): needs more points per leg than we get at 2.5 m, and a trained model.
- **Only keep points on the walking path, without the empty-room scan**: simpler, but a chair or a table leg near the path would count as a person.
- **Park the robot closer to get more points per leg**: the camera then sees a shorter part of the path. Can be revisited if the real scans are too noisy.
