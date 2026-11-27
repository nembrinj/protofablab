# 5. While the robot drives, the tracker follows the walker's points with the odometry

- Status: accepted
- Date: 2026-11-20

## Context

In stage 2 the robot drives beside the walker ([ADR 1](0001-robot-drives-beside-the-walker.md)). The tracker's static mode compares each scan with the scan of the empty room ([ADR 4](0004-find-the-walker-against-the-empty-room.md)). Once the robot moves, the room moves in the scan, and walls and furniture would look like people.

The robot counts its wheel turns: its odometry (`/odom`) says where it is and which way it faces, from where it started, in a fixed frame called odom. Odometry drifts slowly, but over a few seconds it is exact enough.

## Decision

The tracker gets a second mode, `driving`. The mode is chosen by hand with the `mode` setting, also while the tracker runs.

1. The walker is first found as in the static mode, while the robot still stands where it learned the empty room.
2. From then on, each scan is put into the odom frame with the robot's pose at the time of the scan, so the robot's own movement is taken out. Only the points within 0.6 m of where the walker should be belong to the walker: where they were last, moved on by their walking speed. Everything else is ignored.
3. The directions of the path are fixed in the odom frame at that moment. The tracker reports the walker's position along the path from the robot, which the follower needs to keep the gap, and the walking speed over the floor, from the positions in the odom frame. The steps are counted as before.
4. When no points are near for 1 s, the walker is lost. The tracker finds them again only with the empty room: once the robot is back where it learned it, or after it has learned it again.

In both modes, the empty room is only used while the odometry shows the robot within 5 cm and about 6° of where it learned it. Otherwise the tracker reports nobody and asks for the room to be learned again. This check uses the last pose the odometry gave, so a message that comes late over Wi-Fi does not matter. Without any odometry, the robot is taken to be parked.

## Consequences

- No map is needed, and nothing has to be set up for each room.
- The walker must be found before the robot drives: the session starts with the robot parked and the path clear.
- Anything within 0.6 m of the walker is taken as part of them, the room included: a chair right beside the path, a wall they walk along, a second person right next to them. A second person further away is ignored, not counted, so in driving mode the tracker reports one person or nobody.
- When the walker walks on, the track can stay on such an object. In the first recording on the real robot (2026-11-27), it stayed on a corner of the room for about 20 s and reported a person standing still, while the walker had walked on. A follower would have kept its distance to that corner. The follower is not used with the driving mode until this is fixed.
- Following uses the odometry only from one scan to the next, so slow drift does not matter. A wheel that slips makes the walker jump; if the jump is bigger than 0.6 m, the walker is lost.
- When the walker is lost while the robot drives, the tracker does not guess. The follower must stop.
- The LiDAR sits a few centimetres from the robot's centre; this is ignored.
- The heading in the odometry can drift a little while the robot stands still, so the turn limit is wider than the distance limit. A robot turned by less than that, for example by a bump, is not noticed. How much the heading drifts is checked on the real robot.
- In that recording, the robot drove at 0.15 m/s towards the corner, and the speed over the floor stayed at 0.00 ± 0.05 m/s: the odometry takes the robot's own movement out. More drives recorded beside a walker, with `/scan` and `/odom`, set the radius and the time.
- Choosing the mode by hand keeps the behaviour easy to predict while it is tested.

## Alternatives considered

- **A SLAM map, and comparing each scan with it**: needs a good map of each room before the first session.
- **Recognise legs by their shape**: as in [ADR 4](0004-find-the-walker-against-the-empty-room.md), too few points per leg.
- **Take the robot's movement out by matching each scan with the one before**: needs no odometry, but is harder to build, and fails along long, plain walls.
- **Keep only the points that moved since the last scan**: also finds the walker, but loses them as soon as they stand still.
- **Switch the mode automatically when the odometry shows the robot moving**: one setting less, but harder to predict while testing.
