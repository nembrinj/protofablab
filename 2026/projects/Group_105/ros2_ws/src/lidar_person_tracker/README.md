# lidar_person_tracker

Finds a walking person in the LiDAR scans and publishes where they are along the walking path, how fast they move, and how many steps they took. It works with the robot parked, and while it drives beside the walker. How it works and why: [ADR 4](../../../docs/decisions/0004-find-the-walker-against-the-empty-room.md) (parked) and [ADR 5](../../../docs/decisions/0005-follow-the-walker-with-the-odometry.md) (driving).

| | Name | Type |
| --- | --- | --- |
| Reads | `/scan` | `sensor_msgs/LaserScan` |
| Reads | `/odom` | `nav_msgs/Odometry` |
| Publishes | `/person/state` | `gait_interfaces/PersonState` |
| Service | `/lidar_person_tracker/relearn_background` | `std_srvs/srv/Trigger` |

The node publishes nothing while it learns the empty room: for the first 15 scans (about 2 s) it remembers how far away the walls and furniture are in each direction. This is not a map of the room (no SLAM), only one remembered scan to compare with. Position and speed are `NaN` unless exactly one person is on the path.

`steps` counts the walker's steps since the empty room was learned. Take the difference between two messages to get the steps in between. It does not go up while nobody or two people are on the path, and steps while turning on the spot are mostly not counted.

- `lidar_person_tracker/tracker.py`: the logic, without ROS.
- `lidar_person_tracker/steps.py`: the step count, without ROS.
- `lidar_person_tracker/odometry.py`: where the robot was when a scan was taken, without ROS.
- `lidar_person_tracker/tracker_node.py`: the ROS 2 node around it.
- `config/params.yaml`: all parameters, with their meaning.

## Run

```bash
ros2 launch lidar_person_tracker tracker.launch.py
ros2 topic echo /person/state
```

The robot must face the walking path, and the path must be clear for the first 2 seconds. The node watches the floor from 0.5 m to 3.5 m in front of the robot, and 2.5 m to each side. If the camera does not point the same way as the robot's front, set `facing_angle_rad`.

## Modes

The `mode` setting chooses how the walker is found. It can also be changed while the node runs: `ros2 param set /lidar_person_tracker mode driving`. The other settings are read at start-up only; the node refuses to change them while it runs.

| Mode | Robot | How the walker is found |
| --- | --- | --- |
| `static` (default) | Parked | In every scan, against the empty room |
| `driving` | Parked until the walker is found, then driving | Found against the empty room, then followed: in each scan, only the points within 0.6 m of where the walker should be, placed in the odom frame with the odometry |

In driving mode, `position_m` is from the robot and `speed_mps` is over the floor, so the walker's speed does not change when the robot drives. When the walker is not seen for 1 s, they are lost.

In both modes, the empty room is only used while `/odom` shows the robot within 5 cm and about 6° of where it learned it (the heading in the odometry can drift a little while the robot stands still). Once it has moved, the node reports nobody and warns until the room is learned again (robot parked, path clear, `relearn_background`). Without any `/odom`, the robot is taken to be parked.

## Tests

`colcon test --packages-select lidar_person_tracker` runs the tests and the lint. The tests use made-up scans; they check the logic, not the accuracy on the real LiDAR.

Without ROS, from this folder: `PYTHONPATH=. python3 -m pytest test/test_tracker.py test/test_steps.py test/test_odometry.py` (needs `numpy` and `pytest`).

If `colcon test` fails with a `pytest` plugin error, a newer `pytest` installed with `pip --user` is clashing with the one of ROS. Run `PYTHONNOUSERSITE=1 colcon test`, or remove it with `pip uninstall pytest`.

## Replay a recording

Record `ros2 bag record -o walk_test /scan /odom /person/state` on the robot, then try other settings at home:

```bash
ros2 run lidar_person_tracker replay_bag ~/walk_test path_distance_m=1.2 facing_angle_rad=3.14159
```

It prints what the tracker would have said every half second, how many scans showed 0, 1, 2 ... people, and how many steps it counted. Add `--csv walk_test.csv` to also write every scan (people, position, speed, spread of the feet, steps, following, robot moved) to a file, for plots. Add `--mode driving` to try the driving mode on a recording with `/odom`.

## Manual test on the real robot

Setup: robot parked 1 to 3 m from a straight walking line of at least 5 m, facing it; `turtlebot3_bringup` running on the robot; the launch command above running on the laptop.

1. Keep the path clear and start the node. Expected: "Empty room learned" after about 2 seconds, then `detected: false` on `/person/state`.
2. Stand on the line in front of the robot. Expected: `people_count: 1`, `position_m` near 0, `speed_mps` near 0. Shift your weight from foot to foot. Expected: `steps` stays the same.
3. Take two steps to the robot's left, then go back to the middle and take two steps to its right. Expected: `position_m` goes up, back near 0, then down.
4. Walk past the robot at a normal pace, in both directions. Expected: `people_count` stays 1; `speed_mps` about 0.5 to 1.5 m/s while walking, positive towards the robot's left, near 0 at the turns; `steps` goes up by one with each step.
5. Walk with a second person 1.5 m behind. Expected: `people_count: 2`, position and speed `NaN`.
6. Stand right next to the robot, closer than 0.5 m. Expected: `detected: false`.
7. Put a chair on the line. Expected: it counts as a person. Call `ros2 service call /lidar_person_tracker/relearn_background std_srvs/srv/Trigger`. Expected: after 2 seconds, `detected: false`.
8. Unplug the LiDAR or stop the bringup. Expected: a "No LiDAR scan" warning within a few seconds.
9. Leave the robot parked for 10 minutes with the node running. Expected: no "robot has moved" warning. If it comes, the heading in the odometry drifted: note how much (`ros2 topic echo /odom`) and raise `parked_rad`.
10. Record `ros2 bag record /scan` during steps 2 to 5, and during a few walks past the robot where a second person counts the steps by hand and times the walk with a phone. The hand counts check the step count. Compare steps per minute, not the number of steps: the tracker only sees the part of the walk inside the watched zone.

### Driving mode

Setup: as above, with the camera side of the robot towards the path (`facing_angle_rad`), the mode switched with `ros2 param set /lidar_person_tracker mode driving` while the robot is parked, and `ros2 run turtlebot3_teleop teleop_keyboard` to drive the robot slowly along the path, beside the walker, with the default speed limits.

1. Before the walker comes, drive the robot 20 cm forward. Expected: a warning that the robot has moved, and `detected: false`. Call `relearn_background` with the path clear.
2. With the robot parked, let the walker walk in front of it. Expected: "Following the walker".
3. Drive beside the walker at about their speed, forward, then backward on the walk back. Expected: `people_count: 1` all along; `position_m` changes only as much as the gap between the robot and the walker; `speed_mps` is the walker's speed, not the difference to the robot's.
4. Stop the robot while the walker walks on. Expected: `position_m` grows; `speed_mps` stays the walker's speed.
5. Let the walker step out of the LiDAR's view while the robot drives. Expected: within about 1 s "Walker lost", then the robot-moved warning.
6. Record `ros2 bag record /scan /odom` during steps 3 and 4, and replay it with `--mode driving` to set `follow_radius_m` and `follow_lost_s`.

### First results (2026-11-12)

A first, informal test in the lab: four recordings of 43 to 118 s. Robot: LDS-02 LiDAR, about 9 scans per second, 240 beams (1.5° apart). The person walked 1.0–1.9 m from the robot. The results below come from replaying the recordings with the default settings (`facing_angle_rad` 3.14159 for the third recording, where the robot faced away from the path).

- One person walking back and forth: tracked through every pass, never lost in the middle of the zone. Position moves smoothly from one end of the zone to the other.
- Standing still: position steady within ±0.01 m, speed 0.00 m/s.
- Walking speed: 0.6–0.95 m/s on the straight part, near 0 at the turns.
- Empty room: no person reported in any scan.
- At 1–2 m a person gives 4–14 points. How many at 2.5–3.5 m is still unknown; the beams are 6.5 cm apart at 2.5 m and 9 cm apart at 3.5 m.
- A second person walked in by accident during the third recording. The tracker reported two people for those scans (134 of about 1100), and one person in the other three recordings throughout.
- During the test itself the node watched only 1.75–3.25 m in front of the robot, so it reported nobody. The default strip was widened to 0.5–3.5 m because of this.

Steps, from the same replays (2026-11-20). There is no hand count for these recordings yet, so this shows that the count is plausible, not how accurate it is:

- On the straight parts of a walk, each time the feet open and close again is counted once. In `walk_test2`, the three walks through the zone gave 7, 7 and 6 steps over 2.8–3.1 m: 0.39–0.52 m per step, 90–107 steps per minute.
- Standing still: no steps, also with the feet apart (`walk_test2`, 9–14 s).
- Steps while turning at the ends are mostly not counted.
- Scans where the points spread more than 1 m (a second person or an object right next to the walker) are skipped.

Still to do: steps 1–9 above live on the robot with the current settings (so far only seen in replays), and walks with a hand step count.
