# lidar_person_tracker

Finds a walking person in the LiDAR scans of the parked robot and publishes where they are along the walking path and how fast they move. How it works and why: [ADR 4](../../../docs/decisions/0004-find-the-walker-against-the-empty-room.md).

| | Name | Type |
| --- | --- | --- |
| Reads | `/scan` | `sensor_msgs/LaserScan` |
| Publishes | `/person/state` | `gait_interfaces/PersonState` |
| Service | `/lidar_person_tracker/relearn_background` | `std_srvs/srv/Trigger` |

The node publishes nothing while it learns the empty room: for the first 15 scans (about 2 s) it remembers how far away the walls and furniture are in each direction. This is not a map of the room (no SLAM), only one remembered scan to compare with. Position and speed are `NaN` unless exactly one person is on the path.

- `lidar_person_tracker/tracker.py`: the logic, without ROS.
- `lidar_person_tracker/tracker_node.py`: the ROS 2 node around it.
- `config/params.yaml`: all parameters, with their meaning.

## Run

```bash
ros2 launch lidar_person_tracker tracker.launch.py
ros2 topic echo /person/state
```

The robot must face the walking path, and the path must be clear for the first 2 seconds. The node watches the floor from 0.5 m to 3.5 m in front of the robot, and 2.5 m to each side. If the camera does not point the same way as the robot's front, set `facing_angle_rad`.

## Tests

`colcon test --packages-select lidar_person_tracker` runs the tests and the lint. The tests use made-up scans; they check the logic, not the accuracy on the real LiDAR.

Without ROS, from this folder: `PYTHONPATH=. python3 -m pytest test/test_tracker.py` (needs `numpy` and `pytest`).

If `colcon test` fails with a `pytest` plugin error, a newer `pytest` installed with `pip --user` is clashing with the one of ROS. Run `PYTHONNOUSERSITE=1 colcon test`, or remove it with `pip uninstall pytest`.

## Replay a recording

Record `ros2 bag record -o walk_test /scan /person/state` on the robot, then try other settings at home:

```bash
ros2 run lidar_person_tracker replay_bag ~/walk_test path_distance_m=1.2 facing_angle_rad=3.14159
```

It prints what the tracker would have said every half second, and how many scans showed 0, 1, 2 ... people.

## Manual test on the real robot

Setup: robot parked 1 to 3 m from a straight walking line of at least 5 m, facing it; `turtlebot3_bringup` running on the robot; the launch command above running on the laptop.

1. Keep the path clear and start the node. Expected: "Empty room learned" after about 2 seconds, then `detected: false` on `/person/state`.
2. Stand on the line in front of the robot. Expected: `people_count: 1`, `position_m` near 0, `speed_mps` near 0.
3. Take two steps to the robot's left, then go back to the middle and take two steps to its right. Expected: `position_m` goes up, back near 0, then down.
4. Walk past the robot at a normal pace, in both directions. Expected: `people_count` stays 1; `speed_mps` about 0.5 to 1.5 m/s while walking, positive towards the robot's left, near 0 at the turns.
5. Walk with a second person 1.5 m behind. Expected: `people_count: 2`, position and speed `NaN`.
6. Stand right next to the robot, closer than 0.5 m. Expected: `detected: false`.
7. Put a chair on the line. Expected: it counts as a person. Call `ros2 service call /lidar_person_tracker/relearn_background std_srvs/srv/Trigger`. Expected: after 2 seconds, `detected: false`.
8. Unplug the LiDAR or stop the bringup. Expected: a "No LiDAR scan" warning within a few seconds.
9. Record `ros2 bag record /scan` during steps 2 to 5, and during a few walks past the robot where a second person counts the steps by hand and times the walk with a phone. The hand counts check the step count.

### First results (2026-11-12)

A first, informal test in the lab: four recordings of 43 to 118 s. Robot: LDS-02 LiDAR, about 9 scans per second, 240 beams (1.5° apart). The person walked 1.0–1.9 m from the robot. The results below come from replaying the recordings with the default settings (`facing_angle_rad` 3.14159 for the third recording, where the robot faced away from the path).

- One person walking back and forth: tracked through every pass, never lost in the middle of the zone. Position moves smoothly from one end of the zone to the other.
- Standing still: position steady within ±0.01 m, speed 0.00 m/s.
- Walking speed: 0.6–0.95 m/s on the straight part, near 0 at the turns.
- Empty room: no person reported in any scan.
- At 1–2 m a person gives 4–14 points. How many at 2.5–3.5 m is still unknown; the beams are 6.5 cm apart at 2.5 m and 9 cm apart at 3.5 m.
- A second person walked in by accident during the third recording. The tracker reported two people for those scans (134 of about 1100), and one person in the other three recordings throughout.
- During the test itself the node watched only 1.75–3.25 m in front of the robot, so it reported nobody. The default strip was widened to 0.5–3.5 m because of this.

Still to do: steps 1–9 above live on the robot with the current settings (so far only seen in replays), and walks with a hand step count.
