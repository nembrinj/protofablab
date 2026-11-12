# lidar_person_tracker

Finds a walking person in the LiDAR scans of the parked robot and publishes where they are along the walking path and how fast they move. How it works and why: [ADR 5](../../../docs/decisions/0005-find-the-walker-against-the-empty-room.md).

| | Name | Type |
| --- | --- | --- |
| Reads | `/scan` | `sensor_msgs/LaserScan` |
| Publishes | `/person/state` | `gait_interfaces/PersonState` |
| Service | `/lidar_person_tracker/relearn_background` | `std_srvs/srv/Trigger` |

The node publishes nothing while it learns the empty room (the first 15 scans). Position and speed are `NaN` unless exactly one person is on the path.

- `lidar_person_tracker/tracker.py`: the logic, without ROS.
- `lidar_person_tracker/tracker_node.py`: the ROS 2 node around it.
- `config/params.yaml`: all parameters, with their meaning.

## Run

```bash
ros2 launch lidar_person_tracker tracker.launch.py
ros2 topic echo /person/state
```

The robot must face the walking path, and the path must be clear for the first 3 seconds. If the camera does not point the same way as the robot's front, set `facing_angle_rad`.

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

Setup: robot parked about 2.5 m from a straight walking line of at least 5 m, facing it; `turtlebot3_bringup` running on the robot; the launch command above running on the laptop.

1. Keep the path clear and start the node. Expected: "Empty room learned" after about 3 seconds, then `detected: false` on `/person/state`.
2. Stand on the line in front of the robot. Expected: `people_count: 1`, `position_m` near 0, `speed_mps` near 0.
3. Stand 1 m to the robot's left, then 1 m to its right (measured with tape). Expected: `position_m` near +1, then near -1, within 0.15 m.
4. Walk a taped 4 m along the line, timed with a stopwatch, in both directions. Expected: `people_count` stays 1; `speed_mps` is within 0.2 m/s of distance ÷ time, positive towards the robot's left.
5. Walk with a second person 1.5 m behind. Expected: `people_count: 2`, position and speed `NaN`.
6. Stand between the robot and the line, about 1 m from the robot. Expected: `detected: false`.
7. Put a chair on the line. Expected: it counts as a person. Call `ros2 service call /lidar_person_tracker/relearn_background std_srvs/srv/Trigger`. Expected: after 3 seconds, `detected: false`.
8. Unplug the LiDAR or stop the bringup. Expected: a "No LiDAR scan" warning within a few seconds.
9. Record `ros2 bag record /scan` during steps 2 to 5, to set the parameters and to build the rough step count later.

### First results (2026-11-12)

A first, informal test in the lab, without tape marks or stopwatch. Robot: LDS-02 LiDAR, about 9 scans per second, 240 beams (1.5° apart). The lab spot had a wall about 1.9 m in front of the robot, so the person walked about 1.0–1.4 m from it, not 2.5 m. The results below come from replaying the recordings with the walking line moved to that distance (`path_distance_m` 1.05–1.2, `corridor_half_width_m` 0.45, `zone_half_length_m` 2.0–2.2; `facing_angle_rad` 3.14159 for the third recording, where the robot faced away from the path).

- One person walking back and forth: tracked through every pass, never lost in the middle of the zone. Position moves smoothly from one end of the zone to the other.
- Standing still: position steady within ±0.01 m, speed 0.00 m/s.
- Walking speed: 0.6–0.95 m/s on the straight part, near 0 at the turns. Plausible, but not yet checked with a stopwatch.
- Empty room: no person reported in any scan.
- At 1.2 m a leg gives 5–14 points. How many it gives at 2.5 m is still unknown.
- With the default settings (line at 2.5 m), the tracker correctly reported nobody, because nobody walked there.
- In the third recording, 130 of 1100 scans reported two people. In most of them, a second group of points stood about 1.7 m along the path, and the person was turning or standing. To check: was a second person there, or does one person sometimes split into two?

Still to do: steps 1–9 above at 2.5 m, in a place with at least 3 m of free floor in front of the robot.
