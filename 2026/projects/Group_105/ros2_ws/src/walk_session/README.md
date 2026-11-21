# walk_session

Follows a walking session from what the LiDAR tracker says about the walker: when a walk starts and ends, when the walker stops, and the numbers of each walk. When the walker stops, it sends a sentence to speak.

| | Name | Type |
| --- | --- | --- |
| Reads | `/person/state` | `gait_interfaces/PersonState` |
| Publishes | `/session/walk_summary` | `gait_interfaces/WalkSummary`, when a walk ends |
| Publishes | `/session/cue` | `std_msgs/String`, the text to speak |
| Publishes | `/walk_session/markers` | `visualization_msgs/MarkerArray`, the session's panel for RViz |

- `walk_session/session.py`: the logic, without ROS.
- `walk_session/session_node.py`: the ROS 2 node around it.
- `walk_session/view.py`: the panel's text, without ROS.
- `walk_session/cue_speaker.py`: speaks `/session/cue` on the laptop, until `audio_cues` speaks it on the robot.
- `config/params.yaml`: all parameters, with their meaning.
- `launch/session.launch.py`: the whole laptop side of a session, LiDAR only: `lidar_person_tracker`, this node, `cue_speaker` and `session_reporter`; RViz with `rviz:=true`.
- `rviz/session.rviz`: the RViz layout.

## Walks and stops

A walk is one way along the path. With the robot parked, the LiDAR only sees the part of the path in front of it, about 5 m, so a walk is the part of one way that it saw.

| What | How it is noticed (default settings) |
| --- | --- |
| Walk starts | One person moves faster than 0.3 m/s the same way for 1 s |
| Walk ends | The person moves the other way for 1 s: the walk ends where they turned. Or the person is not seen for 2 s: they left the zone. |
| Stop | During a walk: slower than 0.15 m/s, and no new step, for 3 s. Spoken once: "Take your time, start again when ready". |
| Not measured | Two people were seen during the walk, or the walker was lost nearer than 1.8 m to the middle of the zone, not at its edge |

The numbers of a walk:

- duration: from start to end, stops included;
- steps: the difference in the tracker's step count;
- steps per minute and speed: over the time the walker moved (faster than 0.15 m/s), so stops, short pauses and turning on the spot do not lower them;
- stops.

The session ID is the time the node started, on the laptop's clock. The times inside the session come from the scans, so a recording can be replayed. When the node stops during a walk, that walk is not sent, and the node says so.

## Run

On the laptop, in a terminal of the desktop (the speech needs the desktop's sound), with `turtlebot3_bringup` running on the robot and the [dashboard](../../../dashboard/README.md) running:

```bash
ros2 launch walk_session session.launch.py broker_host:=<IP of the computer with the dashboard>
```

`broker_host` defaults to `localhost`. When `audio_cues` speaks the cues on the robot, add `speak_on_laptop:=false`. `cue_speaker` uses `spd-say` from Ubuntu Desktop; set `speech_command` to use another program.

To try it without the robot, start the launch command, then replay a recording in a second terminal: `ros2 bag play ~/walk_test2 --topics /scan`. The tracker learns the empty room from the first 2 s of the recording.

### In RViz

Add `rviz:=true` to the launch command to watch the session. RViz shows it from above, in the `walking_path` frame of the [tracker](../lidar_person_tracker/README.md#in-rviz): the path lies across the screen, the robot in the middle above it.

- The robot's axes (red ahead, green to its left) and the LiDAR's points, in grey.
- From the tracker: the watched strip, the walker's points, and a status line under the strip.
- From this node, a panel above the robot: the session ID, the walk in progress (walking or stopped, its steps and stops so far, the cue while the walker is stopped), and the numbers of the last walk.

A walk that has ended can have a step or two fewer than the panel showed while it was in progress: a walk ends where the walker turned, so the steps after the turn go to the next walk.

## Tests

`colcon test --packages-select walk_session` runs the tests and the lint. The tests use made-up tracker messages.

Without ROS, from this folder: `PYTHONPATH=. python3 -m pytest test/test_session.py test/test_view.py` (needs `pytest`).

## Replays of the first recordings (2026-11-20)

The recordings of 2026-11-12, replayed through the whole launch file, with the dashboard running:

| Recording | Walks | What the session saw |
| --- | --- | --- |
| `walk_test2` | 4 measured | Walk 1: walked in, stood still for 5 s (one stop, spoken), walked on. Walks 2 to 4: 86–96 steps per minute, 0.63–0.80 m/s |
| `walk_test4` | 4 measured | A slow walk: 49–78 steps per minute, 0.35–0.60 m/s, one stop in walk 3 |
| `walk_test3` | 2 measured, 2 not measured | Not measured: a second person walked in. From 72 s on, short moves and pauses of 1–3 s, which make one long walk without a stop |

## Manual test on the real robot

Setup: as in the [tracker's manual test](../lidar_person_tracker/README.md#manual-test-on-the-real-robot), with the launch command above instead of the tracker's, and the dashboard open in a browser.

1. Start with the path clear. Expected: "Session ... started", "Empty room learned", "Connected to the MQTT broker".
2. Walk through the whole zone and out of it at a normal pace. Expected: "Walk 1 started" about 1.5 s after you start walking, "Walk 1 ended: measured" about 2 s after you leave the zone, "Walk 1 sent to the dashboard", and within 10 s a new row in the dashboard's Sessions table (reload the page to pick the new session in the Session menu).
3. Walk into the zone, turn around in the middle, and walk back out. Expected: two walks; the first one ends where you turned.
4. Walk, stand still in the middle for 5 s, then walk on. Expected: about 3 s after you stop, "the walker stopped", and the laptop says "Take your time, start again when ready". The walk has 1 stop, and its steps per minute are close to those of a walk without a stop.
5. Walk, pause for 2 s, walk on. Expected: no stop.
6. Walk with a second person in the zone. Expected: the walk is not measured.
7. Stop the broker (`docker compose stop mosquitto` in `dashboard/`), do a walk, then start it again. Expected: "No MQTT broker" warnings with 1 walk waiting, then "Walk ... sent to the dashboard" once the broker is back.
8. Press Ctrl+C in the middle of a walk. Expected: "Walk ... had not ended; it is not sent".
9. During a few walks, a second person counts the steps by hand and times the walk with a phone. Compare the hand count's steps per minute with the walk's.
10. Start with `rviz:=true` and do steps 2 and 4 again. Expected: the path across the screen below the robot, the walker's points green and moving the same way as you, the status line's position going up as you walk to the robot's left, the steps going up as you walk, and the panel showing the walk, then "stopped" with the cue, then the last walk's numbers. With a second person: orange points. Then switch the tracker to driving mode and drive the robot beside you: the robot stays in the middle, and the status line says "following".
