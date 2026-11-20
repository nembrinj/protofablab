# session_reporter

Sends the numbers of each walk as JSON over MQTT to the [dashboard](../../../dashboard/README.md) ([ADR 3](../../../docs/decisions/0003-mqtt-and-grafana-dashboard.md)).

| | Name | Type |
| --- | --- | --- |
| Reads | `/session/walk_summary` | `gait_interfaces/WalkSummary` |
| Sends | `rehab/<patient_id>/walk` | MQTT, JSON |

Example message:

```json
{"session_id": "20261120-151002", "walk": 2, "measured": true, "duration_s": 5.01,
 "steps": 7, "cadence_spm": 95.7, "speed_mps": 0.67, "stops": 0}
```

A number that is not known (`NaN`) is left out of the message. `patient_id` is an anonymised ID, never a name.

- `session_reporter/reporter.py`: the messages, without ROS.
- `session_reporter/reporter_node.py`: the ROS 2 node.
- `config/params.yaml`: broker address and port, patient ID.

Needs `python3-paho-mqtt` (`sudo apt install python3-paho-mqtt`, or `rosdep install --from-paths src`).

## When the broker cannot be reached

The node keeps trying to connect. Walks wait in a queue and are sent once the broker is back, so a walk is not lost when the dashboard starts late or Wi-Fi drops. While there is no broker, it warns every 10 s, with the number of walks waiting. If the node stops with walks still waiting, it names them in an error.

## Run

Started by `ros2 launch walk_session session.launch.py broker_host:=<IP>`. Alone:

```bash
ros2 run session_reporter reporter_node --ros-args -p broker_host:=<IP> -p patient_id:=p01
```

## Tests

`colcon test --packages-select session_reporter` runs the tests and the lint. Without ROS, from this folder: `PYTHONPATH=. python3 -m pytest test/test_reporter.py`.

The manual test on the real robot is part of the [`walk_session` manual test](../walk_session/README.md#manual-test-on-the-real-robot) (steps 2 and 7).
