# Adaptive Training Assistant

A TurtleBot3 that acts as a personal exercise coach. It finds the user,
positions itself in front of them, counts their repetitions, watches their
heart rate and gives short spoken-style feedback that adapts to how the
user responds.

## What it does

1. **Finds the user.** The robot looks around with a tilting camera and
   turns until it sees a person's upper body.
2. **Positions itself.** It centres the person in the image and drives to
   about 1 m from them, so the whole upper body is visible. Then it stays
   still.
3. **Starts on a gesture.** The user shows a **thumbs up 👍** (held 1 s).
4. **Coaches the exercise.** It counts bilateral arm raises (both arms
   above the shoulders and back down), measures the time per repetition
   and the pace, and encourages the user.
5. **Adapts to the heart rate.** Halfway through, it compares the
   heart-rate rise with the user's previous sessions. If the body responds
   too little or too much, or the heart rate goes above a safe maximum, it
   stops the exercise and suggests a break.
6. **Stops** on a **thumbs down 👎**, after 10 repetitions, or on a break,
   and announces the result, e.g. *"You did 8 repetitions in 1 minute 20
   seconds, at 6 repetitions per minute."*

All feedback is published as sentences on `/training/feedback`, ready for
a speaker, and shown on two dashboards: an OpenCV window and a Node-RED
web page that also works on a phone.

## Hardware

| Part | Role |
|------|------|
| TurtleBot3 Burger (Raspberry Pi) | Mobile base, odometry |
| USB camera, mounted at the **back** of the robot | Person, pose and gesture detection |
| SG90 servo + ESP32 (UDP over Wi-Fi) | Tilts the camera up/down |
| 3D-printed parts (`models/`) | Camera holder, camera pivot support, servo mount |
| Heart-rate sensor *(planned)* | Simulated for now by `heart_rate_simulator` |
| Speaker *(planned)* | Will read `/training/feedback` aloud |

The heavy processing (MediaPipe) runs on a Mac; the robot only streams the
camera and drives the wheels.

## Session flow

```
SEARCHING ──person seen──> CENTERING ──centred + 1 m──> READY
    ▲                                                     │ 👍
    └──────── person lost > 2 s ─────────                 ▼
                                    FINISHED <──👎 / 10 reps / break── IN_PROGRESS
                                    (5 s, then back to READY)
```

- In `READY`, `IN_PROGRESS` and `FINISHED` the robot and the camera servo
  do not move.
- `READY` is sticky: once reached, it stays on while the person is still
  visible (small movements or the gesture itself do not send the robot back
  to `CENTERING`). It only ends when the person is lost for 2 s.

## Architecture

Nine ROS 2 Jazzy nodes in one package, plus a message package.

### Main data flow

Every line is a ROS topic, except the UDP link to the ESP32.

``` mermaid
flowchart TB
    subgraph SENSORS["Robot sensors"]
        direction LR
        ODOM["Odometry"]
        CAM["USB camera"]
        HR["heart_rate_simulator<br/>(later: real sensor)"]
    end

    subgraph VISION["Vision (MediaPipe)"]
        direction LR
        PD["person_detection"]
        GEST["gesture"]
    end

    subgraph POSITIONING["1 · Find and centre the person"]
        direction TB
        BASE["robot_base"]
        TILT["servo_tilt"]
        DRV["servo_driver"]
        WHEELS["TurtleBot3 wheels"]
        ESP["ESP32 servo"]
        BASE -->|/cmd_vel| WHEELS
        BASE -->|/servo/search_enable| TILT
        TILT -->|"/servo/aligned<br/>/servo/scan_complete"| BASE
        TILT -->|/servo/angle| DRV
        DRV -->|UDP| ESP
    end

    subgraph EXERCISE["2 · Run the exercise"]
        direction TB
        LOGIC["training_logic"]
        EX["exercise_counter"]
        LOGIC -->|/exercise/enable| EX
        EX -->|/exercise/rep_event| LOGIC
    end

    CAM -->|/image_raw/compressed| PD
    CAM -->|/image_raw/compressed| GEST
    ODOM -->|/odom| BASE
    PD -->|/person/position| BASE
    PD -->|/person/position| TILT
    PD -->|/person/position| LOGIC
    PD -->|/person/landmarks| EX
    BASE -->|/person/ready| LOGIC
    GEST -->|/gesture| LOGIC
    HR -->|/heart_rate| LOGIC
```

### State and display

`training_logic` shares the current state on `/training/status` with the
nodes that change behaviour with it. `gesture` listens to the exercise
counter to ignore gestures while the arms are raised.

``` mermaid
flowchart LR
    LOGIC["training_logic"]
    STATUS(["/training/status"])
    GEST["gesture"]
    BASE["robot_base"]
    TILT["servo_tilt"]
    EX["exercise_counter"]
    PD["person_detection"]
    HR["heart_rate_simulator"]
    DASH["training_dashboard<br/>(OpenCV window)"]
    RB["rosbridge"]
    NR["Node-RED dashboard"]

    LOGIC -->|"current state"| STATUS
    STATUS --> GEST
    STATUS --> BASE
    STATUS --> TILT
    STATUS --> DASH
    EX -->|/exercise/feedback| GEST

    PD -->|/person/preview/compressed| DASH
    BASE -->|/person/distance| DASH
    EX -->|/exercise/rep_event| DASH
    HR -->|/heart_rate| DASH
    LOGIC -->|"/training/feedback<br/>/training/feedback_level"| DASH

    STATUS --> RB
    LOGIC -->|/training/feedback| RB
    EX -->|/exercise/rep_event| RB
    HR -->|/heart_rate| RB
    RB -->|"websocket :9090"| NR
```

### Nodes

| Node | What it does |
|------|--------------|
| `person_detection` | Runs MediaPipe Pose once per frame. Publishes the shoulder midpoint and shoulder width (`/person/position`), all 33 landmarks (`/person/landmarks`) and an annotated preview. A person needs at least 3 clearly visible body points; the *upper body* needs both shoulders and the nose, so chairs are not taken for people. |
| `robot_base` | Drives the wheels. Searches (centre → left → right turns while the camera sweeps), turns to centre the person, then drives to the target distance. Publishes `/person/ready` and `/person/distance`. Stops without odometry, and sends a stop command on Ctrl+C. |
| `servo_tilt` | Tilts the camera so the shoulders are at the image centre (`/servo/aligned`), and sweeps 70°–100° when `robot_base` asks for a search. Decides the servo angle (`/servo/angle`) within its limits, moving it smoothly. |
| `servo_driver` | Forwards the angle to the ESP32 over UDP. The ESP32 only moves the servo there, so the limits and the start angle live in one place (`servo_tilt`). |
| `gesture` | MediaPipe Gesture Recognizer, only in `READY` and `IN_PROGRESS`: thumbs up = `start`, thumbs down = `stop`, each held 1 s. |
| `exercise_counter` | Counts arm raises from the landmarks while enabled. Publishes one JSON `/exercise/rep_event` per repetition: `repetitions`, `repetition_duration_seconds`, `pace_reps_per_minute`, `exercise_duration_seconds`. |
| `training_logic` | The session flow, the heart-rate checks and the feedback. Publishes `/training/status`, `/training/feedback`, `/training/feedback_level`. |
| `heart_rate_simulator` | Stand-in sensor: `/heart_rate` (`std_msgs/Float32`, bpm) once per second. A real sensor only has to publish on the same topic. |
| `training_dashboard` | OpenCV window: camera view, state, repetitions, pace, time, heart rate, distance, feedback. |

**Custom messages** (`adaptive_training_assistant_msgs`):
`PersonPosition` (detected, upper body detected, shoulder midpoint
`tracking_x/y` in 0..1, `shoulder_width` as a fraction of the image) and
`PoseLandmarks` (the 33 MediaPipe points, so Pose runs only once).

## Technical points

**Distance without a depth camera.** `robot_base` estimates the distance
from the shoulder width in the image:
`distance = SHOULDER_WIDTH_M / (2 · tan(HFOV/2) · shoulder_width)`, with an
average shoulder width of 0.38 m and a 60° field of view. Readings outside
0.4–4 m are ignored. The robot first centres the person, then corrects the
distance, so a half-visible person is never used to drive.

**Back-facing camera.** The camera looks out of the robot's back, so
driving "away" from the person is a forward command
(`LINEAR_DIRECTION = -1`).

**Stable decisions.** Noisy single frames should not make the robot jump
between states:
- the tracked point is smoothed (low-pass filter),
- the servo uses a deadband with hysteresis, and must be centred for
  0.5 s before it counts as aligned,
- `/person/ready` turns on only after 0.5 s and then stays on (sticky),
- a gesture must be held 1 s, and short misses (< 0.4 s) do not restart it.

**Safe stopping.** The TurtleBot keeps its last speed until it gets a new
command, so `robot_base` handles Ctrl+C itself and sends a stop to the
wheels before ROS shuts down. Without odometry for 0.5 s the robot stops.
The servo needs nothing special: it receives an angle, not a speed, so when
the messages stop it simply stays where it is.

**Network.** All nine nodes run on the Mac, so the topics between them
stay on the Mac. Only the robot's own topics cross the Wi-Fi between the
Raspberry Pi and the Mac: the camera images (`/image_raw/compressed`) and
`/odom` to the Mac, `/cmd_vel` back to the robot, plus the UDP servo
commands to the ESP32. On a weak phone hotspot, camera images can arrive
with gaps of ~0.6 s (check with `ros2 topic hz`). The timeouts and the
sticky `READY` are chosen so such gaps do not change the state.

### Heart-rate adaptation

The heart-rate sensor (or the simulator) only sends a bpm value every
second; it knows nothing about the exercise. `training_logic` makes the
decisions: it counts the repetitions, so it knows when the halfway point
is reached. The user's previous sessions are represented by constants at
the top of `training_logic_node.py` (to be replaced by real session
history):

| Check | Constant | Result |
|-------|----------|--------|
| At 50% of the repetitions, the rise since the start is below 5 bpm | `SLOW_RISE_AT_HALF` | Break: *"Your body doesn't seem to be responding as expected…"* |
| At 50%, the rise is above 30 bpm | `FAST_RISE_AT_HALF` | Break: *"Your heart rate is rising faster than usual…"* |
| At any time, above 150 bpm | `MAX_EXPECTED_HEART_RATE` | Break: *"Your heart rate is higher than expected…"* |
| Otherwise (usually ~15 bpm) | `EXPECTED_RISE_AT_HALF` | *"You're doing great! Halfway there!…"* |

If no heart rate has arrived for 5 s, the halfway check is skipped.

### Feedback levels

`FEEDBACK_LEVEL` sets how much the robot talks. Each message has an
importance; **safety messages are never turned off**.

| Importance | Examples | Level 1 (detailed) | Level 2 (moderate) | Level 3 (minimal) |
|------------|----------|:---:|:---:|:---:|
| SAFETY | heart-rate breaks | ✔ | ✔ | ✔ |
| ESSENTIAL | start, stop, end, instructions | ✔ | ✔ | ✔ |
| MILESTONE | halfway | ✔ | ✔ | – |
| DETAIL | per-repetition encouragement | ✔ | – | – |

### Heart-rate simulator

In `heart_rate_simulator_node.py`, keep one scenario block active and
comment out the other two. The values count from when the node starts, so
start the exercise within about a minute of the launch.

| Scenario | Start | Rise | Max | Expected result |
|----------|------:|-----:|----:|-----------------|
| 1. Low, does not rise | 65 bpm | 0 bpm/s | 65 bpm | break at halfway |
| 2. Rises steadily (normal) | 70 bpm | 0.5 bpm/s | 130 bpm | "You're doing great!" |
| 3. Normal, then far too high | 70 bpm | 2 bpm/s | 180 bpm | break above 150 bpm |

## Run it

All machines use `ROS_DOMAIN_ID=102` (already set in the Pixi environment
on the Mac).

**1. Robot** (ssh to the Raspberry Pi), one terminal each:

``` bash
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

**2. Build** (Mac, once after each change). The package name keeps the
existing spelling `trainning`:

``` bash
cd ~/ros_ws && pixi shell
colcon build --merge-install --packages-select adaptive_training_assistant_msgs adaptive-trainning-assistance
source install/setup.zsh
```

**3. Node-RED dashboard** (optional), one terminal each:

``` bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
node-red
```

Open <http://127.0.0.1:1880/adaptive-training> (or
`http://<Mac IP>:1880/adaptive-training` from a phone). The flow is in
`src/node_red/training_dashboard_flows.json`; it subscribes through
rosbridge (`ws://localhost:9090`) to `/exercise/rep_event`,
`/training/status`, `/training/feedback` and `/heart_rate`. First-time
setup: `pixi add ros-jazzy-rosbridge-suite`, and the Node-RED palettes
`@flowfuse/node-red-dashboard` and `node-red-contrib-ros2-suite`.

**4. Start the assistant** (Mac):

``` bash
ros2 launch adaptive-trainning-assistance training_assistant.launch.py
```

The MediaPipe models are downloaded once, on the first run, to
`~/.cache/adaptive_training_assistant/`.

## Settings

Settings are constants at the top of each node file; rebuild after a
change.

| File | Main settings |
|------|---------------|
| `robot_base_node.py` | `TARGET_DISTANCE_M` (1.0), `SHOULDER_WIDTH_M` (distance calibration: `0.38 × real_distance / TARGET_DISTANCE_M`), `LINEAR_DIRECTION`, speeds and tolerances |
| `servo_tilt_node.py` | `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` / `INITIAL_SERVO_ANGLE` (45° / 100° / 70°, only here), `SEARCH_MIN_ANGLE` |
| `training_logic_node.py` | `TARGET_REPETITIONS`, heart-rate history constants, `FEEDBACK_LEVEL` |
| `heart_rate_simulator_node.py` | scenario (see above) |
| `gesture_node.py` | `MIN_SCORE`, `HOLD_SECONDS`, `FRAME_STRIDE` |
| `person_detection_node.py` | detection thresholds (`MIN_VISIBILITY`, `MIN_VISIBLE_LANDMARKS`, …) |
| `servo_driver_node.py` | `SERVO_IP`, `SERVO_PORT` of the ESP32 |

## Repository

```
Readme.md
docs/          presentations, demo videos, journal, deliverables, materials
models/        3D-printed parts (STL)
requirements.txt   Python packages (MediaPipe, OpenCV, NumPy)
src/
  adaptive_training_assistant/src/
    adaptive-trainning-assistance_msg/   custom messages
    adaptive_training_assistance/        the nine nodes and the launch file
  node_red/      Node-RED flow
  sketch_nov4b/  ESP32 servo sketch (receives the angle over UDP)
```

## Next steps

- Real heart-rate sensor publishing on `/heart_rate`.
- Speaker reading `/training/feedback` aloud.
- Real session history (repetitions, heart rate) instead of the
  predefined constants, shown on the Node-RED "Training History" page.
