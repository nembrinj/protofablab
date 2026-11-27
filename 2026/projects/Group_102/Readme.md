# Adaptive Training Assistant

A TurtleBot3 that acts as a personal exercise coach. It finds the user,
positions itself in front of them, counts their repetitions, watches their
heart rate and gives short spoken-style feedback that adapts to how the
user responds.

## What it does

1. **Finds the user:** sweeps the camera up and down, turns ~30°, and
   repeats in a circle until it sees a person's upper body.
2. **Positions itself:** centres the person and drives to about 1.3 m, so
   the whole upper body is visible.
3. **Starts** when the user shows a **thumbs up 👍** (held 1 s).
4. **Coaches:** counts bilateral arm raises (both arms above the shoulders
   and back down), the time per repetition and the pace.
5. **Adapts to the heart rate:** halfway through, it compares the rise with
   the user's previous sessions. Too little ends the exercise with a break;
   too fast, or higher than ever, only warns the user to be careful and to
   show 👎 if they need to stop.
6. **Stops** on a **thumbs down 👎**, after 10 repetitions, or on a break,
   and announces the result, e.g. *"You did 8 repetitions in 1 minute 20
   seconds, at 6 repetitions per minute."*

Feedback is published as sentences on `/training/feedback`, ready for a
speaker. State, repetitions, pace, heart rate and feedback are shown on a
**Node-RED** web dashboard (also on a phone); a small OpenCV window shows
the annotated camera image.

## Hardware

| Part | Role |
|------|------|
| TurtleBot3 Burger (Raspberry Pi) | Mobile base, odometry |
| USB camera at the **back** of the robot | Person, pose and gesture detection |
| SG90 servo + ESP32 (UDP over Wi-Fi) | Tilts the camera up/down |
| 3D-printed parts (`models/`) | Camera holder, camera pivot support, servo mount |
| Polar H10 heart-rate chest strap (Bluetooth LE) | Heart rate, read on the Mac by `heart_rate` |
| Speaker *(planned)* | Will read `/training/feedback` aloud |

All processing (MediaPipe, ROS nodes) runs on a Mac; the robot only
streams the camera and drives the wheels.

## Session flow

```
SEARCHING ──person seen──> CENTERING ──centred + 1.3 m──> READY
    ▲                                                     │ 👍
    └──────── person lost > 2 s ─────────                 ▼
                                    FINISHED <──👎 / 10 reps / break── IN_PROGRESS
                                    (5 s, then back to READY)
```

In `READY`, `IN_PROGRESS` and `FINISHED` the robot and the camera do not
move. `READY` is sticky: it stays on while the person is visible and only
ends when they are lost for 2 s.

## Architecture

Nine ROS 2 Jazzy nodes in one package, plus a message package. Every line
below is a ROS topic, except the UDP link to the ESP32.

### Main data flow

``` mermaid
flowchart TB
    subgraph SENSORS["Robot sensors"]
        direction LR
        ODOM["Odometry"]
        CAM["USB camera"]
        HR["heart_rate<br/>(Polar H10)"]
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

`training_logic` shares the current state on `/training/status`; `gesture`
ignores gestures while the exercise counter reports the arms raised.

``` mermaid
flowchart LR
    LOGIC["training_logic"]
    STATUS(["/training/status"])
    GEST["gesture"]
    BASE["robot_base"]
    TILT["servo_tilt"]
    EX["exercise_counter"]
    PD["person_detection"]
    HR["heart_rate<br/>(Polar H10)"]
    CAMV["camera_view<br/>(OpenCV window)"]
    RB["rosbridge"]
    NR["Node-RED dashboard"]

    LOGIC -->|"current state"| STATUS
    STATUS --> GEST
    STATUS --> BASE
    STATUS --> TILT
    EX -->|/exercise/feedback| GEST

    PD -->|/person/preview/compressed| CAMV

    STATUS --> RB
    LOGIC -->|/training/feedback| RB
    EX -->|/exercise/rep_event| RB
    HR -->|/heart_rate| RB
    RB -->|"websocket :9090"| NR
```

### Nodes

| Node | What it does |
|------|--------------|
| `person_detection` | MediaPipe Pose, once per frame. Publishes the shoulder midpoint and width (`/person/position`), all 33 landmarks (`/person/landmarks`) and an annotated preview. A person needs at least 3 clearly visible body points; the *upper body* needs both shoulders and the nose, so chairs are not taken for people. |
| `robot_base` | Drives the wheels: circular search, turning to centre the person, driving to the target distance. Publishes `/person/ready` and `/person/distance`. |
| `servo_tilt` | Decides the camera angle (`/servo/angle`): keeps the shoulders at the image centre (`/servo/aligned`) and sweeps 70°–100° during a search. |
| `servo_driver` | Forwards the angle to the ESP32 over UDP; the ESP32 only moves the servo there. |
| `gesture` | MediaPipe Gesture Recognizer, only in `READY` and `IN_PROGRESS`: 👍 = `start`, 👎 = `stop`, each held 1 s. |
| `exercise_counter` | Counts arm raises while enabled. Publishes one JSON `/exercise/rep_event` per repetition: `repetitions`, `repetition_duration_seconds`, `pace_reps_per_minute`, `exercise_duration_seconds`. |
| `training_logic` | Session flow, heart-rate checks and feedback. Publishes `/training/status`, `/training/feedback`, `/training/feedback_level`. |
| `heart_rate` | Finds the Polar strap over Bluetooth (`bleak`), publishes every value on `/heart_rate` (`std_msgs/Float32`, bpm), and keeps searching if it disconnects. |
| `camera_view` | OpenCV window with the annotated camera image (`q` closes it). |

**Custom messages** (`adaptive_training_assistant_msgs`): `PersonPosition`
(person / upper body detected, shoulder midpoint `tracking_x/y` in 0..1,
`shoulder_width` as a fraction of the image) and `PoseLandmarks` (the 33
MediaPipe points, so Pose runs only once).

## Technical points

**Distance without a depth camera.** `robot_base` estimates it from the
shoulder width in the image:
`distance = SHOULDER_WIDTH_M / (2 · tan(HFOV/2) · shoulder_width)`
(0.38 m average shoulders, 60° field of view). Readings outside 0.4–4 m
are ignored, and the robot only drives once the person is centred.

**Back-facing camera.** Driving away from the person is a forward command
(`LINEAR_DIRECTION = -1`).

**Stable decisions.** The tracked point is smoothed; the servo uses a
deadband with hysteresis and must be centred for 0.5 s; `/person/ready`
turns on after 0.5 s and stays on; a gesture must be held 1 s, and misses
under 0.4 s do not restart it.

**One place for the servo limits.** `servo_tilt` sends an angle, not a
speed, so the limits and the start angle exist only there, and if the
messages stop the servo simply stays where it is.

**Safe stopping.** The TurtleBot keeps its last speed until a new command
arrives, so `robot_base` handles Ctrl+C itself and sends a stop before ROS
shuts down. Without odometry for 0.5 s the robot stops.

**Network.** The nodes all run on the Mac, so only the robot's topics
cross the Wi-Fi: camera images and `/odom` to the Mac, `/cmd_vel` back,
plus the UDP servo angles. On a weak phone hotspot images can arrive with
gaps of ~0.6 s (`ros2 topic hz`); the timeouts and the sticky `READY` are
chosen so such gaps do not change the state.

### Heart rate

The sensor only sends bpm values; `training_logic` counts
the repetitions and makes the decisions. The user's previous sessions are
constants in `training_logic_node.py` (to be replaced by real history):

| Check | Constant | Result |
|-------|----------|--------|
| At 50% of the repetitions, rise below 5 bpm | `SLOW_RISE_AT_HALF` | Break: *"Your body doesn't seem to be responding as expected…"* |
| At 50%, rise above 30 bpm | `FAST_RISE_AT_HALF` | Warning, exercise goes on: *"Your heart rate is rising faster than usual. Please be careful. If you feel you need to stop, show a thumbs down."* |
| Any time, above 150 bpm (once per exercise) | `MAX_EXPECTED_HEART_RATE` | Warning, exercise goes on: *"Your heart rate is higher than ever before. Please be careful…"* |
| Otherwise (usually ~15 bpm) | `EXPECTED_RISE_AT_HALF` | *"You're doing great! Halfway there!…"* |

The halfway check is skipped if no heart rate arrived in the last 5 s.
When the exercise ends, the reason and the result are sent as one message.

### Feedback levels

`FEEDBACK_LEVEL` sets how much the robot talks; **safety messages are
never turned off**.

| Importance | Examples | Level 1 (detailed) | Level 2 (moderate) | Level 3 (minimal) |
|------------|----------|:---:|:---:|:---:|
| SAFETY | heart-rate breaks and warnings | ✔ | ✔ | ✔ |
| ESSENTIAL | start, stop, end, instructions | ✔ | ✔ | ✔ |
| MILESTONE | halfway | ✔ | ✔ | – |
| DETAIL | per-repetition encouragement | ✔ | – | – |

## Run it

All machines use `ROS_DOMAIN_ID=102` (already set in the Pixi environment
on the Mac). One terminal per command.

**1. Robot** (ssh to the Raspberry Pi):

``` bash
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

**2. Build** (Mac, after each change; the package name keeps the spelling
`trainning`):

``` bash
cd ~/ros_ws && pixi shell
colcon build --merge-install --packages-select adaptive_training_assistant_msgs adaptive-trainning-assistance
source install/setup.zsh
```

**3. Node-RED dashboard:**

``` bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
node-red
```

Open <http://127.0.0.1:1880/adaptive-training> (from a phone:
`http://<Mac IP>:1880/adaptive-training`). The flow
(`src/node_red/training_dashboard_flows.json`) subscribes through
rosbridge (`ws://localhost:9090`) to `/exercise/rep_event`,
`/training/status`, `/training/feedback` and `/heart_rate`. First time:
`pixi add ros-jazzy-rosbridge-suite`, and the Node-RED palettes
`@flowfuse/node-red-dashboard` and `node-red-contrib-ros2-suite`.

**4. Start the assistant** (wear the Polar strap first):

``` bash
ros2 launch adaptive-trainning-assistance training_assistant.launch.py
```

The MediaPipe models are downloaded on the first run to
`~/.cache/adaptive_training_assistant/`. The heart-rate node needs the
`bleak` package (once: `pixi add --pypi bleak`), and on macOS the terminal
needs Bluetooth permission. Wear the strap before the launch; it connects
after ~10 s ("Connected to Polar H10 …").

## Settings

Constants at the top of each node file; rebuild after a change.

| File | Main settings |
|------|---------------|
| `robot_base_node.py` | `TARGET_DISTANCE_M` (1.3 m) ± `DISTANCE_TOLERANCE_M` (0.15 m); `SHOULDER_WIDTH_M` (calibration: `0.38 × real_distance / TARGET_DISTANCE_M`); `LINEAR_DIRECTION`; `SEARCH_STEP` (0.5 rad ≈ 30°); speeds |
| `servo_tilt_node.py` | `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` / `INITIAL_SERVO_ANGLE` (45° / 100° / 70°); `SEARCH_MIN_ANGLE` |
| `training_logic_node.py` | `TARGET_REPETITIONS`, heart-rate history constants, `FEEDBACK_LEVEL` |
| `heart_rate_node.py` | `DEVICE_NAME` (part of the strap's Bluetooth name, `Polar`) |
| `gesture_node.py` | `MIN_SCORE`, `HOLD_SECONDS`, `FRAME_STRIDE` |
| `person_detection_node.py` | `MIN_VISIBILITY`, `MIN_VISIBLE_LANDMARKS`, … |
| `servo_driver_node.py` | `SERVO_IP`, `SERVO_PORT` of the ESP32 |

## Repository

```
Readme.md
docs/              presentations, demo videos, journal, deliverables, materials
models/            3D-printed parts (STL)
requirements.txt   Python packages (MediaPipe, OpenCV, NumPy, bleak)
src/
  adaptive_training_assistant/src/
    adaptive-trainning-assistance_msg/   custom messages
    adaptive_training_assistance/        the nodes and the launch file
  node_red/        Node-RED flow
  sketch_nov4b/    ESP32 servo sketch (receives the angle over UDP)
```

## Next steps

- Speaker reading `/training/feedback` aloud.
- Real session history instead of the predefined constants, shown on the
  Node-RED "Training History" page.
