# Adaptive Training Assistant

> **Work in Progress:** This project is under active development. Features, interfaces, and documentation may change as the prototype evolves.

A ROS 2 prototype that uses a TurtleBot USB camera and MediaPipe Pose Landmarker to detect arm-raise exercises, count repetitions, and provide basic training feedback.

## Repository Structure

```text
Group_102/
├── README.md
├── requirements.txt
├── docs/
│   ├── journal.csv
│   ├── materials-needed.md
│   └── presentations/
│       ├── project_proposal.pptx
│       └── midterm.pptx
└── src/
    ├── exercise_counter_node.py
    └── training_logic_node.py
```

## Current Implementation

### Exercise Detection

`exercise_counter_node.py` receives camera images from the TurtleBot and uses MediaPipe Pose Landmarker to detect arm raises.

A repetition is counted when both wrists are detected above the shoulders and subsequently return to the lowered position.

The node publishes repetition counts, timing information, and vision feedback through ROS 2 topics. It also supports an optional camera preview.

### Training Logic

`training_logic_node.py` listens for completed-repetition events and vision feedback from the exercise detection node.

It tracks progress toward a configurable repetition target and publishes training feedback and status information.

## ROS 2 Communication

The nodes communicate through ROS 2 topics to exchange camera images, exercise measurements, repetition events, and training feedback.

The main communication flow is:

1. The TurtleBot USB camera publishes image frames.
2. The Exercise Counter subscribes to the camera topic and processes the images using MediaPipe.
3. The Exercise Counter publishes repetition events and exercise metrics.
4. The Training Logic node receives these events and tracks the training progress.
5. The Training Logic node publishes feedback and training status.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- TurtleBot3 Burger with Raspberry Pi 4
- USB webcam connected to the TurtleBot
- `usb_cam` ROS 2 package installed on the TurtleBot
- Python dependencies listed in `requirements.txt`
- ROS 2 networking configured so the computer and TurtleBot can discover each other
- Pixi environment configured for the project

## Running the Robot

The following instructions describe the current workflow for running the prototype.

### 1. Connect to the TurtleBot

Connect to the Raspberry Pi onboard the TurtleBot through SSH:

```bash
ssh ubuntu@192.168.1.102
```

### 2. Launch the USB Camera

On the TurtleBot, start the USB camera node:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

This starts the USB camera and publishes image frames to the ROS 2 environment.

Keep this terminal running.

### 3. Set Up the Python Environment

In the project root directory, create and activate a Python virtual environment:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
```

Upgrade pip and install the project dependencies:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### 4. Run the Exercise Counter Node

From the project root, run:

```bash
pixi run python src/exercise_counter_node.py \
  --ros-args \
  -p show_preview:=true
```

The Exercise Counter subscribes to the TurtleBot camera stream, detects body landmarks using MediaPipe, and counts arm-raise repetitions.

The `show_preview` parameter enables the camera preview.

### 5. Run the Training Logic Node

Open a second terminal in the project root and run:

```bash
pixi run python src/training_logic_node.py \
  --ros-args \
  -p target_repetitions:=15
```

The `target_repetitions` parameter defines the number of repetitions required to complete a training set.

In this example, the training target is 15 repetitions.

## Documentation

- `docs/presentations/` contains the project proposal and midterm presentations.
- `docs/journal.csv` records the team's project work.
- `docs/materials-needed.md` lists the materials required for the prototype.

## Current Scope and Limitations

- The current detector recognizes both-arm raises; it is not a general exercise classifier.
- The repetition duration measures the detected raised phase, not the full movement cycle.
- Exercise detection depends on the visibility and quality of the camera stream.
- The prototype currently provides basic training feedback and repetition tracking.
