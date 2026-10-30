# Adaptive Training Assistant

> **Work in Progress:** This project is under active development. Features, interfaces, and documentation may change as the prototype evolves.

A ROS 2 prototype that uses a TurtleBot camera and MediaPipe Pose Landmarker to detect arm-raise exercises, count repetitions, and provide basic training feedback.

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
    └── exercise_detection/
        ├── exercise_counter_node.py
        ├── training_logic_node.py
        └── training_assistant.launch.py
```

## Current Implementation

### Exercise Detection

`exercise_counter_node.py` receives camera images from the TurtleBot and uses MediaPipe Pose Landmarker to detect arm raises. A repetition is counted when both wrists are detected above the shoulders and then return to the lowered position.

The node publishes the repetition count, timing information, and vision feedback through ROS 2 topics.

### Training Logic

`training_logic_node.py` listens for completed-repetition events and the detector's vision feedback. It tracks progress toward a configurable repetition target and publishes basic training feedback and status information.

### Launch File

`training_assistant.launch.py` defines a launch configuration for starting the Exercise Counter and Training Logic nodes together in a ROS 2 package environment.

## ROS 2 Communication

The current prototype uses ROS 2 topics to exchange camera images, exercise measurements, and training feedback between nodes. The topic names, message types, and final communication interfaces are still under development and have not yet been defined.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- A TurtleBot camera publishing a `sensor_msgs/msg/Image` topic visible to the ROS 2 environment
- The Python dependencies listed in `requirements.txt`
- ROS 2 networking configured so the computer and TurtleBot can discover each other

Create and activate a virtual environment that can access the ROS 2 Python packages, then install the project dependencies:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

The dependencies currently listed in `requirements.txt` are:

- `mediapipe==1.1.1`
- `numpy==1.26.4`
- `opencv-contrib-python==4.11.0.86`

## Running the Nodes

Make sure the TurtleBot camera is running and its image stream is available in the ROS 2 environment. Use the same `ROS_DOMAIN_ID` on the computer and the TurtleBot. The following examples use `102`; change it if your setup uses a different value.

Run Exercise Detection in one terminal:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=102
python src/exercise_detection/exercise_counter_node.py
```

Run Training Logic in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=102
python src/exercise_detection/training_logic_node.py
```

The first start downloads the MediaPipe pose model automatically and stores it in the user's cache directory. Internet access is needed for this initial download unless a model file is supplied through the `model_path` ROS parameter.

The launch file is included for use after the ROS 2 package has been configured, built, and its `exercise_counter` and `training_logic` executables have been registered. The repository files shown above do not, by themselves, define that package installation setup.

## Documentation

- `docs/presentations/` contains the project proposal and midterm presentations.
- `docs/journal.csv` records the team's project work.
- `docs/materials-needed.md` lists the materials required for the prototype.

## Current Scope and Limitations

- The current detector recognizes both-arm raises; it is not a general exercise classifier.
- The repetition duration measures the detected raised phase, not the full movement cycle.