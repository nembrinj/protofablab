# Robot-Guided Home Rehabilitation

Monitoring and therapist feedback between physiotherapy sessions, using a TurtleBot3 Burger.

Group 105 project for the Prototyping and Fabrication course.

## Team

- Leonid Pomortsev: LiDAR, navigation, session logic, and dashboard
- Matin Mahmoudzadeh: camera, camera mast, step detection, and audio cues

## Presentations

| Presentation | Slides                                               |
| ------------ | ---------------------------------------------------- |
| Initial      | [presentation_initial.pdf](presentation_initial.pdf) |
| Mid-term     | [presentation_midterm.pdf](presentation_midterm.pdf) |
| Final        | To be added                                          |

## Problem

Gait rehabilitation depends on walking exercises that patients do at home, alone. Patients lose the rhythm, get tired, and stop early. The physiotherapist sees none of it, only what the patient remembers at the next appointment.

## Solution

A robot that supervises the walking exercise between appointments. It:

- **guides**: positions itself beside the walking path and sets the rhythm of the walk with an audible beat;
- **measures**: cadence (steps per minute). A camera counts the steps; the LiDAR tells when the count can be trusted, and measures walking speed;
- **cues**: speaks a correction when the patient drifts off the rhythm or stops;
- **reports**: publishes a session summary to a dashboard the physiotherapist can act on.

## Proof of concept

The proof of concept shows that a robot can count a walker's steps reliably by itself and correct them in the moment.

The TurtleBot3 drives itself to a spot beside a walking path and stays still while a person walks back and forth past it. During the session it plays a beat at the target cadence, measures each walk past it, and speaks a correction when the person stops or drifts off the beat. After each walk it sends a summary to the physiotherapist's dashboard.

- One healthy volunteer, one walking exercise, on a flat walking path of about 10 m.
- Camera zone of about 3–4 m, set by the camera's field of view.
- Stretch goals: step-time symmetry, step length, and rotating in place to keep the walker in view.

### Evaluation

- **Detection latency**: how quickly the robot notices that the person has stopped.
- **Measurement accuracy**: measured cadence compared with a manual step count. First goal: within ±5%, adjusted after the first recordings. The same walks are compared three ways: camera alone, LiDAR alone, and both together.
- **Cue reliability**: whether the right cue fires, across repeated walks.

### Limitations

- Not a clinical device, and not tested with patients.
- Provides no physical support.
- Complements physiotherapy sessions and does not replace them.

## Architecture

![System architecture](docs/img/architecture.svg)

A webcam on the robot streams video to a laptop, which detects steps with pose estimation. The robot's LiDAR tracks where the walker is and how fast they walk. The laptop runs the session logic and sends the beat and spoken cues back to a speaker on the robot. Session summaries go over MQTT to a Grafana dashboard. See [docs/architecture.md](docs/architecture.md) for the components and interfaces, and [docs/decisions/](docs/decisions/) for the design decisions.

## Hardware

- TurtleBot3 Burger
- Additional equipment: see [required-equipment.md](required-equipment.md)

## Demo

To be added.

## Repository structure

```text
.
├── README.md                    # This page
├── required-equipment.md        # Hardware we use or borrow from the lab
├── presentation_initial.pdf     # Initial idea presentation
├── presentation_midterm.pdf     # Mid-term presentation
├── docs/
│   ├── architecture.md          # System architecture and interfaces
│   ├── decisions/               # Design decision records
│   └── img/                     # Diagrams
└── walk_analysis_with_camera/   # Gait and cadence analysis from video
```
