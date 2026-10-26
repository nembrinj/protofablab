# Robot-Guided Home Rehabilitation

Monitoring and therapist feedback between physiotherapy sessions, using a TurtleBot3 Burger.

Group 105 project for the Prototyping and Fabrication course.

## Team

- Leonid Pomortsev
- Matin Mahmoudzadeh

## Presentations

| Presentation | Slides                                               |
| ------------ | ---------------------------------------------------- |
| Initial      | [presentation_initial.pdf](presentation_initial.pdf) |
| Mid-term     | To be added                                          |
| Final        | To be added                                          |

## Problem

Gait rehabilitation depends on walking exercises that patients do at home, alone. Patients lose their rhythm, drift into asymmetry as they tire, and cut sessions short. The physiotherapist sees none of it, only what the patient remembers at the next appointment.

## Solution

A robot that supervises the walking exercise between appointments. It:

- **guides**: sets the route and the rhythm of the walk with an audible beat;
- **measures**: estimates cadence, speed, and symmetry from leg movement, using a camera and pose estimation;
- **cues**: speaks a correction when the patient drifts off the rhythm or stops;
- **reports**: publishes a session summary to a dashboard the physiotherapist can act on.

## Proof of concept

- One healthy volunteer, one walking exercise, on a flat corridor.
- Capture zone of about 7 m.

### Evaluation

- **Detection latency**: how quickly the robot notices that the person has stopped.
- **Measurement accuracy**: measured cadence compared with a manual step count.
- **Cue reliability**: whether the right cue fires, across repeated walks.

### Limitations

- Not a clinical device, and not tested with patients.
- Provides no physical support.
- Complements physiotherapy sessions and does not replace them.

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
└── walk_analysis_with_camera/   # Gait and cadence analysis from video
```
