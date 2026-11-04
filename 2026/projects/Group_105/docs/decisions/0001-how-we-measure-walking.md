# 1. How we measure walking: the camera counts steps, the LiDAR decides when to trust them

- Status: accepted
- Date: 2026-11-04

## Context

Cadence (steps per minute) is the main measurement. It is used twice: during the walk, the robot compares it with the beat and corrects the patient; afterwards, the physiotherapist follows how it changes over sessions. The physiotherapist sets a target cadence, and how well the patient keeps it shows how they walk. Higher is not always better: in Parkinson's, people often take quick, tiny steps. Walking to a beat improves walking speed and step length in Parkinson's and after a stroke, according to systematic reviews, with moderate-quality evidence (see Sources). So the core technical problem is counting steps correctly.

The robot has two sensors that can see a walker, and each fails in different situations:

| | Camera with pose estimation | LiDAR (TurtleBot3) |
| --- | --- | --- |
| Sees | A skeleton of the walker, about 30 times per second, in pixels | One horizontal slice at shin height, 5 times per second, in metres: two leg blobs |
| Good at | Seeing every single step | Where the person is, and whether they are moving |
| Fails when | The person turns, stands and shifts weight, or is half in view: it counts steps that are not there | Counting steps: about 3 samples per step, and the legs hide each other when seen from the side |

## Decision

**The camera counts the steps.** `gait_monitor` runs MediaPipe Pose on each frame. A step is the moment the two feet are furthest apart along the path, which is when the front heel lands. The distance is divided by shin length (knee to ankle), so the distance to the camera does not matter. Steps must alternate left and right, at a plausible interval (about 0.3–1.5 s, tuned on recordings). Only the legs need to be in view.

**The LiDAR decides when the count can be trusted.** `lidar_person_tracker` reports where the person is along the path and how fast they move. `walk_session` counts camera steps only while the person is in the middle of the camera zone and walking along the path:

| Situation | Result |
| --- | --- |
| Walking through the middle | Steps counted |
| Turning at the end of the zone | Ignored |
| Standing, shifting weight | Ignored, and a stop if it lasts N seconds |
| Entering or leaving the view | Ignored |
| Person not found, or two people | Walk not measured, reported as such |

**What is measured and where it is used:**

| Measurement | During the walk | In the physiotherapist's summary |
| --- | --- | --- |
| Cadence | Compared with the beat → correction cue | Per walk, and first walk vs last walk (fatigue) |
| Stops (no steps and no movement) | "Take your time" cue | Count and time |
| Walking speed (LiDAR) | No | Per walk |

**Accuracy.** First goal: cadence within ±5% of a hand count. This is our own starting target, not a clinical standard, and is adjusted after the first recordings. We record the same walks and compare three results with the hand count: camera alone, LiDAR alone, and both together. For this comparison the LiDAR tracker also produces a rough step count.

## Consequences

- The two detectors do not use each other's data: each can be built and tested alone, by one team member each.
- The LiDAR removes the situations where the camera goes wrong, instead of averaging two inaccurate counts.
- A walk that cannot be measured is reported as not measured, never guessed, and never triggers a correction.
- The camera sits low, beside the path ([ADR 2](0002-robot-parks-beside-path.md)), and only needs the legs in view.
- Video of a person at home is sensitive: frames are processed live and never stored; only numbers leave the laptop.
- The current prototype in `walk_analysis_with_camera/` detects steps from ankle lift. It stays as a baseline in the accuracy comparison.

## Alternatives considered

- **Camera alone**: counts fake steps when the person turns, stands, or is half in view.
- **LiDAR alone** (the initial idea): too slow to see single steps, and the legs hide each other from the side.
- **Averaging or comparing camera and LiDAR step counts**: the LiDAR count is too rough to correct the camera.
- **Ankle lift** (current prototype): the ankle barely rises in a shuffling gait, so steps are missed.
- **Heel in front of the hips** (common gait-lab method): needs the hips in view, which a low camera may not see.
- **Wearable sensor on the ankle**: accurate, but the patient must put it on correctly, and it moves the project away from the robot.

## Sources

- Ghai S, Ghai I, Schmitz G, Effenberg AO. Effect of rhythmic auditory cueing on parkinsonian gait: a systematic review and meta-analysis. *Scientific Reports*, 2018. <https://pmc.ncbi.nlm.nih.gov/articles/PMC5764963/>
- Ghai S, Ghai I. Effects of (music-based) rhythmic auditory cueing training on gait and posture post-stroke: a systematic review & dose-response meta-analysis. *Scientific Reports*, 2019. <https://pmc.ncbi.nlm.nih.gov/articles/PMC6379377/>
