# 4. The beat and spoken cues play on the robot

- Status: accepted
- Date: 2026-11-04

## Context

The patient must hear a steady beat at the prescribed cadence and short spoken corrections. A beat sent as individual messages over Wi-Fi would arrive with jitter, which defeats rhythmic cueing.

## Decision

A USB speaker on the robot plays the audio. A node on the Raspberry Pi generates the beat locally from a target tempo, and speaks short cue texts with offline text-to-speech. The laptop sends only the tempo and the cue texts.

## Consequences

- Beat timing does not depend on the network.
- The sound comes from the robot, which makes it the clear point of contact for the patient.
- The speaker draws power from the robot; check it does not cause brownouts on the LiDAR and camera.
- Fallback for the demo: the same node can run on the laptop with its own speaker.

## Alternatives considered

- **Laptop speaker**: simplest, but the laptop is not where the patient walks.
- **Beat as one message per beat**: network jitter makes the rhythm uneven.
