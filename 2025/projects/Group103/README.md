# Ball-E

## Abstract

## Introduction

In [Le Train](https://www.opus89-collectif.com/en-creation.html) by Joséphine de Weck, hope appears only briefly—fragile, fleeting, and easily lost. Ball-E was created as a robotic stage companion that supports this atmosphere through light and repetition rather than narration or action. Positioned in the background, the robot continuously collects and launches illuminated balls, releasing short arcs of light that recall fireflies or shooting stars: momentary glimmers of hope in an otherwise dark space.

At the same time, the endless cycle of collecting and throwing evokes the myth of Sisyphus. The robot’s task never resolves; meaning emerges through repetition and persistence rather than progress. Imperfect motion, visible effort, and mechanical delay are not errors but expressive qualities that align the system with the emotional undercurrent of the play.

Technically, Ball-E combines mechanical design, embedded electronics, and computer vision to autonomously detect illuminated objects, move toward them, and launch them back into the environment. This repository documents the design and development of the prototype, exploring how simple autonomous systems can contribute to mood, symbolism, and dramaturgy in theatrical contexts.

## System Overview

Ball-E is an autonomous robotic sytem designed to repeatedly collect and project illumintaed balls in a theatrical dark environment. The system integrates mechanical, electronic and software components oto create a continuous loop of perception, movement and collection and propulsion.
So conceptually, the robot performs two core actions:

- Detect and move to illuminated balls
- Collect and launch the balls

### System Architecture

Which leads us to the following system architecture, composed of three tightly coupled subsytems:

1. **Perception**: Lead by..
   - A full-HD Usb camera mounted on the robot used for "seeing" the balls and send to the maincomputing station
   - A script for executing real-time blob detection for lights executed on the maincomputing station
2. **Movement and Control**: Making use of..
   - Prebuild [Turtlebot](./Media/turtle_bot_sideview.jpeg) with a lidar for object detection and navigating the landscape
   - Raspberry Pi 4 for communicating with the maincomputing station by means of pub/sub
3. **Collection and Launch Mechanism**: With making use of or printing..
   - A Breadboard connecting 2x 5V DC-motors for the propulsion, connected to
   - A [ramp](./Media/base_ramp_2.jpeg) which is connected to the robot and
   - An extension of the ramp which gives the optimal angle for the propulsion in regards to the chosen motors
   - A [funnel](./Media/funnel_2.jpeg) leading the balls to the DC-motors
4. **Connecting everything**: By a python script which automates
   - the movement
   - the detection
   - the threshold for activating the propulsion motors

### Hardware List

TODO: reference pictures!

| Amount | Parts                                                                 | Descriptions |
|--------|-----------------------------------------------------------------------|--------------|
| 1x     | [TurtleBot Preassembled](./Media/turtle_bot_frontview.jpeg)           |              |
| 1x     | [3D-printend base_ramp](./Media/base_ramp_2.jpeg)                     |              |
| 1x     | [3D-printend ramp_extension](./Media/ramp_extension.jpeg)             |              |
| 1x     | [3D-printend ramp clamps](./Media/ramp_clamps.jpeg)                   |              |
| 2x     | [3D-printend motor holders](./Media/ramp_clamps.jpeg)                 |              |
| 2x     | [3D-printend gears without teeth](./Media/gear.jpeg)                  |              |
| 1x     | [foam for coating the gears](./Media/foam.jpeg)                       |              |
| 2x     | [5V DC-Motors](./Media/dc-motors.jpeg)                                |              |
| 1x     | [Motor Relay](./Media/relay.jpeg)                                     |              |
| 2x     | [Powerbanks](./Media/powerbanks.jpeg)                                 |              |
| 2x     | [Usb power connectors](./Media/usb_power_connectors.jpeg)             |              |
| 2x     | [Usb power connectors](./Media/usb_power_connectors.jpeg)             |              |
| 1x     | [Bread Board](./Media/bread_board.jpeg)                               |              |
| xx     | [Some male-male and male-female electric cables](./Media/cables.jpeg) |              |
| 1x     | [Microcontroller](./Media/microcontroller.jpeg)                       |              |
| 1x     | [Funnel](./Media/funnel_2.jpeg)                                       |              |

### System Schemas

This is how the finished hardware architecture looks like:

This is how the motors are connected on the breadboard with the microcontroller, the relay and the powerbanks:

This is how the finished software architecture looks like:

## Design and Fabrication of Hard- and Software

- Present propulsion mechanism by going through the devlopment stage part by part:

  - First two tests (videos?)
  - LED-Ball
    - Use following schema to describe iterations
      - _Intial idea_: ...
      - _Iteration 1_:
        - **Considered Implementation:**
        - **What Worked:**
        - **What Didn't Work:**
        - **Next Improvements:**

  - Ramp
  - DC-Holders
  - Another test (ramp exit angle to flat)
  - Ramp-Extension
  - Funnel
  - Turtelbot3 connector

  - Circuit Arduino

    - 2 Schemas:
      - Visual + camera pic
      - logical, should be possible to export from same program as done viusal with
    - Parallel
    - Relay
    - 2x battery-pack

### Autonomous Movement to LED-Ball

To enable our robot to autonomously move to one of many LED-Balls such that our simultaneous collection and propulsion mechanism can grab a hold of it, we require two mechanisms. First, our robot must be able to locate the LED-balls and secondly, our robot needs to be able to navigate to one of these LED-balls and move such that the ball nicely rolls through the funnel into the two spinning DC-Motors.

**Initial Idea**
Our first intuition was to use a USB-Camera, and then locate the shining 3D printed balls by finding the brightest pixel(s) in the camera's image. Then, we could try to center one of those "pixels"/detected LED-Ball within the camera's view, through rotating the robot (This would require the camera to be placed approximately at the center of the ramp.). When the detected LED-Ball then is centered, our Turtlebot could simply move straight and the let the ball roll into the spinning DC-Motors.

Given a hint by our professors, we decided to go for a more robust detection approach: instead of trying to interpret pixel data in an overly simplistic way, we used opencv's blob detection.

#### Detecting the LED-Balls using Blob Detection

##### Set up camera

First, we installed a USB-camera by following the tutorial in [AN_02](...), plugging it directly into the Raspi on the Turtlebot3.
However, we could only receive the camera's raw image on the ROS master, which had a laggy framerate. It turned out that we were missed installing a certain package.

While testing the blob detection under different lighting, we noticed that the camera's auto-exposure and white balance interfered with LED detection. When we tested dim lighting conditions similar to the FabLab's, the camera's automatic configurations overcompensated and lit the image as if we were at a sunny beach. We solved this by disabling auto-exposure and white balance and setting a fixed exposure via a startup service [camera-fix.service](./Code/camera-fix.service), such that the Turtlebot would automatically configure the camera at boot.

#### Blob Detection

Blob detection was a crucial part of our method. Because in order to develop the navigational part, we need to detect some LED-Balls. Which is why we leveraged AI there to make quick progress when creating a simple blob detector for our LEDs.

To support our devolopment, we first recorded some [example data](./Code/example_data/), allowing us to test and tune the detector without needing the full robot setup.

Our first implementation attempt revealed an unexpected challenge. The initial version of our 3D-printed ball had holes to maximize LED light emission. Combined with a somewhat reflective floor surface, this created blob-shaped reflections  [placeholder for such picture](..) that confused OpenCV's `SimpleBlobDetector` module, leading us to develop a custom approach that works as follows:

1. **Peak Detection**: Identify the brightest local maxima in the image, assuming LED balls are the brightest light sources on stage
2. **Circularity Validation**: Approximate a radius around each peak and measure the light intensity fall-off pattern. A true LED ball should have relatively uniform radial intensity decay
3. **Size Filtering**: Use expected blob size to filter out tiny reflections from the holes in the ball
4. **Color Filtering**: Since we use green LEDs, verify that detected blobs match the expected green color spectrum

**Improvement Through Hardware Iteration:**
This performed well enough. So, after implementing this algorithm and testing it a bit, we followed our professors' suggestion to eliminate the holes in the LED ball and use transparent filament instead. This hardware change significantly improved detection reliability by eliminating the problematic reflections, validating our multi-stage filtering approach.

**Color Calibration Issue:**
We had already begun developing the blob detection when we corrected the camera's auto-exposure and white balance. So, in the demonstration, what occured was that some light sources were detected although they weren't close to our green. This was actually, because our initial example pictures had more of a cyanish green color and we missed out to consider this change after fixing up the camera settings.

### Python Movememnt Script

...

### Start up software

...

## Result

- Show demo video/link demo video
- Discussion of limitations and result in general?

## Discussion

- Change of initial idea
- Interpretation of results
- Lessons Learned

## Conclusion

- Summarize our project

## Future Works/Perfect Project

- What improvements would we make?
- How would the perfect implementation with a team of xx people and many resources be like.
