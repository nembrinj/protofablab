# Ball-E

## Abstract

- Leave for the end to simply summarize the whole thing into a few sentences

## Introduction

In [Le Train](https://www.opus89-collectif.com/en-creation.html) by Joséphine de Weck, hope appears only briefly; fragile, fleeting, and easily lost. Ball-E was created as a robotic stage companion that supports this atmosphere through light and repetition rather than narration or action. Positioned in the background, the robot continuously collects and launches illuminated balls, releasing short arcs of light that recall fireflies or shooting stars: momentary glimmers of hope in an otherwise dark space.

At the same time, the endless cycle of collecting and throwing evokes the myth of Sisyphus. The robot's task never resolves; meaning emerges through repetition and persistence rather than progress. Imperfect motion, visible effort, and mechanical delay are not errors but expressive qualities that align the system with the emotional undercurrent of the play.

Technically, Ball-E combines mechanical design, embedded electronics, and computer vision to autonomously detect illuminated objects, move toward them, and launch them back into the environment.

- Add contributions

This repository documents the design and development of the prototype, exploring how simple autonomous systems can contribute to mood, symbolism, and dramaturgy in theatrical contexts.

- List the content with links to the titles ( leave for the end)

## System Overview

Ball-E is an autonomous robotic sytem designed to repeatedly collect and shoot illumintaed balls in a theatrical dark environment. The system integrates mechanical, electronic and software components to create a continuous loop of perception, movement, and collection and propulsion.

So conceptually, the robot performs two core actions:

- Detect and move to illuminated balls
- Collect and launch the balls

### System Architecture

This leads us to the following system architecture, composed of three tightly coupled subsytems:

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
   - A [funnel](./Media/funnel_2.jpeg) guiding the balls to the DC-motors
4. **Connecting everything**: By a Python  script which automates
   - the movement
   - the detection
   - the threshold for activating the propulsion motors

### Hardware List

TODO: reference pictures!

| Amount | Parts                                                                        | Descriptions |
|--------|------------------------------------------------------------------------------|--------------|
| 1x     | [TurtleBot Preassembled](./Media/Images/turtle_bot_frontview.jpeg)           |              |
| 1x     | [3D-printend base_ramp](./Media/Images/base_ramp_2.jpeg)                     |              |
| 1x     | [3D-printend ramp_extension](./Media/Images/ramp_extension.jpeg)             |              |
| 1x     | [3D-printend ramp clamps](./Media/Images/ramp_clamps.jpeg)                   |              |
| 2x     | [3D-printend motor holders](./Media/Images/ramp_clamps.jpeg)                 |              |
| 2x     | [3D-printend gears without teeth](./Media/Images/gear.jpeg)                  |              |
| 1x     | [foam for coating the gears](./Media/Images/foam.jpeg)                       |              |
| 2x     | [5V DC-Motors](./Media/Images/dc-motors.jpeg)                                |              |
| 1x     | [Motor Relay](./Media/Images/relay.jpeg)                                     |              |
| 2x     | [Powerbanks](./Media/Images/powerbanks.jpeg)                                 |              |
| 2x     | [Usb power connectors](./Media/Images/usb_power_connectors.jpeg)             |              |
| 2x     | [Usb power connectors](./Media/Images/usb_power_connectors.jpeg)             |              |
| 1x     | [Bread Board](./Media/Images/bread_board.jpeg)                               |              |
| xx     | [Some male-male and male-female electric cables](./Media/Images/cables.jpeg) |              |
| 1x     | [Microcontroller](./Media/Images/microcontroller.jpeg)                       |              |
| 1x     | [Funnel](./Media/Images/funnel_2.jpeg)                                       |              |

### System Schemas

This is how the finished hardware architecture looks like:

<!-- <img src="path" style="max-width:600px; width:100%; height:auto;"> -->

This is how the motors are connected on the breadboard with the microcontroller, the relay and the powerbanks:

<img src="./Schemas/circuit.svg" style="max-width:600px; width:100%; height:auto;">

This is how the finished software architecture looks like: (todo: is this really software architecture??)

<img src="./Schemas/component_diagramm.png" style="max-width:600px; width:100%; height:auto;">

## Design and Fabrication of Hard- and Software

### Initial Prototype

For designing our propulsion mechanism, we did some research like anybody does by watching some YouTube videos. There we found a video where one was using two DC-Motors to which some type of wheels wrapped with some sort of rubber-band where attached, which thus where spun. Then, a ping pong ball was fed into the wheels and the spinning wheels managed to transfer some of their momentum to the ball to fling it away.

So, we tried to recreate this in our first iteration, using wood, from which we lasercutted two circles and put a hole inside of each to attach them to a DC-Motor each. We connected the DC-Motors to a battery each with a button in between to control whether they spin or not and thus, created the following 1. prototype showcased in this video:

<video src="./Media/Videos/propulsion_prototype_1.mp4" controls style="max-height:600px; height:100%; width:auto;"></video>

This works well, but our protoype wasn't very precise, the ball isn't thrown well consistently. And, although on camera the ball which we created by rolling paper into the shaped of a ball and taping it, looks like it flies quite a bit, in reality it doesn't fly that far, which was a bit of a concern for us. The ball in which we'll place the LED, we knew, would likely be heavier, since alone the coin battery for the LED is heavier. Thus, we need to also make some efficiency adaptations.

### Continous Iteration

Since our first prototype showed promise, we started with designing and testing key components. However, we also made some key decisions that motivated the following components. After discussion with our professors which gave their helpful tips and ideas, we adjusted our approach to:

- combine collection and propulsion into one. Basically, to directly feed the ball into the DC-Motors which instantly propulse it into the air. So, no storage of LED-Balls
- and decided t make the ball for the led 3d printed, with holes to let the leds light shine through

As a result the DC-motors should basically be placed at ground level. At least to implement this in the simplest way possible. So, we thought similar to our protoype we could then, use a ramp to guide the ball upwards.

So, we needed a 3D-printed ball, some type of ramp -> also 3d print and then, somewhat attach the dc-motors to this ramp. We'll now look at each of those components and how, we iterated them in development and highlight special considerations.

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

### Python Movement Script

To control to whole movement and operation of our Turtlebot3, we use a Python script similar to the `ca_controller.py` or `goal_controller.py` from the exercise [AN_02] or [AN_03], respectively. Hence, our Python script is executed on the same node as the ROS-master and enforces control over the robot by subscribing and publishing on ROS-topics. We collect and publish the following in- and outputs from and to the Turtelbot3:

| Direction | Description       | Purpose              | Type       | Object Type     | Topic                         |
|-----------|-------------------|----------------------|------------|-----------------|-------------------------------|
| Input     | Compressed images | LED/blob detection   | Subscriber | CompressedImage | /usb_cam/image_raw/compressed |
| Input     | LIDAR scan data   | Collision avoidance  | Subscriber | LaserScan       | /scan                         |
| Output    | Movement          | Robot locomotion     | Publisher  | Twist           | /cmd_vel                      |
| Output    | DC-Motors on/off  | Ball propulsion      | Publisher  | String          | /dc_motor_cmd                 |

We use the image to let our simple blob detector deduce where LEDs are located on the image, control the robots movement to guide it to a LED-ball while checking LIDAR scan info to avoid collisions to then safely collect the LED-ball by turning on the DC-Motors and stopping them shortly after.

To iniate any navigation towards a LED, one must first be detected. We assume that our robot has no further information about the location of the LEDs, which would otherwise be unreasonable. Detection is thus based on the robot searching the room with its camera for any LEDs. This demandes two different phases of operation:

- A scanning phase in which the robot actively searches for LEDs
- and a collection phase in which at least one LED has been found and thus, our Turtelbot3 moves to collect it.

#### Scanning Phase

In the scanning phase, we want to move our robot around as much as possible, to find any LEDs. This means, we need collision avoidance, to avoid letting our robot meet a wall or such. Here, we didn't want to overcomplicate things and build upon the `ca_controller.py` we could build ourselves in the respective exercise.

To cover as many viewpoints of our stagelike room, we made the robot scan in two interchanges mode: straight and circular.
In the circular mode, we decide upon turning narrowly and strongly into one direction: left or right. We decide upon which direction, by checking, in which the distance to any possible obstacles is the furthest; thus, offering the lesser chance of interupting the movement, to avoid a collision.

To switch between those two patterns, we simply use a timer for how long the robot executes each mode. For the demo video, we wanted the robot to turn around after having thrown a ball to quickly repeat its process. Thus, we set the circle duration to 16s and the straight scanning to 4s, with the circular search mode, being the one first executed.

During all this, we always check to avoid collisions. Since we move our robot only forward, we only check for obstacles more or less in front.

**Collision Avoidance:**
Collision avoidance is handled exclusively using the LIDAR `/scan` data. From each scan, we extract the minimum distance to obstacles in three sectors in front of the robot (left, front, and right). These distances are continuously updated and used across all motion states.

The robot enforces a minimum safety distance $d_\text{min}$. When an obstacle enters a buffer zone slightly above this distance, the forward velocity is scaled down proportionally to the remaining free space. If the closest obstacle is detected in front, the robot additionally applies a steering correction towards the side with more free space. If the closest obstacle is on either side, a small turn away from it is applied.

Due to this buffer-based velocity scaling, the commanded forward velocity becomes zero or even negative once an obstacle is closer than $d_\text{min}$. This causes the robot to automatically slow down and, if necessary, move backwards. This behavior is intentional, as it allows the robot to recover from situations where it gets too close to an obstacle to safely maneuver, effectively backing up to regain sufficient free space. It also allows for a funny interaction where one can move towards the robot to shoo away and it will effectively back up.

#### Centering and Collection

Given that we have detected one or more LED ball(s), our approach to collect it is based upon first centering it on the camera image to then move straight, while keeping it in sight and, if close enough move straight through it for a while to collect it. This is based on the assumption that the USB-camera is positioned approximately at the center of the ramp and the spinnig DC-motors.
From the many LED-balls to collect, we at each time select the one that in the camera's image is closest to the centering line we choose.

So, to effectively collect the LED-ball, we have an automata with three stages:

- _Centering_:
In practice, our camera is not exactly centered on the ramp. But this is also not necessary, it can be approximate, as we use the funnel to help guide the LED-ball into the spinning wheels of the DC-motors. Instead our camera is just placed on the side of the ramp, which still makes it approximately close to the ramps center. To adjust for our offset, instead of centering the LED around the exact vertical center line the x-axis of the cameras image, we offset this by a constant. We further, consider an allowed error, since the funnel allows for small inaccuracies. Thus, in the centering phase, we turn the robot while very slowly moving forward, such that the led ball is within this allow error centered on the vertical x-line.

Given this vertical line, we can check the LED's x-offset to it and let our robot slowly turn until the LED's blob on the image is located approximately within the vertical x-line + the allowed error.

- _Approaching_:
We can then fairly assume that since the LED ball is in clear view, their aren't any obstacles between us and the LED ball in front. This is based on assuming a typical squared theatre stage. Still, there could be wall or an obstacle behind the LED ball; so, we still need to check for obstacles in front. If there is one too close, then this implies that our large propulsion construct in front, won't make collecting the ball without hitting a wall. Otherwise, in the approaching phase, the robot, simply moves straight while correcting slightly its direction to keep the LED ball approximately centered.

- _Collecting_:
As soon as the LED is approximately close, we switch to collecting it. This we detect by checking when the LED-ball moves below a certain horizontal y-axis on the camera. This presumes the camera is angled to point slightly downwards, otherwise, this won't work.

To collect the ball, the DC-Motors need to be spinning. Letting them spin at all times will drain battery too qucily. Initially, we turned them on at the start of the collecting phase, via publishing on the ROS topic. But in some cases this was too late. Hence, we turn the DC-Motors on when transitioning to the approaching phase and thus, they should already be running.

To allow for the ball to roll through the funnel into the spinning motors, we decelerate our robot and for a fixed duration of 10s tell it to move straight to ensure collection, unless an obstacle blocks its path. This should allow for the ball to be caught by the funnels. We use this timer, as during collecting, it is likely to loose vision of the ball, even though hit has not yet been flung into th air. Further, the robot must move slowly, otherwise it just takes the ball on its journey, without actually shoot it out.

**Why not merge the two centering and approaching ?**

- It helps to simplify selection, when multiple balls are detected.
- Makes moving to it simpler, as clear vision to a centered ball indicates moving straight should demand any parcoury avoidance skills to move around any obstacles, as there shouldn't be any, assuming a clear theatre stages, with only rougher object props.
- Also, it gives this nice comical/cartoonish effect, of turning and then, moving to it, which could in a further work be even more emphasized.

**Difficulties in centering:**

During centering it can very much occur, that the blob detection for one or another frame can fail to detect the LED. Hence, we implemented a cooldown buffer, which counts the successive frames in which no blob was detected. If this exceeds a limit of 15 frames, then we can savely assume, that the led has been lost. Otherwise, we stop the robot from turning, and hope the blob detection mechanism can get detect it again.

The following UML-State diagram summarizes the interactions:

<img src="./Schemas/movement_algo.png" style="max-width:600px; width:100%; height:auto;">

### Start Up Software

We have three devices that run code:

| Device      | Usage                                     | Short             |
| ----------- | ----------------------------------------- | ----------------- |
| Our own PC  | Runs the ROS master and the Python script | [PC]              |
| The Raspi   | Controls the TurtleBot3                   | [turtlebot_raspi] |
| The Arduino | Controls the DC-Motors                    | [Arduino]         |

To start the whole interconnected system, the code should be run in the following order, with the target device denoted in square brackets. Use a new terminal for each command. For this to work, we assume that:

- All devices have been set up on the same Wi-Fi network
- The connection to the ROS master has been correctly configured
- The following components have been correctly set up:
  - **USB-camera**: Plugged into the Raspi on the TurtleBot3, with setup steps from exercise [AN_02, task 6, method 1] completed
  - **Python script**: Located correctly on your PC, with the CMake file properly modified and built
  - **Arduino**: The correct firmware has been loaded onto it

#### Startup Sequence

First, launch the ROS master:

```bash
roscore # [PC]
```

Next, set up rosserial Python for the Arduino to connect to ROS via TCP:

```bash
rosrun rosserial_python serial_node.py tcp 11411 # [PC]
```

The Arduino will connect automatically when powered by the battery.

Now, set up the USB camera on the TurtleBot3. Turn the TurtleBot3 on and SSH into it from your PC:

```bash
ssh ubuntu@ubuntu.local # [PC] (This works if you have the avahi-daemon installed to provide mDNS)

# Then, enter the password

roslaunch turtlebot3_bringup turtlebot3_robot_usbcam.launch # [turtlebot_raspi]
```

Finally, run the Python script that contains all the control logic:

```bash
rosrun turtlebot3_profab blob_centeration.py # [PC]
```

#### Testing

Here are some tests you can run to ensure the required components are operational:

**Test Arduino connection:**
Check if the Arduino is listening to the `/dc_motor_cmd` topic:

```bash
rostopic pub /dc_motor_cmd std_msgs/String "data: 'on'" # [PC]
```

or

```bash
rostopic pub /dc_motor_cmd std_msgs/String "data: 'off'" # [PC]
```

**Test USB camera:**
Check if the USB camera is sending images by opening RViz and inspecting the Image section:

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch # [PC]
```

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
