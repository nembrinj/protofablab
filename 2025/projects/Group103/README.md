# Ball-E

## Abstract

- Leave for the end to simply summarize the whole thing into a few sentences

## Introduction

In [Le Train](https://www.opus89-collectif.com/en-creation.html) by Joséphine de Weck, hope appears only briefly; fragile, fleeting, and easily lost. Ball-E was created as a robotic stage companion that supports this atmosphere through light and repetition rather than narration or action. Positioned in the background, the robot continuously collects and launches illuminated balls, releasing spectacle of short arcs of light that recall fireflies or shooting stars: momentary glimmers of hope in an otherwise dark space.

At the same time, the endless cycle of collecting and throwing evokes the myth of Sisyphus. The robot's task never resolves; meaning emerges through repetition and persistence rather than progress. Imperfect motion, visible effort, and mechanical delay are not errors but expressive qualities that align the system with the emotional undercurrent of the play.

Technically, Ball-E combines mechanical design, embedded electronics, and computer vision to autonomously detect illuminated objects, move toward them, and launch them back into the environment.

- Add contributions

This repository documents the design and development of the prototype, exploring how simple autonomous systems can contribute to mood, symbolism, and dramaturgy in theatrical contexts.

- List the content with links to the titles ( leave for the end)

## System Overview

Ball-E is an autonomous robotic sytem designed to repeatedly collect and shoot illumintaed balls in a theatrical dark environment. The system integrates mechanical, electronic and software components to create a continuous loop of perception, movement, and collection and propulsion.

Conceptually, the robot performs two core actions:

- Detect and move to illuminated LED-balls
- Collect and launch the LED-balls

### System Architecture

This leads us to the following system architecture, composed of three tightly coupled subsytems:

1. **Perception**: Lead by..
   - A full-HD USB camera mounted on the robot used for "seeing" the balls, sending its image to the maincomputing station
   - A script for executing real-time blob detection for lights executed on the maincomputing station
2. **Movement and Control**: Making use of..
   - Prebuild [Turtlebot](./Media/turtle_bot_sideview.jpeg) with LIDAR for object detection and navigating the landscape
   - Raspberry Pi 4 for communicating with the maincomputing station by means of publishing and subscribing to ROS topics
3. **Collection and Launch Mechanism**: With making use of or printing..
   - A Breadboard connecting 2x 2.5-6V DC-motors for the propulsion, connected to
   - A [ramp](./Media/base_ramp_2.jpeg) which is connected to the robot and
   - An extension of the ramp which gives the optimal angle for the propulsion in regards to the chosen motors
   - A [funnel](./Media/funnel_2.jpeg) guiding the balls to the DC-motors
4. **Connecting everything**: By a Python  script which automates
   - the movement,
   - the detection,
   - and activating the propulsion motors to collect and launch the LED-Ball

<img src="./Schemas/component_diagramm.png" style="max-width:600px; width:100%; height:auto;">

### Hardware  Components

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

## Mechanical Design

### Propulsion mechanism

Before designing our propulsion mechanism, we did some research like any responsible person would do: By watching YouTube videos. We found a clip where a propulsion device was created using two spinning DC-Motors, with wheels wrapped in rubber band attached, to shoot a Ping Pong ball.

We tried to recreate this contraption, cutting two circles with a hole in the middle into a piece of wood, using a Lasercutter. We attached the wooden circles to the DC-Motors and mounted the Motors onto another piece of wood, with one in between acting as a ramp. We powered the DC-Motors up by using a 4.5V battery pack for each with a button on a breadbord in between to control the power. As a ball, we used a scrambled up piece of paper, with some tape around to hold it together. Et voila, our first prototype was created and it worked:

[Watch the propulsion prototype video](./Media/Videos/propulsion_prototype_1.mp4)

<video src="./Media/Videos/propulsion_prototype_1.mp4" controls style="max-height:600px; height:100%; width:auto;"></video>

However, we noted that, altough on video the paper ball seems to fly quite a bit, in reality it was rather underwhelming. This could become a critical point of our project. The ball must fly high enough to create a good effect. And to add to that, our actual LED-ball is going to be heavier. Nevertheless, we decided to stick with this idea and hoped that through more precise work, we could still build this propulsion mechanism effective enough.

Anyhow, our first prototype showed promise and we started with designing and testing key components. After some discussion with our professors, which gave their helpful tips and ideas in class and at the midterm, we made up our mind, how we wanted to begin implementing a more robust prototype:

- Create a 3D-printed ball to hold the LED and coin battery which powers it.
- To reduce complexity, we decided to combine collection and propulsion into one. Thus, placing the spinning wheels close to the floor, to shoot the LED directly from there.
- Use a ramp to guide ball upwards and connect the DC-Motors to it.

And so we began, iterating through these three parts (LED-Ball, DC-Motor holder and wheels, Ramp) through making changes in the 3D-model to printing and adjusting through repeating this. We present the following main iterations and changes we made on these components, in the following Subsections.

#### LED-Ball

_Intial idea_:
Our first idea included 3D printing the outside of a ball and putting a LED inside that is directly attached to a coin battery. A bit like a Ping Pong ball with a light inside. To make the light shine through the ball we thought about adding holes to itside outside.

_Iteration 1_:

- **Considered Implementation:** To check how well our prototype would work with a 3D printed ball, we first pulled a pre-made 3D model from the internet, which useed hexagonal holes.
- **What Didn't Work:** The large size of the hex-shaped holes, cause the balls surface to be less-smooth. This could decrease how high the ball can be shot, when it rolls up the ramp. Further, the ball we printed was clearly to small to fit a LED and a coin battery inside.
- **Next Improvements:**
  - Create are own model
  - Balance size and number of holes wto maintain roundness of the ball.

_Iteration 2_:

- **Considered Implementation:** We modelled our own ball in Fusion360. First, we focused solely on the ball and its shape without the LED and coin battery in mind. But, we already considered that we need to put them inside somehow. Thus, we can't print the full ball at once. The Prusa Slicer has a neat feature, which allows to cut 3D-models and place connectors between the split shapes, such that one can assemble them later on. So, we printed the ball in halves, with small connectors, sticking out of one half and their corresponding holes in the other.
- **Special Considerations:** To make are ball easily adaptable, especially since, the hole and ball size might need to be changed, excessively used the parameter feature in Fusion360.
- **What Worked:** Our modelled ball rolled much nicer. Assembling the ball afterwards works.
- **What Didn't Work:** The connectors where two small and break easily to due the layer direction of the print. Also, the connector placement in  the Prusa Slicer isn't very exact, resulting in difficulties of in putting the halves together, when the connectors where symmetrically placed, such that it wasn't clear which plug belonged to which hole.
- **Next Improvements:**
  - Use larger and better placed connectors for increased stability
  - Add coin batter holder.

_Iteration 3_:

- **Considered Implementation:** To increase the size of the connectors, the thickness of the balls outside wall needed to be increased. The coin battery we wanted to place at the center of the ball to not disturb the weight distribution too much. Here, we also made a seperate test print, solely printing the coin battery holderholder, to determine a good fit beforehand.
- **What Worked:** Making the wall thicker added a little improvement. The ball's halves hold better together. The coin battery holder came out need and we even added a little hexagonal pattern to it.
- **What Didn't Work:** Still, the connectors are suboptimal, because they can still break, especially when disassembling the ball into its halves. The coin battery fits well inside, but there isn't much room for the LED to fit inside the ball.
- **Next Improvements:**
  - Model connectors directly in Fusion360, make them somehow larger and more robust.
  - Make room for the LED.

_Iteration 4_:

- **Considered Implementation:** We realised that using a dowel mechanism to connect the two ball halves instead of a one-sided plug would solve two problems. First, when modelling the connectors ourselves, it is difficult to eyeball the buffer well, such that the plug fits into the whole, but at the same time isn't too loose. Using a dowel allows us test the dowels diameter by printing it alone instead of the whole ball again. Secondly, it allows to print the dowel seperately and thus, by printing it horizontally the layers a printed such that our small dowel becomes nearly indestructible.
To make room for the LED, we realised that the battery holder might better be placed at the bottom of the ball. This would also provide a nice effect of the ball stabilizing with the LED pointing up, after rolling around due to the battery changing its center of mass.
Given its new position, we could also nicely align the new connectors with the coin battery hold, using now only two but instead thicker connectors.
- **What Worked:** The new dowels worked really well, and the coin battery still fits very nicely into its newly positioned holder. Due to placing the connectors no longer on the ball's outside wall, we could also reduce its thickness, reducing its weight.
- **What Didn't Work:** However, the coin battery fits so well that there isn't enough room for the led pins to fit into. The hex pattern on the battery holder added some print instabilities.
- **Next Improvements:** We need to add some cutouts for the LED pins to fit and remove the hex pattern.

_Final Iteration_:

- **Considered Implementation:** Early on our professors already gave use the idea to use transparent filament instead of hole cutouts to allow for better blob detection. And while we were first cautious with that, since then the light emitted might be to little, when we saw during blob detection development that the cutout holes create small ball shaped reflections on the floor that get mistaken for the LED ball in detection, we decided to go with their idea. So, we removed the holes, decreased the thickness of the ball's outside and printed with it transparent PLA.
We decided to remove the hexagonal pattern from the coin battery holder and add the LED pin cutouts.

#### Ramp

_Intial idea_:
The ramp should have a nice curvature without any edges in it, such that the LED-ball wouldn't be decelerated by anything. But it also needs to well guide the ball, so its track can't be flat, but needs to be curved such that the ball follows a straight line. For the best effect our ball would optimally be shot nearly straight up.

_Iteration 1_:

- **Considered Implementation:** We set out to create a solid base ramp with adjustable parameters for its height, length, curvature etc. such that we can easily adjust these, if they don't fit.
- **What Worked:** The ramp came out very solid already, proving that our concept is on the right track.
- **What Didn't Work:** We modelled the ramp such that it was lifted of the floor by a few centimeters. But, we adapted our project to place the wheels spun by the DC-Motors close to the floor, it would make a project a lot simpler to remove this extra cusion below the ramp and placed it at floor level, as well. When testig our ramp by flinging the ball into it using just our fingers, we realised that the curvature of the ramp was way too steep, and the exit angle of 90 degrees might be too.
- **Next Improvements:**
  - Change the steepness and exit angle.
  - Put the ramp to the floor, make adjustment for top cut off.

_Iteration 2_:

- **Considered Implementation:** Adapting the steepness, floor offset and curvature was easily done, due to us using pareameters in fusion. Thus, we could just set the floor height to zero, increase the length of the ramp and change the exit angle.
- **What Worked:** Now, the ball rolls up more easily and can be rolled into the ramp directly from the floor.
- **What Didn't Work:** The ramp takes quite some time to print. And we knew that we would still be adding things to it (e.g. DC-Motor holders).
- **Next Improvements:**
  - Make ramp more modifiable.

_Final Iteration_:

- **Considered Implementation:** To allow for a adding components to the ramp and thus, making external modifications, we added squared cutouts to the ramp, such that we can later simply uses connectors to plug other things onto it, similar to the dowels for the LED ball.

- DC-Holders

### Preliminary Tests

Having our key components ready, we wanted to check how well they work together. We assembled everything and ran the following tests:

[Watch our test video here](./Media/Videos/test_2_with_ramp_1.mp4) [or here](./Media/Videos/test_2_with_ramp_2.mp4).

<video src="./Media/Videos/test_2_with_ramp_1.mp4" controls style="max-width:500px; width:100%; height:auto;"></video>
<video src="./Media/Videos/test_2_with_ramp_2.mp4" controls style="max-width:500px; width:100%; height:auto;"></video>

- Ramp-Extension
- Funnel
- Turtelbot3 connector

<!--Vorlage für Beschreibung der 3D-Modelle 

_Intial idea_:
_Iteration 1_:

- _Considered Implementation:_
- _What Worked:_
- _What Didn't Work:_
- _Next Improvements:_ -->

#### Controlling the DC-Motors

To remotely control the DC-Motors we made use of our Arduino Adafruit Feather ESP32-S3, allowing us to control the DC-Motors via a ROS-topic.

Since we were concerned about the power of our propulsion mechanism, we wanted to have at least 5V DC-Motors. However, the Arduino runs on 3.3V and so, taking its signal alone as power isn't enough. We needed an external powersource to power up the DC-Motors, while the Arduinos signal is solely used for control.
This is the typical scenario for using a relay. Our professors also suggested using a driver module to allow to control more exactly the power the DC-Motors receiver. But since we only need either full or no power, we decided to use a relay module.
The DC-Motors we got, should be the ones with 2.5 - 6 V. We used a rechargable battery that in theory should generate 5V but in practice managed only about 2.9 V. Hence, we put the DC-Motors in parallel, as otherwise, the battery wouldn't even be able to turn them on.

**Problems:**
At first we used one batterypack to supply both the dc-motors and the arduino with power. However, this caused the arduino to power of as soon as the DC-Motors where switched on, likely due to different resistances and the batterie's low voltage. Thus, we ended up using two recharagable battery packs, one for the dc-motors and one for the arduino.

This alltogether gave us the following circuit, once more cleanly as a diagram modelled with Fritzing and once, the real implementation:

<img src="./Schemas/circuit.svg" style="max-width:600px; width:100%; height:auto;">

<img src="./Media/Images/circuit_irl.jpg" style="max-width:600px; width:100%; height:auto;">
(Here, we only have one DC-Motor connected through the red and pink wires leaving the image.)

Now, we only need to set up the connection between the ROS master and our Arduino. For this we modified the `TcpHelloWorld` example from `"File"->"Examples"->"Rosserial Arduino Library"` which comes with the `Rosserial Arduino Library`. But instead of publishing to a topic, we set up a topic called `\dc_motor_cmd`,on which `Strings` are sent, which we subscribe to. If our Arduino receives a String `"off"` then the arduino pushes low or no voltatge over PIN 13, to which the Relay Module is connected. If it receives `"on"`, then it pushes high enough voltage to set off the Relay, turning on the DC-Motors.

We set up the WiFi to connect to the Hotspot that all other components are connected to as well. Then, our ROS-master can communicate with the Arduino over the rosserial python through tcp. And we can simply publish to the ROS topic `\dc_motor_cmd` to control the DC-Motors.

### Autonomous Movement to LED-Ball

To enable our robot to autonomously move to one of many LED-Balls such that our simultaneous collection and propulsion mechanism can grab a hold of it, we require two mechanisms. First, our robot must be able to locate the LED-balls and secondly, our robot needs to be able to navigate to one of these LED-balls and move such that the ball nicely rolls through the funnel into the two spinning DC-Motors.

**Initial Idea**
Our first intuition was to use a USB-Camera, and then locate the shining 3D printed balls by finding the brightest pixel(s) in the camera's image. Then, we could try to center one of those "pixels"/detected LED-Ball within the camera's view, through rotating the robot (This would require the camera to be placed approximately at the center of the ramp.). When the detected LED-Ball then is centered, our Turtlebot could simply move straight and the let the ball roll into the spinning DC-Motors.

Given a hint by our professors, we decided to go for a more robust detection approach: instead of trying to interpret pixel data in an overly simplistic way, we used opencv's blob detection.

#### Detecting the LED-Balls

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

### Navigation to LED Balls

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

## Results

- Show demo video/link demo video
- Discussion of limitations and result in general?

## Discussion

- Change of initial idea
- Interpretation of results
- Lessons Learned

## Conclusion

- Summarize our project

## Perfect Project

- What improvements would we make?
- How would the perfect implementation with a team of xx people and many resources be like.
