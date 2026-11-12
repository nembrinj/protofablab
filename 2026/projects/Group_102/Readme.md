# Adaptive Training Assistant --- Run Guide

This guide describes the ROS 2 nodes in the Adaptive Training Assistant,
how they connect, and how to start the system.


## 1. System architecture

``` mermaid
flowchart TD
    CAM["USB camera"] -->|/image_raw/compressed| PD["person_detection"]
    PD -->|/person/preview/compressed| DASH["training_dashboard"]
    PD -->|person position / readiness| POS["positioning_controller"]
    POS -->|search controller| SEARCH["robot_search_controller"]
    SEARCH -->|/cmd_vel| ROBOT["TurtleBot3 Burger"]
    ODOM["TurtleBot3 odometry /odom"] --> SEARCH
    POS -->|servo commands| SERVO["servo_driver"]
    SERVO -->|UDP commands| ESP["ESP32 servo controller"]
    PD -->|person readiness| LOGIC["training_logic"]
    LOGIC -->|/exercise/enable| EX["exercise_counter"]
    CAM -->|/image_raw/compressed| EX
    EX -->|/exercise/rep_event| LOGIC
    LOGIC -->|training status / feedback| DASH
    EX -->|exercise metrics / feedback| DASH
```

The graph shows the main data flow. Exact topic names for every
connection can depend on the topic names configured in the source files.

## 2. What each node does

  -----------------------------------------------------------------------
  Node                                Purpose
  ----------------------------------- -----------------------------------
  `person_detection`                  Reads the compressed camera stream,
                                      detects the person, and publishes
                                      person-position/readiness
                                      information plus an annotated
                                      camera preview.

  `positioning_controller`            Coordinates person positioning and
                                      servo behavior. It also starts the
                                      robot search controller internally.

  `robot_search_controller`           Searches for the person and
                                      commands the robot through
                                      `/cmd_vel`. It needs `/odom`; it
                                      stops the robot if odometry times
                                      out. Do not launch it separately
                                      when using the main launch file.

  `servo_driver`                      Sends servo commands to the ESP32
                                      over UDP. The current configuration
                                      in the latest log is
                                      `172.20.10.4:4210`.

  `exercise_counter`                  Uses the camera and pose model to
                                      detect and count bilateral arm
                                      raises. It starts disabled and
                                      waits for `/exercise/enable`.

  `training_logic`                    Waits for stable person readiness,
                                      enables the exercise, tracks the
                                      target of 10 arm raises, and
                                      publishes training state/feedback.

  `training_dashboard`                Displays the annotated camera
                                      preview and training/exercise
                                      information. It subscribes to
                                      `/person/preview/compressed`.

  -----------------------------------------------------------------------

## 3. Before starting

You need: - The TurtleBot3 Burger powered on. - The Raspberry Pi and Mac
connected to the same network. - The USB camera connected to the
robot. - The ESP32 servo controller powered and reachable at the
configured IP/port. - ROS 2 Jazzy installed/configured in the Pixi
environment on the Mac and on the robot.

Use the same ROS domain on both computers:

``` bash
export ROS_DOMAIN_ID=102
```


Only use that example if Cyclone DDS is installed and is the
configuration you have chosen.

## 4. Start the Robot and Camera --- Raspberry Pi

Connect to the Raspberry Pi:

``` bash
ssh ubuntu@127.168.1.102
```

Choose one of the following options:

### Option A --- Combined launch file (if configured)

If you have created and built the `usbcam_robot.launch.py` file, start
both the TurtleBot3 and USB camera with one command:

``` bash
ros2 launch turtlebot3_bringup usbcam_robot.launch.py
```

### Option B --- Start them separately

If you have not created the combined launch file, open two separate
terminals on the Raspberry Pi.

**Terminal 1 --- Start the TurtleBot3:**

``` bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2 --- Start the USB camera:**

``` bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="mjpeg2rgb"
```

Keep both terminals running while using the robot.

You should see a publishing rate. If `/odom` is missing or has no
messages, fix the robot bringup/odometry first. Do not proceed to robot
movement tests until this works.

## 5. Build and start the application --- Mac terminal

Open a terminal on the Mac and enter the workspace:

``` bash
cd ~/ros_ws
```

Activate the Pixi environment using the method you normally use for this
workspace. If you use the Pixi shell:

``` bash
pixi shell
```

Set the ROS environment:

``` bash
export ROS_DOMAIN_ID=102
```

Build the application package. The ROS package name contains the
existing spelling `trainning`:

``` bash
colcon build --merge-install --packages-select adaptive-trainning-assistance
```

Source the merged workspace for Zsh:

``` bash
source install/setup.zsh
```

Then launch the complete application:

``` bash
ros2 launch adaptive-trainning-assistance training_assistant.launch.py
```

Keep this terminal open while the application is running.

The launch file starts these six application nodes: 1.
`person_detection` 2. `positioning_controller` 3. `servo_driver` 4.
`exercise_counter` 5. `training_logic` 6. `training_dashboard`

`robot_search_controller` is started internally by
`positioning_controller`; do not start a second copy manually.

## 6. Verify the system --- another Mac terminal

Open a second Mac terminal. Enter the Pixi environment and source the
workspace in that terminal too:

``` bash
cd ~/ros_ws
pixi shell
export ROS_DOMAIN_ID=102
source install/setup.zsh
```

Check that the main topics are visible:

``` bash
ros2 topic list
```

Check odometry specifically:

``` bash
ros2 topic info /odom
ros2 topic hz /odom
```

Check the camera stream:

``` bash
ros2 topic info /image_raw/compressed
ros2 topic hz /image_raw/compressed
```

Check the annotated preview:

``` bash
ros2 topic info /person/preview/compressed
ros2 topic hz /person/preview/compressed
```

Check exercise and training topics:

``` bash
ros2 topic list | grep -E 'person|exercise|training'
```

## 7. Important troubleshooting

### `Odometry timeout: robot stopped and waiting`

This is the current known blocker. The search controller is not
receiving odometry, so it stops the robot for safety.

1.  On the Raspberry Pi, check `ros2 topic hz /odom`.
2.  On the Mac, check `ros2 topic hz /odom`.
3.  Confirm `ROS_DOMAIN_ID=102` on both machines.
4.  Confirm the TurtleBot3 bringup is running on the Raspberry Pi.
5.  Do not launch another bringup if one is already active.

### Camera preview is missing

Check that `/image_raw/compressed` is publishing on the robot and
visible from the Mac. Then check `/person/preview/compressed`. The
dashboard subscribes to the annotated preview, not directly to the raw
camera topic.

### Exercise counter is disabled

This is expected at startup. The training logic enables it after stable
person readiness is detected. Check that person detection is receiving
camera frames and that the readiness topic is being published.

### Servo does not move

The startup message only confirms the driver initialized with the
configured IP and port. It does not prove the ESP32 received a valid
command. Confirm the ESP32 IP, UDP port, network connection, and the
exact command format expected by the ESP32 firmware before testing
movement.
