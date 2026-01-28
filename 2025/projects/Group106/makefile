# Launch navigation_map in tandem
controller:
	roslaunch turtlebot3_profab controller.launch

collavoidance:
	roslaunch turtlebot3_profab collavoid.launch

sim_collavoidance:
	roslaunch turtlebot3_profab sim_collavoid.launch

sim_goal:
	rosrun turtlebot3_profab goal_controller.py

image_processing:
	rosrun turtlebot3_profab image_processing.py

# Control robot
#
# 1.) local-1: roscore
# 2.) local-2: make ssh_robot
# 3.) robot-1: roslaunch turtlebot3_bringup turtlebot3_robot.launch

# 4.a) local-3: make teleop
# 4.b) local-3: rosrun turtlebot3_profab ca_controller.py

# To see the real robot in simulation
# 5.) Optional: make gazebo 

ssh_robot:
	ssh ${TURTLEBOT3_HOSTNAME}

robot:
	roslaunch turtlebot3_bringup turtlebot3_robot.launch

teleop:
	roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

gazebo:
	roslaunch turtlebot3_gazebo turtlebot3_world.launch

slam:
	roslaunch turtlebot3_slam turtlebot3_slam.launch

save_map:
	rosrun map_server map_saver -f ~/profab_ws/map/profablab_02

navigation:
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch

navigation_map:
	roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/ubuntu/profab_ws/map/profablab_02.yaml

# CAMERA
# 1.) On computer:
# make navigation (launches rviz)
# 2.) On robot:
camera:
	roslaunch turtlebot3_profab turtlebot3_robot_usbcam.launch
# Check it works:
# rostopic echo /usb_cam/camera_info
# rostopic echo /usb_cam/camera_info
# Otherwise: pgrep video and pkill -9 <video-process-id>

camera_laptop:
	roslaunch turtlebot3_profab laptop_usbcam.launch

# Launches a ROS node to handle serial communication with the Arduino over TCP. 
# I.e. ROS message -> (serialize) -> byte sequence over TCP -> (deserialize) -> ROS message
# This node, running on the PC, is a rosserial server.
# On the Arduino, there is a rosserial client running (via the "Rosserial Arduino Library")
# Based on the configs of the Arduino, will create publishers and subscribers.
# TODO: maybe rename to rosserial_node
light_node:
	rosrun rosserial_python serial_node.py tcp 11511 __name:=serial_node_1

motor_node:
	rosrun rosserial_python serial_node.py tcp 11512 __name:=serial_node_2

# ROS node used for Node-RED <-> ROS communication
server:
	roslaunch rosbridge_server rosbridge_websocket.launch

rqt:
	rqt
