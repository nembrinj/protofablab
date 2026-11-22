#!/bin/bash
# Starts the camera, rosbridge, video server, control node and web page.
# Run the robot bringup in another deidcicated terminal first for this to work. 

source /opt/ros/jazzy/setup.bash
cd ~/robot_web

# I do not think the below will change at any point 
CAMERA=$(readlink -f /dev/v4l/by-id/usb-046d_Brio_105_2530ZB82RFC8-video-index0)

# adjusting width and height will cause seg fault here (idk why) also 
# also currently there is a caching issue per run cheap fix is to jus kill processes

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=$CAMERA \
  -p image_width:=640 -p image_height:=480 -p framerate:=15.0 \
  -p pixel_format:=mjpeg2rgb -p brightness:=128 &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run web_video_server web_video_server &
python3 web_control.py &
python3 camera_lift.py &  
python3 -m http.server 8000 &

echo "Open http://100.75.136.77:8000"
wait
