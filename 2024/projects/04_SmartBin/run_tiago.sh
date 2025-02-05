export ROS_MASTER_URI= # Add your TIAGO ROS Master URI here
export ROS_IP= # Add your TIAGO ROS IP address here
roscore &
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun rosserial_python serial_node.py tcp 11311 &
docker build -t smart-bin-node-red ./node-red && docker run -it -p 1880:1880 smart-bin-node-red
