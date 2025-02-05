export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I)
roscore &
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun rosserial_python serial_node.py tcp 11311 &
roslaunch pmb2_2dnav_gazebo pmb2_navigation.launch public_sim:=true &
docker build -t smart-bin-node-red ./node-red && docker run -it -p 1880:1880 smart-bin-node-red
