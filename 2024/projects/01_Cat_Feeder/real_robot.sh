#!/bin/bash

echo "simulation starts"
#change IP with yours on robot wifi (ip addr)
IP=128

gnome-terminal -- bash -c "export ROS_MASTER_URI=http://pmb2-56c:11311; export ROS_IP=10.68.0.$IP; cd ~/pmb2_public_ws/src/pmb2_navigation/pmb2_2dnav/config/rviz; rviz rviz -d navigation.rviz"

sleep 15

echo "rosbridge starts"
gnome-terminal -- bash -c "export ROS_MASTER_URI=http://pmb2-56c:11311; export ROS_IP=10.68.0.$IP; roslaunch rosbridge_server rosbridge_websocket.launch"

echo "node-red starts"
# change path according to your folders
gnome-terminal -- bash -c "cd ./Complementary_Code/Node-RED_flows; node-red project_cat_feeder.json"

echo "successful launch"
