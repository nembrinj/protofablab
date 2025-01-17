#!/bin/bash

echo "simulation starts"
gnome-terminal -- bash -c "roslaunch pmb2_2dnav_gazebo pmb2_navigation.launch public_sim:=true lost:=false;"

sleep 20

echo "rosbridge starts"
gnome-terminal -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch"

echo "node-red starts"
# change path according to your folders
gnome-terminal -- bash -c "cd ./Complementary_Code/Node-RED_flows; node-red project_cat_feeder.json"

echo "successful launch"
