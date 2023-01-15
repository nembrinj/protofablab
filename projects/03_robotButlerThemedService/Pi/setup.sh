#!/bin/sh
sudo apt update
sudo apt-get install -y libhdf5-dev libhdf5-serial-dev python3-pyqt5 libatlas-base-dev libjasper-dev
sudo apt-get install cmake
sudo apt-get install mosquitto mosquitto-clients
sudo systemctl enable mosquitto.service
pip install --upgrade pip setuptools wheel
sudo pip3 install -r requirements.txt
