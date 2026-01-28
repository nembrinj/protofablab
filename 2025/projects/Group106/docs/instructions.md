## Creating a package

```bash
cd src/
catkin_create_pkg turtlebot3_profab std_msgs turtlebot3_msgs sensor_msgs geometry_msgs rospy
```

## Installing external packages

### On the robot

Repo dependencies
```shell
sudo apt-get install python3-vcstool
cd src/
vcs-import < ../dependencies-bot.repos
```
Binary dependencies
```shell
./install_apt_requirements.sh requirements/robot-requirements.apt
```

## Finding the IP address of the Raspberry Pi

Make sure the Raspberry is connected to the same local network as 
your laptop/PC (e.g. use ethernet cable for the Raspberry Pi)

```shell
hostname -I
sudo nmap -sn <hostname>/24
```
find ubuntu entry

## Updating the Wifi settings on the Raspberry Pi

```shell
sudo vim /etc/netplan/50-cloud-init.yaml
```

## Git user isolation on bot (not great, separate HOMEs)

```shell
sudo adduser <name>
git config --global user.name "<name>"
git config --global user.email "<email>"
```

## Using ssh key of laptop

```shell
ssh -A <user>@<ip>
```

## Launch Node-RED dashboard

```shell
node-red node-red/<dashboard name>
```

## Controlling the (Arduino) light intensity via a ROS Node

Deploy `./arduino/light_ros` onto the arduino (use correct Wifi credentials). 

Make sure tcp port (`make light_node`) and topic name match up with the Arduino code. 

Make sure the IP Adress in the Arduino code matches the roscore ip adress

In 2 different terminals:

```shell
roscore
make light_node
```

In a third terminal:

```shell
rostopic pub light_intensity std_msgs/Int16 <value between 0 and 255> --once
```

## Controlling the (Arduino) light intensity via Node-RED

```shell
make server
make light_node
node-red ./node-red/light_ros.json
```

## Controlling the motor speed (in RPMs) via a ROS Node
Deploy `./arduino/motor_ros` onto the arduino (use correct Wifi credentials). 

Make sure tcp port (`make motor_node`) and topic name match up with the Arduino code. 

In 2 different terminals:

```shell
roscore
make motor_node
```

In a third terminal:

```shell
rostopic pub motor_speed std_msgs/Int16 <value between 0 and 15> --once
```

or, for negative values:

```shell
rostopic pub motor_speed std_msgs/Int16 "data: <value between -15 and 15>" --once
```

## Creating a custom map

```shell
make robot (on robot)
roscore
make gazebo
make slam
make teleop 
```

move around using teleop.

Saving the map:
```shell
rosrun map_server map_saver -f ~/profab_ws/map/<map_name>
```