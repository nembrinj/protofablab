#!/usr/bin/env python3
'''
web_control.py

Receives simple text commands from the web page (via rosbridge) and turns
them into speed commands that make the TurtleBot3 drive.

Flow: web page button -> /web/cmd (String) -> this node -> /cmd_vel -> wheels (movement yay)

Safety: if the web page stops sending commands for 1 second

LiDAR circle: the circle buttons use the LiDAR to circle the person in front of
the camera (maths in lidar_circle.py). While circling the robot also stops if the
person is lost or something is in the way (press stop before circling again).
'''

import time                          # used for the safety timeout
import rclpy                         # ROS 2 Python library: starts ROS and runs our node
from rclpy.node import Node          # base class for every ROS 2 node
from rclpy.qos import qos_profile_sensor_data   # QoS settings that match the LiDAR publisher
from std_msgs.msg import String      # message type the web page sends (forward, stop. etc)
from sensor_msgs.msg import LaserScan   # message type of the LiDAR scan on /scan

# Really important:
#   TurtleBot3 on ROS 2 Jazzy accepts TwistStamped (speeds + timestamp)
#   on /cmd_vel. 
from geometry_msgs.msg import TwistStamped

import lidar_circle                  # our LiDAR circling math (lidar_circle.py)


#   SETTINGS
SPEED = 0.1        # forward/backward speed in m/s (Burger max is about 0.22)
TURN_SPEED = 1.0   # turning speed in rad/s when turning on the spot
RADIUS = 0.5       # circle radius in metres for circle_left / circle_right
                   # (not used by the LiDAR circle which measures the radius itself)


class WebControl(Node):
    '''
    ROS 2 node that listens to the web page on /web/cmd and drives
    the robot by publishing speed commands on /cmd_vel.
    '''

    def __init__(self):
        '''
        Sets up the publisher subscriber, starting state and a timer
        that runs update() 10 times per second.
        '''
        super().__init__('web_control')

        # Publisher: sends speed commands to the robots driver
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber: calls on_command() every time the web page sends a command
        self.sub = self.create_subscription(String, '/web/cmd', self.on_command, 10)

        # Subscriber: stores the newest LiDAR scan (sensor QoS so it matches the LiDAR)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.command = 'stop'          # current command from the web page (always start stopped)
        self.moving = False            # True while we are sending drive commands
        self.last_time = time.time()   # when the last web command arrived

        self.scan = None               # newest LiDAR scan
        self.radius = None             # locked circle radius (None = no person locked yet)
        self.halted = None             # circle command that was stopped by a safety check

        self.timer = self.create_timer(0.1, self.update)   # run update() every 0.1 s
        self.get_logger().info('web_control started')

    def on_command(self, msg):
        '''
        Called whenever the web page sends a command.

        Stores the command and the time it arrived. Only logs when the command
        changes, because the page sends the current command 10 times per second.

        Args:
            msg (ROS String): the command (use .data for standard py string)
        '''
        self.last_time = time.time()   # the page is still alive

        # After a LiDAR safety stop the page keeps sending the same circle command,
        # so ignore it until the user presses something else (stop).
        if msg.data == self.halted:
            return
        self.halted = None

        if msg.data != self.command:
            self.get_logger().info('command: ' + msg.data)
            self.radius = None        # new command -> forget the locked person
        self.command = msg.data

    def on_scan(self, msg):
        '''
        Called for every new LiDAR scan (about 5 per second). Just stores it.

        Args:
            msg (LaserScan): one full 360 degree scan
        '''
        self.scan = msg

    def send(self, linear, angular):
        '''
        Builds and publishes one speed command for the robot.

        Args:
            linear (float): forward (+) / backward (-) speed in m/s
            angular (float): turn left (+) / turn right (-) speed in rad/s
        '''
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()   # when the command was made
        msg.header.frame_id = 'base_link'                    # speeds are relative to the robot
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def safety_stop(self, reason):
        '''
        Stops a LiDAR circle because of a safety check and keeps it stopped
        until the user presses another button.

        Args:
            reason (str): why the robot stopped (shown in the terminal)
        '''
        self.get_logger().warn(reason + ' - stopping')
        self.halted = self.command
        self.command = 'stop'
        self.radius = None
        if self.moving:
            self.send(0.0, 0.0)
            self.moving = False

    def circle_step(self, direction):
        '''
        One update of the LiDAR circle (math in lidar_circle.py).

        Args:
            direction (int): +1 to circle driving forward, -1 driving backward
        '''
        if self.scan is None:
            self.safety_stop('no LiDAR data')
            return
        points = lidar_circle.scan_points(self.scan)

        # Safety: something in the way?
        if lidar_circle.path_blocked(points, direction):
            self.safety_stop('something is in the way')
            return

        # Step 1: find the person in front of the camera
        person = lidar_circle.find_person(points, self.radius)
        if person is None:
            self.safety_stop('no person found in front of the camera')
            return

        # Step 2: lock the radius the first time
        if self.radius is None:
            self.radius = person[0]
            self.get_logger().info('person locked at ' + str(round(self.radius, 2)) + ' m')

        # Step 3: steer around the person
        linear, angular = lidar_circle.circle_speeds(person, self.radius, direction)
        self.get_logger().info('person: ' + str(round(person[0], 2)) + ' m at ' + str(round(person[1])) + ' deg',
                               throttle_duration_sec=1.0)
        self.send(linear, angular)
        self.moving = True

    def update(self):
        '''
        Runs 10 times per second and sends the right speeds for the current command.

        Safety: if the web page has not sent anything for 1 second
        the command switches to  stop .
        '''
        if time.time() - self.last_time > 1.0:
            self.command = 'stop'

        if self.command == 'forward':
            self.send(SPEED, 0.0)                 # drive straight forward
        elif self.command == 'backward':
            self.send(-SPEED, 0.0)                # drive straight backward
        elif self.command == 'left':
            self.send(0.0, TURN_SPEED)            # turn left on the spot
        elif self.command == 'right':
            self.send(0.0, -TURN_SPEED)           # turn right on the spot

        #NOTE chnge: these now coincide with the lidar 
        elif self.command == 'circle_left':
            self.circle_step(+1)                  # circle by driving forward
            return
        elif self.command == 'circle_right':
            self.circle_step(-1)                  # circle by driving backward
            return
        else:
            # stop: send one zero command then stay quiet
            # I can not think of any current proposed benefits to looping 0 cmd
            self.radius = None
            if self.moving:
                self.send(0.0, 0.0)
                self.moving = False
            return

        self.moving = True   # we just sent a drive command


def main():
    '''
    Starts ROS 2, creates the node and keeps it running until Ctrl+C.
    '''
    rclpy.init()
    node = WebControl()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
