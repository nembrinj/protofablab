#!/usr/bin/env python3
'''
web_control.py

Receives simple text commands from the web page (via rosbridge) and turns
them into speed commands that make the TurtleBot3 drive.

Flow: web page button -> /web/cmd (String) -> this node -> /cmd_vel -> wheels (movement yay)

Safety: if the web page stops sending commands for 1 second
'''

import time                          # used for the safety timeout
import rclpy                         # ROS 2 Python library: starts ROS and runs our node
from rclpy.node import Node          # base class for every ROS 2 node
from std_msgs.msg import String      # message type the web page sends (forward, stop. etc)

# Really important:
#   TurtleBot3 on ROS 2 Jazzy accepts TwistStamped (speeds + timestamp)
#   on /cmd_vel. A plain Twist is ignored wheels would not move.
from geometry_msgs.msg import TwistStamped


#   SETTINGS 
SPEED = 0.1        # forward/backward speed in m/s (Burger max is about 0.22)
TURN_SPEED = 1.0   # turning speed in rad/s when turning on the spot
RADIUS = 0.5       # circle radius in metres for circle_left / circle_right


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

        # Publisher: sends speed commands to the robot's driver
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber: calls on_command() every time the web page sends a command
        self.sub = self.create_subscription(String, '/web/cmd', self.on_command, 10)

        self.command = 'stop'          # current command from the web page (always start stopped)
        self.moving = False            # True while we are sending drive commands
        self.last_time = time.time()   # when the last web command arrived

        self.timer = self.create_timer(0.1, self.update)   # run update() every 0.1 s
        self.get_logger().info('web_control started')

    def on_command(self, msg):
        '''
        Called whenever the web page sends a command.

        Stores the command and the time it arrived. Only logs when the command
        changes, because the page sends the current command 10 times per second.

        Args:
            msg (String): the command, e.g.  forward ,  stop ,  circle_left 
        '''

        if msg.data != self.command:
            self.get_logger().info('command: ' + msg.data)
        self.command = msg.data
        self.last_time = time.time()   # the page is still alive

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


        # NOTE: these are currently just simple naive cirvles and dont operate around a specific fixed point 
        elif self.command == 'circle_left':
            # Driving forward while turning makes a circle: w = v / r
            self.send(SPEED, SPEED / RADIUS)
        elif self.command == 'circle_right':
            self.send(SPEED, -SPEED / RADIUS)     # same but turning right
        else:
            # stop: send one zero command then stay quiet
            # I can not think of any current proposed benefits to looping 0 cmd
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