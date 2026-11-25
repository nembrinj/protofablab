#!/usr/bin/env python3
'''
camera_lift.py

Raises and lowers the camera with a string on the stepper motor
Listens on /web/lift for up or down.
Each command moves the camera one step; within limits.

Anything with NOTE CHNGE in this file needs to be tested still. 
IMPORTANT!!!!: start this node with the camera at the BOTTOM. That position counts as 0.
'''

# NOTE: if this program running in the back messes 
# with the blocking beetween ros2 or website either rework blocking priorities or add threading (if thats even possible here)   

import time                          # used for the short pause between motor steps 
import rclpy                         # ROS 2 Python library: starts ROS and runs our node
from rclpy.node import Node          # base class for every ROS 2 node
from std_msgs.msg import String      # message type the web page sends ( up / down )
import RPi.GPIO as GPIO              # controls the Raspberry Pi's GPIO pins that drive the motor


#           SETTINGS 
PINS = [17, 18, 27, 22]   # motor driver inputs IN1, IN2, IN3, IN4 (BCM numbering)
DELAY = 0.001             # seconds per halfstep; increase if the motor stalls
STEP_SIZE = 2048          # halfsteps per click (4096 = one full motor turn)

#IMPORTANT CHANGE NOTE: THIS HASNT BEEN MEASURED YET THIS IS JUST ARBITRARY FOR NOW  
MAX_HEIGHT = 8192         # halfsteps from bottom to top 

UP_IS_CLOCKWISE = True    #NOTE: Adjust based on motor mounting flip False != clckwise

# Fullstep sequence for the stepper motor  (4 phases)
SEQ = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1],
]


class CameraLift(Node):
    '''
    ROS 2 node that winds the string up or down when the web page asks for it.
    '''

    def __init__(self):
        '''
        Sets up the GPIO pins and the subscriber.
        The camera must be at the bottom when the node starts (position 0).
        '''
        super().__init__('camera_lift')

        # set up the 4 motor pins as outputs all off
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PINS, GPIO.OUT, initial=0)

        self.position = 0   # current height in half-steps (0 = bottom)

        # Subscriber: calls on_command() when the web page sends up or down.
        # the bottom may be a trivial issue with blocking 
        # Queue size 1 = at most one extra click waits while the motor is moving.
        self.create_subscription(String, '/web/lift', self.on_command, 1)
        self.get_logger().info('camera_lift started (camera should be at the bottom)')

    def on_command(self, msg):
        '''
        Called when the web page sends up or down.
        Moves the camera one step, never below the bottom or above MAX_HEIGHT.

        Args:
            msg (String):  up  or  down 
        '''
        if msg.data == 'up':
            steps = min(STEP_SIZE, MAX_HEIGHT - self.position)    # dont go past the top
        elif msg.data == 'down':
            steps = -min(STEP_SIZE, self.position)                # dont go below the bottom
        else:
            return 

        if steps == 0:
            self.get_logger().info('limit reached. not moving')
            return

        self.move(steps)

    def move(self, steps):
        '''
        Turns the motor by the given number of halfsteps,

        A stepper motor turns by switching its 4 coils on and off in a fixed
        pattern (SEQ). Each row of SEQ is one halfstep. Playing the rows in
        order turns the motor one way playing them backwards turns it the other way.

        Args:
            steps (int): half-steps to move (+ = up, - = down)
        '''
        # 1. work out which way the motor has to turn
        going_up = steps > 0
        if going_up == UP_IS_CLOCKWISE:
            seq = SEQ          # play the pattern forwards  -> clockwise
        else:
            seq = SEQ[::-1]    # play the pattern backwards -> counter-clockwise

        # 2. Do the half-steps one by one
        for i in range(abs(steps)):
            phase = seq[i % 4]               # which of the 4 rows to use now (repeats 0-4)

            # switch each of the 4 coils on (1) or off (0) for this row
            for coil in range(4):
                GPIO.output(PINS[coil], phase[coil])

            time.sleep(DELAY)                # give the motor time to move before the next step (may need to play around with this)

        # 3. turn all coils off so the motor doesnt passively heat up
        GPIO.output(PINS, 0)

        # 4. log the new height position 
        self.position += steps
        self.get_logger().info('camera height: ' + str(self.position) + ' / ' + str(MAX_HEIGHT))


def main():
    '''
    Starts ROS 2 and the node, and releases the GPIO pins on Ctrl+C.
    '''
    rclpy.init()
    node = CameraLift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(PINS, 0)   # make sure the motor is off
        GPIO.cleanup()         # release the pins


if __name__ == '__main__':
    main()
