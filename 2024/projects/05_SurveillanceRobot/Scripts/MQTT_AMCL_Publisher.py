#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
from tf.transformations import euler_from_quaternion

# MQTT Broker details
broker_address = "localhost"

def on_connect(client, userdata, flags, rc, properties=None):
    rospy.loginfo("Connected to MQTT broker with result code " + str(rc))

def amcl_callback(data):
    # Extract position
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y

    # Extract orientation (quaternion)
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w

    # Convert quaternion to yaw (Euler)
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])

    # Publish to MQTT as JSON
    msg = {"x": current_x, "y": current_y, "yaw": yaw}
    client.publish("pmb2/amcl_pose", json.dumps(msg))

if __name__ == '__main__':
    rospy.init_node('mqtt_amcl_publisher', anonymous=True)

    client = mqtt.Client(client_id="WSL2AMCLPublisher")
    client.on_connect = on_connect
    client.connect(broker_address, 1883)
    client.loop_start()

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.spin()
    client.loop_stop()