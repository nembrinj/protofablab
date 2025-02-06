#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped
import json
from tf.transformations import quaternion_from_euler

# MQTT Broker details
broker_address = "localhost"

rospy.init_node('mqtt_goal_subscriber', anonymous=True)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode('utf-8'))
    goal_x = data.get("x")
    goal_y = data.get("y")
    goal_yaw = data.get("yaw", 0.0)  # Default yaw to 0 if not provided

    # Convert yaw to quaternion
    q = quaternion_from_euler(0, 0, goal_yaw)

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.orientation.x = q[0]
    goal_msg.pose.orientation.y = q[1]
    goal_msg.pose.orientation.z = q[2]
    goal_msg.pose.orientation.w = q[3]

    goal_pub.publish(goal_msg)
    rospy.loginfo(f"Published new goal: x={goal_x}, y={goal_y}, yaw={goal_yaw}")

def on_connect(client, userdata, flags, rc, properties=None):
    rospy.loginfo("Connected to MQTT broker with result code " + str(rc))
    client.subscribe("pmb2/goal")

if __name__ == '__main__':
    client = mqtt.Client(client_id="WSL2GoalSubscriber")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address, 1883)
    client.loop_start()

    rospy.spin()
    client.loop_stop()