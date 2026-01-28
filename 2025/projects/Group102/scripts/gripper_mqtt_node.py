#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from std_msgs.msg import String

class GripperMQTT:
    def __init__(self):
        rospy.init_node("gripper_mqtt_node")
        self.broker = rospy.get_param("~broker_ip", "192.168.1.17")

        self.event_pub = rospy.Publisher("/robot/event", String, queue_size=10)
        rospy.Subscriber("/robot/event", String, self.on_event)

        self.client = mqtt.Client()
        self.client.on_message = self.on_mqtt
        self.client.connect(self.broker, 1883, 60)
        self.client.subscribe("claw/state")
        self.client.loop_start()

        rospy.loginfo(f"[MQTT] connected to {self.broker}")
        rospy.spin()

    def on_event(self, msg):
        if msg.data == "REACHED_OBJECT":
            self.client.publish("claw/cmd", "CLOSE")
            rospy.loginfo("[MQTT] claw/cmd <- CLOSE")
        elif msg.data == "ARRIVED_DROPOFF":
            self.client.publish("claw/cmd", "OPEN")
            rospy.loginfo("[MQTT] claw/cmd <- OPEN")

    def on_mqtt(self, client, userdata, msg):
        payload = msg.payload.decode(errors="ignore").strip().upper()
        rospy.loginfo(f"[MQTT] {msg.topic}={payload}")
        if payload == "CLOSED":
            self.event_pub.publish(String("GRIPPED"))
        elif payload == "OPENED":
            self.event_pub.publish(String("RELEASED"))

if __name__ == "__main__":
    GripperMQTT()
