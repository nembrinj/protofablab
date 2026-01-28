#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json

class Controller:
    def __init__(self):
        rospy.init_node("controller_node")
        rospy.loginfo("Controller node started")

        # Subscribers and Publishers for FSM communication
        self.cmd_pub = rospy.Publisher("/controller/cmd", String, queue_size=10)
        self.status_sub = rospy.Subscriber("/controller/status", String, self.status_callback)
    
        # MQTT
        self.broker = "172.20.10.4"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
    
        self.client.connect(self.broker, 1883, 60)
        rospy.loginfo("MQTT bridge connected to broker")

        self.client.loop_start()

        # initial State
        self.state = "LOCALISING"
        rospy.sleep(1)
        self.cmd_pub.publish("localise")
        
        self.light_intensity = 0.0
        
    # MQTT CALLBACKS
    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"MQTT connected with code{rc}")

        # subscriptions
        client.subscribe("robot/command")
        rospy.loginfo("Subscribed to robot/command for button commands")
        
        client.subscribe("light/intensity")
        rospy.loginfo("Subscribed to light/intensity for light sensor data")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        rospy.loginfo(f"[MQTT] {topic} = {payload}")
        
        # handle light intensity updates
        if topic == "light/intensity" and float(payload) != 0.0:
            self.light_intensity = float(payload)
            print(f"Updated light intensity to: {self.light_intensity}")
        
        # handle button commands
        elif topic == "robot/command":
            rospy.loginfo(f"[MQTT] Button command received: {payload}")
            if payload == "FORWARD":
                publish.single("light/intensity", str(0), hostname=self.broker)
                self.cmd_pub.publish("move_forward")
                rospy.loginfo("########### Moving Forward ###########")
            elif payload == "BACKWARD":
                publish.single("light/intensity", str(0), hostname=self.broker)
                self.cmd_pub.publish("move_backward")
                rospy.loginfo("########### Moving Backward ###########")
            elif payload == "CIRCLE_RIGHT":
                self.cmd_pub.publish("move_circle_right")
                rospy.loginfo("########### Moving in Circle Right ###########")
            elif payload == "CIRCLE_LEFT":

                self.cmd_pub.publish("move_circle_left")
                rospy.loginfo("########### Moving in Circle Left ###########")  
            elif payload == "STOP":
                self.cmd_pub.publish("stop")
                rospy.loginfo("########### Stopping all actions ###########")
            
            self.state = "FACE_FOLLOWING"
                

    def status_callback(self, msg):
        # FSM transitions based on status messages
        # LOCALISING -> FACE_SEARCH
        if self.state == "LOCALISING" and msg.data == "localised":
            publish.single("light/intensity", str(0.5), hostname=self.broker)
            print("##############################33   ", self.light_intensity)
            self.state = "FACE_SEARCH"
            rospy.loginfo("########### Localization completed, starting face search ###########")
            self.cmd_pub.publish("search_face")

        # FACE_SEARCH -> FACE_FOLLOWING
        elif self.state == "FACE_SEARCH" and msg.data == "face_found":
            print("##############################33", self.light_intensity)
            if self.light_intensity != 0.0:
                publish.single("light/intensity", str(self.light_intensity), hostname=self.broker)
                rospy.loginfo(f"[MQTT] Published light intensity: {self.light_intensity}")
            self.state = "FACE_FOLLOWING"
            rospy.loginfo("########### Face found, starting following ###########")
            self.cmd_pub.publish("follow_face")
            
        # FACE_FOLLOWING -> FACE_SEARCH
        elif self.state == "FACE_FOLLOWING" and msg.data == "face_lost":
            publish.single("light/intensity", str(0.5), hostname=self.broker)
            self.state = "FACE_SEARCH"
            rospy.loginfo("########### Face lost, starting search ###########")
            self.cmd_pub.publish("search_face")


if __name__ == "__main__":
    node = Controller()
    rospy.spin()
