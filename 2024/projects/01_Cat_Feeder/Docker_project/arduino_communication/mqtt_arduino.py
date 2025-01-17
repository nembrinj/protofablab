# ls /dev/tty* to check serials
import serial
import paho.mqtt.client as mqtt
import glob

# MQTT settings
mqttServer = '172.17.0.1'
mqttPort = 1883
mqttTopic1 = 'Detection'  # Topic telling if a cat is detected
mqttTopic2 = 'Instructions'  # Topic telling if the cat can be fed

can_feed = False
sees_cat = False

# MQTT connection, for details see documentation https://pypi.org/project/paho-mqtt/
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribe the client to the topics
    client.subscribe(mqttTopic1)
    client.subscribe(mqttTopic2)

# MQTT message handling
def on_message(client, userdata, msg):
    global can_feed
    global sees_cat
    # Decode the message
    message = msg.payload.decode("utf-8")
    topic = msg.topic
    print('Topic', topic)
    print('Message', message, '\n')

    # If a feeding instruction is received (feed!), and previously the cat was not to be fed, the cat can now be fed
    if(can_feed == False and topic == 'Instructions' and message=='feed!'):
        can_feed = True

    # If a feeding instruction is received (do not feed!), and previously the cat was to be fed, the cat now cannot be fed
    if(can_feed == True and topic == 'Instructions' and message=='do not feed!'):
        can_feed = False

    # If a cat is detected, and previously the cat was not detected, the cat is now seen by the camera
    if(sees_cat == False and topic == 'Detection' and message=='cat!'):
        sees_cat = True

    # If a cat is no more detected, but previously the cat was detected, now the cat is not seen by the camera
    if(sees_cat == True and topic == 'Detection' and message!='cat!'):
        sees_cat = False

    # If the cat is seen and can be fed, send the feeding instruction to the arduino
    if(sees_cat == True and can_feed == True):
        can_feed = False
        print("sending to arduino")
        ser.write(b"feed")
        # Send to the broker that the cat has been fed
        client.publish('Instructions', 'fed', 1, True)

# Find the USB device and connect to it
usb_device = glob.glob('/dev/ttyUSB*')[0]
ser = serial.Serial(usb_device, 9600, timeout=1)
print(f"connected to {usb_device}")

# Setup MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(mqttServer, mqttPort, 0)

# Loop forever
client.loop_forever()
