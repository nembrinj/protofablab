import cv2
from ultralytics import YOLO
import logging
import paho.mqtt.client as mqtt

# MQTT settings
mqttServer = '172.17.0.1'
mqttPort = 1883
topic = 'Detection'  # Topic to publish to when a cat is detected

# Classes to detect
classes_to_find = ['cat', 'teddy bear']  # 'teddy bear' is used to detect a stuffed cat, for testing purposes

# State variables
oldstate = None  # to keep track of the previous detection
newstate = 'nothing'  # to keep track of the new detection

# To be sure the camera sees nothing and that it is not just a false detection
nothing_count = 0

# Function to send a notification to the MQTT broker
def send_notification(state):
    client.publish(topic, state, 1, True)
    print('Sending notification', state)


# Function to get the height of the bounding box of the detected object. This relates to the distance of the cat from the camera
def get_height(box):
    [x1, y1, x2, y2] = box.xyxy[0]
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

    return y2 - y1


# Function to set the camera settings
def set_camera_settings(videoCap):
    videoCap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # How many frames are stored in the buffer
    videoCap.set(cv2.CAP_PROP_FPS, 1)  # Frames per second
    videoCap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Width of the frames
    videoCap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Height of the frames


if __name__ == '__main__':
    logging.getLogger('ultralytics').setLevel(logging.WARNING)
    # Connect to the MQTT broker
    client = mqtt.Client()
    client.connect(mqttServer, mqttPort, 0)

    print('Application started')
    # Load the model
    yolo = YOLO('yolo11n.pt')
    print('Model loaded')
    
    # Load the video capture
    videoCap = cv2.VideoCapture(-1)
    set_camera_settings(videoCap)

    print('Video capture loaded')

    while True:
        ret, frame = videoCap.read()

        # If no frame has been grabbed, continue
        if not ret:
            continue

        # Get the results of the detection on the frame
        results = yolo.track(frame, stream=True)
        
        for result in results:
            # Get the classes names
            classes_names = result.names

            detected_objects = []

            # Iterate over each box
            for box in result.boxes:
                
                # Get the class
                cls = int(box.cls[0])

                # Get the class name
                class_name = classes_names[cls]

                detected_objects.append(class_name)

                # Check if confidence is greater than 40 percent and if the class is a cat
                if box.conf[0] > 0.4 and class_name in classes_to_find:
                        height = get_height(box)

                        # If the height is greater than 100 pixels, the cat is sufficiently close to the camera
                        if height > 100:
                            newstate = 'cat!'
                            nothing_count = 0
                        else:
                            newstate = 'cat is too far'
                            nothing_count = 0
            
            # Check that no cat or teddy bear are detected
            if len(list(set(classes_to_find) & set(detected_objects))) == 0:
                nothing_count += 1

        # No cat should be seen for six frame to say that no cat is present
        if nothing_count>=6:
            nothing_count = 0
            newstate = 'nothing'

        # Send notifications only when the state changes
        if oldstate != newstate:
            oldstate = newstate
            send_notification(newstate)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and destroy all windows
    videoCap.release()
    cv2.destroyAllWindows()
    print('Application done')
