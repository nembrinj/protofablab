from flask import Flask, request, jsonify
from flask_cors import CORS
import roslibpy
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import os
import time
import threading
from collections import deque
import numpy as np
import cv2

# load the .env file
load_dotenv()

# Configurations for MQTT and ROS
MQTT_HOST = os.getenv('MQTT_HOST')
MQTT_PORT = int(os.getenv('MQTT_PORT'))
TOPIC_PEN_STATE = 'pen_state'
ROS_CLIENT_HOST = os.getenv('ROS_CLIENT_HOST')
ROS_CLIENT_PORT = int(os.getenv('ROS_CLIENT_PORT'))

# Flask app setup
app = Flask(__name__)
CORS(app)


# ****************************************************************************************************
# MQTT functions
def on_connect(client, userdata, flags, rc):
    """Callback function when MQTT connects."""
    if rc == 0:
        print("Successfully connected to MQTT broker.")
    else:
        print(f"Connection failed with error code {rc}")


def initialize_mqtt(mqtt_client):
    """Initialize and connect to the MQTT broker."""
    mqtt_client.on_connect = on_connect
    retries = 2  # Number of connection retries

    for attempt in range(retries):
        try:
            # Attempt connection
            mqtt_client.connect(MQTT_HOST, MQTT_PORT)
            print(
                f"Attempting to connect to MQTT broker {MQTT_HOST}:{MQTT_PORT}")
            mqtt_client.loop_start()  # Start the background loop
            time.sleep(0.5)  # Allow time for connection to establish

            # Check connection status
            if mqtt_client.is_connected():
                print("MQTT connected successfully!")
                return True
            else:
                print("MQTT connection not yet established. Retrying...")
        except Exception as e:
            print(f"MQTT connection failed: {e}")
        time.sleep(0.5)  # Wait before retrying

    print("Max retries reached. MQTT connection failed.")
    return False


# ****************************************************************************************************
# ROS functions
def initialize_ros(ros_client):
    retries = 2  # Number of connection retries
    for attempt in range(retries):
        try:
            ros_client.run()
            if ros_client.is_connected:
                print(f"ROS connected to {ROS_CLIENT_HOST}:{ROS_CLIENT_PORT}")
                return
        except Exception as e:
            print(f"ROS connection failed: {e}")
        if attempt < retries - 1:
            print("Retrying ROS connection...")
            time.sleep(2)
        else:
            print("Max retries reached for ROS. Exiting...")
            exit(1)


# ****************************************************************************************************
# Intialize Components
# Initialize MQTT connection
mqtt_client = mqtt.Client()
if initialize_mqtt(mqtt_client):
    print("MQTT is connected!")
else:
    print("Running without MQTT.")

# Initialize ROS connection
ros_client = roslibpy.Ros(host=ROS_CLIENT_HOST, port=ROS_CLIENT_PORT)
initialize_ros(ros_client)


# ****************************************************************************************************
# Get the map from the ROS topic /map and crop it to the non-unknown regions
global map_cached


# Callback function to process the map message from the ROS topic
def process_map_message(map):
    global map_cached
    width = map['info']['width']
    height = map['info']['height']
    resolution = map['info']['resolution']
    original_origin = map['info']['origin']

    # Convert map data to numpy array
    map_data = np.array(
        map['data'], dtype=np.int8).reshape((height, width))

    unique_values, counts = np.unique(map_data, return_counts=True)

    # Combine the values and their counts for easier interpretation
    value_counts = dict(zip(unique_values, counts))

    total_pixels = map_data.size
    for value, count in value_counts.items():
        percentage = (count / total_pixels) * 100
        print(f"Value: {value}, Percentage: {percentage:.2f}%")

    # Crop the image to remove the extra padding around the map
    unknown_value = -1

    # Create a binary map: non-unknown regions as 1, unknown as 0
    binary_map = (map_data != unknown_value).astype(np.uint8)

    # Find all non-zero (non-unknown) points
    # Returns an array of points [(x1, y1), (x2, y2), ...]
    coords = cv2.findNonZero(binary_map)

    # Calculate the bounding rectangle around the non-unknown region
    # (x, y) is the top-left corner, (w, h) are width and height
    x, y, w, h = cv2.boundingRect(coords)

    # Crop the original map image based on this bounding rectangle
    cropped_map = map_data[y:y+h, x:x+w]

    print(
        f'The image has been reduced in size by {(1 - (cropped_map.size / map_data.size))*100:.2f}% using as unknown value {unknown_value}.')

    # Save the cropped map data
    map['data'] = cropped_map.flatten().tolist()
    map['info']['width'] = w
    map['info']['height'] = h

    # Calculate the new origin
    new_origin_x = original_origin['position']['x'] + (x * resolution)
    new_origin_y = original_origin['position']['y'] + (y * resolution)

    # Update the origin in the map metadata
    map['info']['origin']['position']['x'] = new_origin_x
    map['info']['origin']['position']['y'] = new_origin_y

    # Save the cropped map to the global variable
    map_cached = map


# Subscribe to the /map topic to get the map data
map_topic = roslibpy.Topic(ros_client, 'map', 'nav_msgs/OccupancyGrid')
# Set the callback function to process the map message
map_topic.subscribe(process_map_message)


# ****************************************************************************************************
# *************************************ROUTES AND ENDPOINTS******************************************
# ****************************************************************************************************
# Routes for testing the backend
# Route to change the pen state (up or down) used for implement testing in the frontend
@app.route('/change_pen_state', methods=['POST'])
def change_pen_state():
    try:
        # Check if MQTT is connected
        if not mqtt_client.is_connected():
            return jsonify({'error': 'MQTT connection is not available'}), 503

        # Retrieve the pen state from the request
        pen_state = request.json.get('penState')
        if pen_state not in ['up', 'down']:
            return jsonify({'error': 'No pen state provided'}), 400

        # Publish the pen state to the MQTT broker
        mqtt_client.publish(TOPIC_PEN_STATE, pen_state)
        print(
            f"Pen state '{pen_state}' published to topic '{TOPIC_PEN_STATE}'")

        return jsonify({'message': 'Pen state changed successfully'}), 200
    except Exception as e:
        print('Error changing pen state:', e)
        return jsonify({'error': str(e)}), 500


# Route to send a goal to the robot
@app.route('/send_goal', methods=['POST'])
def send_goal():
    try:
        # Retrieve the goal coordinates from the request
        pose = request.json.get('pose')
        if not pose:
            return jsonify({'error': 'No pose provided'}), 400

        # Publish the goal to the ROS topic
        goal_topic = roslibpy.Topic(
            ros_client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
        poseMessage = roslibpy.Message(
            {"header": {"frame_id": "map"}, "pose": pose})
        goal_topic.publish(poseMessage)
        goal_topic.unadvertise()

        return jsonify({'message': 'Goal sent successfully'}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# ****************************************************************************************************
# Routes for the handling of the drawing process
# Route to get the map data (cropped to the non-unknown regions)
@app.route('/get_map',  methods=['GET'])
def get_map():
    try:
        if map_cached is None:
            return jsonify({'error': 'Map data not yet available.'}), 503

        # Return the cached map data
        return jsonify({'map': map_cached}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# Route to send a sequence of commands to the robot and initialize the drawing process
@app.route('/sequence_of_commands', methods=['POST'])
def sequence_of_commands():
    global monitoring_active, process_running
    try:
        # Retrieve the sequence of commands from the request
        commands = request.json.get('commands')
        if not commands:
            return jsonify({'error': 'No commands provided'}), 400

        monitoring_active.set()
        process_running = True  # Set process as running

        # Start a thread to monitor the robot's position and pen state
        monitor_thread = threading.Thread(
            target=monitoring_thread, daemon=True)
        monitor_thread.start()
        # Execute the sequence of commands in a separated thread
        command_thread = threading.Thread(
            target=execute_command_sequence_thread, args=(commands,))
        command_thread.start()

        return jsonify({'message': 'Commands sequence execution started'}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# Route to retrieve the monitoring data (robot's position and pen state)
@app.route('/get_monitoring_data', methods=['GET'])
def get_monitoring_data():
    global monitoring_buffer, monitoring_active
    try:
        # Convert the deque to a list for JSON serialization
        data = list(monitoring_buffer)

        # Clear the buffer after reading
        monitoring_buffer.clear()

        if not monitoring_active.is_set() and not data:
            return jsonify({'error': 'Monitoring data not available'}), 503

        return jsonify(data), 200
    except Exception as e:
        print(f"Error retrieving monitoring data: {e}")
        return jsonify({'error': str(e)}), 500


# Route to stop the drawing process (interrupting gracefully the drawing process)
@app.route('/stop_process', methods=['POST'])
def stop_process():
    global process_running
    try:
        # Set the flag to false to stop threads
        process_running = False
        return jsonify({'message': 'Process stopped successfully'}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# ****************************************************************************************************
# *************************************THREADS AND CALLBACKS******************************************
# Global flag to indicate drawing process status
process_running = False
# Event to control monitoring thread
monitoring_active = threading.Event()
# Shared buffer for robot's position and pen state
monitoring_buffer = deque(maxlen=1000)
# Variable to store the current pen state
current_pen_state = 'up'
# Variable to store the current goal status
current_goal_status = None


# Function to execute the sequence of commands
def execute_command_sequence_thread(commands):
    global current_goal_status, process_running

    # Subscribe to the status topic
    status_topic = roslibpy.Topic(
        ros_client, '/move_base/status', 'actionlib_msgs/GoalStatusArray')
    status_topic.subscribe(status_callback)

    try:
        for command in commands:
            if not process_running:
                print("Process stopped.")
                break  # Stop the sequence if requested
            try:
                if command['type'] == 'pen_state':
                    # Handle pen state change
                    pen_state = command['data']['pen_state']
                    mqtt_client.publish(TOPIC_PEN_STATE, pen_state)
                elif command['type'] == 'move':
                    # Handle goal command
                    pose = command['data']['pose']
                    goal_topic = roslibpy.Topic(
                        ros_client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
                    pose_message = roslibpy.Message(
                        {"header": {"frame_id": "map"}, "pose": pose})
                    goal_topic.publish(pose_message)

                    # Wait for the goal to be reached or failed
                    current_goal_status = None
                    # 3: Goal reached, 4: Goal failed
                    while current_goal_status not in [3, 4] and process_running:
                        time.sleep(0.2)

                    if current_goal_status == 3:
                        # goal reached successfully
                        continue
                    elif current_goal_status == 4:
                        # Failed to find a plan for the goal
                        break  # Stop the sequence on failure
                else:
                    print("Unknown command type:", command['type'])
            except Exception as e:
                print("Error processing command:", e)
                break  # Stop the sequence on any critical error
    finally:
        # Put the pen up
        mqtt_client.publish(TOPIC_PEN_STATE, "up")
        # Prepare the request to cancel the goal (an empty GoalID will cancel the last goal)
        cancel_goal = roslibpy.Topic(
            ros_client, '/move_base/cancel', 'actionlib_msgs/GoalID')
        goal_id = roslibpy.Message({
            'stamp': {'secs': 0, 'nsecs': 0},
            'id': ''
        })
        try:
            cancel_goal.publish(goal_id)
        except Exception as e:
            print(f"Error canceling the goal: {e}")
        # Unsubscribe from the status topic
        status_topic.unsubscribe()
        # Stop the monitoring thread
        monitoring_active.clear()


# Define the callback for the /move_base/status topic
def status_callback(message):
    global current_goal_status
    try:
        if 'status_list' in message:
            status_list = message['status_list']
            if status_list:
                # Use the last status in the list
                current_goal_status = status_list[-1]['status']
    except Exception as e:
        print("Error in status_callback:", e)


# Function to continuosly monitor the robot drawing process
def monitoring_thread():
    # Subscribe to the position topic /amcl_pose
    position_topic = roslibpy.Topic(
        ros_client, '/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped')
    position_topic.subscribe(position_callback)

    # Subscribe to the MQTT pen state topic
    mqtt_client.subscribe(TOPIC_PEN_STATE)
    mqtt_client.message_callback_add(TOPIC_PEN_STATE, pen_state_callback)

    try:
        while monitoring_active.is_set():
            time.sleep(0.1)  # Avoid busy-waiting
    finally:
        # Unsubscribe when stopping
        position_topic.unsubscribe()
        mqtt_client.message_callback_remove(TOPIC_PEN_STATE)
        print("Monitoring thread stopped.")


# Callback function to handle the reception of a message from /amcl_pose
def position_callback(message):
    global current_pen_state
    try:
        position = message['pose']['pose']['position']
        position_data = {'x': position['x'],
                         'y': position['y'], 'z': position['z']}

        # Save the position and pen state to the buffer
        monitoring_buffer.append(
            {'position': position_data, 'pen_state': current_pen_state})
    except Exception as e:
        print(f"Error in position_callback: {e}")


# Callback function to handle the change in the pen_state (if there is no mqtt connection
# the current_pen_state will stay always up)
def pen_state_callback(client, userdata, message):
    global current_pen_state
    try:
        current_pen_state = message.payload.decode()
    except Exception as e:
        print(f"Error in pen_state_callback: {e}")


if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        ros_client.terminate()
