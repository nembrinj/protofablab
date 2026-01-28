import cv2
import numpy as np
import yaml

# MAP PROCESSING 
def load_map(pgm_path, yaml_path):
    """Load a map from a PGM image and its corresponding YAML metadata."""
    # binary map: 0-free, 255-occupied
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    # apply threshold to get binary image
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)

    # load map metadata
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    return binary, data["resolution"], data["origin"]

def find_column(binary_map):
    """Find the position of the column in the binary map."""
    # find contours of the obstacles
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # filter small contours
    contours = [c for c in contours if cv2.contourArea(c) > 50]

    # find the largest contour assuming it's the column
    contour = max(contours, key=cv2.contourArea)
    # get the center of the contour as the column position
    (x, y), _ = cv2.minEnclosingCircle(contour)
    return int(y), int(x)

def grid_to_world(row, col, grid_shape, resolution, origin):
    """Convert grid indices to world coordinates."""
    x0, y0, theta0 = origin
    num_filas = grid_shape[0]

    # grid to map coordinates conversion based on resolution and origin
    x = x0 + col * resolution
    y = y0 + (num_filas - row) * resolution

    # rotate according to origin orientation
    if theta0 != 0.0:
        x_rot = x0 + (x - x0) * np.cos(theta0) - (y - y0) * np.sin(theta0)
        y_rot = y0 + (x - x0) * np.sin(theta0) + (y - y0) * np.cos(theta0)
        return x_rot, y_rot

    return x, y
    
def point_towards_column(x_robot, y_robot, x_column, y_column):
    """Calculate the angle the robot should point towards the column."""
    yaw = np.arctan2(y_column - y_robot, x_column - x_robot) + np.pi/2
    return yaw
    
def distance_to_column(x_robot, y_robot, x_column, y_column):
    """Calculate Euclidean distance to the column."""
    return np.sqrt((x_robot - x_column)**2 + (y_robot - y_column)**2)