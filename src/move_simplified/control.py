import json
import math
import random
import traceback
from time import sleep

import cv2
import cv2.aruco as aruco
import requests
import numpy as np
from PIL import Image

# URL of your FastAPI server (adjust as needed)
API_URL = "http://localhost:8000/command"

# A constant that is not part of the config.
MOVEMENT_FACTOR = 0.125

# Other constants
WIDTH_TOTAL = 1280
HEIGHT_TOTAL = 720

HEIGHT_MAX = HEIGHT_TOTAL/2 + 300
WIDTH_MAX = WIDTH_TOTAL - 300

HEIGHT_MIN = HEIGHT_TOTAL/2 - 300
WIDTH_MIN = 300

# --- Motor positions (in the same coordinate system as the marker detection) ---
# Adjust these values as needed for your actual ceiling layout.
MOTOR_TIP_POS = (WIDTH_TOTAL, HEIGHT_TOTAL/2)      # e.g., top center
MOTOR_LEFT_POS = (0, HEIGHT_TOTAL)                   # e.g., bottom left
MOTOR_RIGHT_POS = (0, 0)                             # e.g., bottom right

# Global variable to store last movement command
last_move = None


def read_config():
    """
    Reads configuration parameters from config.json.
    Expected JSON format:
    {
      "HIGH_HEIGHT": 2500,
      "LOW_HEIGHT": 1500,
      "DEFAULT_MOTOR_SPEED": 2000,
      "AREA_TOLERANCE": 500,
      "POS_TOLERANCE": 10,
      "MAX_MOVEMENT_RETRIES": 10,
      "AREA2DISTANCE_CONSTANT": 60000
    }
    Returns a dictionary with the parameter names and values.
    """
    try:
        with open("config.json", "r") as f:
            config = json.load(f)
    except Exception as e:
        print("Error reading config.json:", e)
        config = {}
    return config


def send_coord(coord):
    """
    Sends the coordinate to the REST API.
    coord is a tuple or list of 4 values: (tip, left, right, speed).
    """
    coord_str = f"({coord[0]},{coord[1]},{coord[2]},{coord[3]})"
    try:
        response = requests.post(API_URL, params={"cmd": coord_str})
        if response.status_code != 200:
            print(f"Error: received status {response.status_code}")
    except Exception as e:
        print(f"Error sending coordinate: {e}")

def move(area, current_position, target_area, target_position):
    """
    Smarter move function based on inverse kinematics.
    
    current_position: (x, y) from the marker detection (current horizontal position of the object)
    target_position: (x, y) where x,y are the desired horizontal coordinates.
    
    This function computes the required change in cable length for each motor to move the object
    from its current position to the target position while maintaining the same vertical parameter.
    """
    global last_move
    # Read the latest configuration parameters.
    config = read_config()
    DEFAULT_MOTOR_SPEED = config.get("DEFAULT_MOTOR_SPEED", 2000)
    AREA2DISTANCE_CONSTANT = config.get("AREA2DISTANCE_CONSTANT", 60000)
    
    x_current, y_current = current_position
    x_target, y_target = target_position

    def cable_length(motor_pos, x, y, h):
        dx = x - motor_pos[0]
        dy = y - motor_pos[1]
        da = AREA2DISTANCE_CONSTANT * 1 / math.sqrt(h)
        # Debug printing
        print(dx*dx, dy*dy, da*da)
        return math.sqrt(dx*dx + dy*dy + da*da)
    
    L_tip_current = cable_length(MOTOR_TIP_POS, x_current, y_current, area)
    L_tip_target  = cable_length(MOTOR_TIP_POS, x_target, y_target, target_area)
    delta_tip = L_tip_target - L_tip_current

    L_left_current = cable_length(MOTOR_LEFT_POS, x_current, y_current, area)
    L_left_target  = cable_length(MOTOR_LEFT_POS, x_target, y_target, target_area)
    delta_left = L_left_target - L_left_current

    L_right_current = cable_length(MOTOR_RIGHT_POS, x_current, y_current, area)
    L_right_target  = cable_length(MOTOR_RIGHT_POS, x_target, y_target, target_area)
    delta_right = L_right_target - L_right_current

    print(delta_tip, delta_left, delta_right)
    tip_move   = delta_tip
    left_move  = delta_left
    right_move = delta_right

    move_coord = (tip_move * MOVEMENT_FACTOR, left_move * MOVEMENT_FACTOR, right_move * MOVEMENT_FACTOR, DEFAULT_MOTOR_SPEED)
    last_move = move_coord
    print("Computed move command:", move_coord)
    send_coord(move_coord)

def undo_last_move():
    """
    Sends the inverse of the last movement command.
    """
    config = read_config()
    DEFAULT_MOTOR_SPEED = config.get("DEFAULT_MOTOR_SPEED", 2000)
    global last_move
    if last_move is not None:
        undo_command = (-last_move[0], -last_move[1], -last_move[2], DEFAULT_MOTOR_SPEED)
        print("Undoing last move")
        send_coord(undo_command)

def valid_coords(aruco_center):
    """
    Returns True if the given aruco_center (x, y) lies inside the triangle defined by:
      (WIDTH_MIN, HEIGHT_MAX), (WIDTH_MIN, HEIGHT_MIN) and (WIDTH_MAX, HEIGHT_TOTAL/2).
    Uses the triangle‐area method with a 1% tolerance.
    """
    x, y = aruco_center
    def triangle_area(x1, y1, x2, y2, x3, y3):
         return abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2)) / 2.0)
    x1, y1 = WIDTH_MIN, HEIGHT_MAX
    x2, y2 = WIDTH_MIN, HEIGHT_MIN
    x3, y3 = WIDTH_MAX, HEIGHT_TOTAL/2
    total_area = triangle_area(x1, y1, x2, y2, x3, y3)
    area1 = triangle_area(x, y, x2, y2, x3, y3)
    area2 = triangle_area(x1, y1, x, y, x3, y3)
    area3 = triangle_area(x1, y1, x2, y2, x, y)
    return abs(total_area - (area1 + area2 + area3)) < 0.01 * total_area

def find_aruco_markers(frame, aruco_dict_type=aruco.DICT_4X4_50, debug=True):
    """
    Detects ArUco markers in a frame.
    If more than one marker is detected, returns {"multiple": True}.
    Otherwise, if exactly one marker is found and its center is computed, returns its data.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        if debug:
            for i, id in enumerate(ids):
                cv2.polylines(frame, [np.int32(corners[i])], True, (0, 255, 0), 2)
            # Draw the valid-region triangle
            v1 = (int(WIDTH_MIN), int(HEIGHT_MAX))
            v2 = (int(WIDTH_MIN), int(HEIGHT_MIN))
            v3 = (int(WIDTH_MAX), int(HEIGHT_TOTAL/2))
            cv2.line(frame, v1, v2, (0, 0, 255), 2)
            cv2.line(frame, v2, v3, (0, 0, 255), 2)
            cv2.line(frame, v3, v1, (0, 0, 255), 2)
            cv2.imwrite("arucos_frame.jpg", frame)

        print(ids)
                    
        # Only one marker detected – process it.
        #i = ids.find(3)
        try:
            ids = ids.tolist()
            i = [id[0] for id in ids].index(3)
            x_min = int(min(corners[i][0][:, 0]))
            y_min = int(min(corners[i][0][:, 1]))
            width = int(max(corners[i][0][:, 0]) - x_min)
            height = int(max(corners[i][0][:, 1]) - y_min)
            area = width * height            
            return {"position": (x_min, y_min), "width": width, "height": height, "area": area, "multiple": len(ids) > 1}
        except:
            print(traceback.format_exc())
            return None
    else:
        # No markers detected.
        return None

def check_hibernation_mode(cap, selected_dict, debug=True):
    """
    Takes 5 consecutive frames (with a short delay) and counts how many times more than one marker is detected.
    If this happens at least 80% of the time, returns True (i.e. hibernation mode should be active).
    """
    retries = 5
    multiple_count = 0
    for i in range(retries):
        ret, frame = cap.read()
        if not ret:
            continue
        data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
        if data is not None and "multiple" in data and data["multiple"]:
            multiple_count += 1
        sleep(0.1)
    if multiple_count >= int(0.8 * retries):
        return True
    return False

if __name__ == "__main__":
    # Read config once to set up initial parameters.
    config = read_config()
    # Use the config parameter "high_height" for target positions.
    high_height = config.get("HIGH_HEIGHT", 2500)
    low_height  = config.get("LOW_HEIGHT", 1500)  # Not used further in this example.

    selected_dict = cv2.aruco.DICT_4X4_50

    camera_id = 0
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Error: Could not open USB webcam. Check the index.")
        exit()

    debug = True
    ret, frame = cap.read()
    if debug and ret:
        cv2.imwrite("captured_frame.jpg", frame)

    #input("Waiting for you to load mask.jpg")
    frame_depth = Image.open("mask.jpg").convert("L")
    mask_np = np.array(frame_depth)

    # Compute center from black pixels in mask
    black_coords = np.argwhere(mask_np == 0)
    average_y, average_x = np.mean(black_coords, axis=0)
    middle_x = int(average_x)
    middle_y = int(average_y)

    # Define a square side (200 pixels)
    square_side = 200
    half_side = square_side // 2

    # Create target positions using high_height from config.
    positions = [
        (middle_x - half_side, middle_y - half_side, high_height),  # Top-left
        (middle_x + half_side, middle_y - half_side, high_height),  # Top-right
        (middle_x + half_side, middle_y + half_side, high_height),  # Bottom-right
        (middle_x - half_side, middle_y + half_side, high_height)   # Bottom-left
    ]
    print("Valid target positions:", positions)
    
    if not positions:
        print("No valid target positions found.")
        exit()

    i = 0

    while True:
        # Always update the config before a major step.
        config = read_config()
        HIBERNATION_HEIGHT = 4500
        # First, check if we should be in hibernation mode.
        if check_hibernation_mode(cap, selected_dict, debug=debug):
            print("Hibernation mode triggered. Moving to hibernation target.")
            # In hibernation mode, we command the motors to move to:
            # target_area = 8000, target_position = (400, 360)
            while check_hibernation_mode(cap, selected_dict, debug=debug):
                ret, frame = cap.read()
                if not ret:
                    continue
                data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
                # If a valid single marker is found, move if not within tolerance.
                if data is not None:
                    current_area = data["area"]
                    current_position = data["position"]
                    if (abs(current_area - HIBERNATION_HEIGHT) > config.get("AREA_TOLERANCE", 500) or 
                        abs(current_position[0] - 400) > config.get("POS_TOLERANCE", 10) or 
                        abs(current_position[1] - 360) > config.get("POS_TOLERANCE", 10)):
                        move(current_area, current_position, HIBERNATION_HEIGHT, (400, 360))
                else:
                    print("Hibernation mode: multiple markers detected, waiting.")
                sleep(2)
            print("Exiting hibernation mode. Resuming normal operation.")
        
        # Normal main loop operation.
        invalid_movement_retry_count = 0  # Counter for invalid readings
        valid_movement_retry_count = 0    # Counter for movement retries
        target_position = positions[i % len(positions)]
        
        while True:
            config = read_config()  # refresh parameters before each iteration
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame from webcam.")
                break

            data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
            
            # If multiple markers are detected, exit to re-check hibernation.
            if data is not None and "multiple" in data and data["multiple"]:
                print("Multiple markers detected in main loop. Returning to hibernation check.")
                break

            if data is None or not valid_coords(data["position"]):
                invalid_movement_retry_count += 1
                if invalid_movement_retry_count >= 20:
                    print("Max retries reached. Undoing last move and updating target.")
                    undo_last_move()
                    i += 1
                    target_position = positions[i % len(positions)]
                    invalid_movement_retry_count = 0
                sleep(0.02)
                continue
            else:
                invalid_movement_retry_count = 0

            area = data["area"]
            position = data["position"]

            print("status:", position + (area,))
            print("target:", target_position)
            print("\n")

            # Check if the current reading is within tolerance.
            if (abs(area - target_position[-1]) < config.get("AREA_TOLERANCE", 500) and
                abs(target_position[0] - position[0]) < config.get("POS_TOLERANCE", 10) and 
                abs(target_position[1] - position[1]) < config.get("POS_TOLERANCE", 10)) or \
               valid_movement_retry_count > config.get("MAX_MOVEMENT_RETRIES", 10):
                print("Target reached!")
                i += 1
                target_position = positions[i % len(positions)]
                valid_movement_retry_count = 0  # Reset for new target
            else:
                move(area, position, target_position[-1], target_position[:2])
                valid_movement_retry_count += 1
                # Flush the camera buffer.
                for _ in range(5):
                    cap.grab()
            sleep(0.02)

    cap.release()
