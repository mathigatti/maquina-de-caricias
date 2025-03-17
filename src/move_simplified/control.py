import sys
import numpy as np
import cv2
import cv2.aruco as aruco
import requests
import random
import time
import math
from time import sleep
from PIL import Image

# URL of your FastAPI server (adjust as needed)
API_URL = "http://localhost:8000/command"

# Default motor speed (the fourth variable)
DEFAULT_MOTOR_SPEED = 2000

# Global variable to store last movement command
last_move = None

def send_coord(coord):
    """
    Sends the coordinate to the REST API.
    coord is a tuple or list of 4 values: (tip, left, right, speed).
    """
    coord_str = f"({coord[0]},{coord[1]},{coord[2]},{coord[3]})"
    try:
        response = requests.post(API_URL, params={"cmd": coord_str})
        if not response.status_code == 200:
            print(f"Error: received status {response.status_code}")
    except Exception as e:
        print(f"Error sending coordinate: {e}")

def coord_ready():
    return True

# Tolerances (tweak to your needs)
area_tolerance = 500
pos_tolerance = 10

high_height = 2500
low_height = 1500

# Default motor speed (the fourth variable)
DEFAULT_MOTOR_SPEED = 2000

# Global variable to store last movement command
last_move = None

# --- Motor positions (in the same coordinate system as the marker detection) ---
# Adjust these values as needed for your actual ceiling layout.
MOTOR_TIP_POS = (1280, 360)      # e.g., top center
MOTOR_LEFT_POS = (0, 720)     # e.g., bottom left
MOTOR_RIGHT_POS = (0, 0) # e.g., bottom right

def move(area, current_position, target_area, target_position, factor=0.1):
    """
    Smarter move function based on inverse kinematics.
    
    current_position: (x, y) from the marker detection (current horizontal position of the object)
    target_position: (x, y, h) where h is the desired constant vertical (or height-related) parameter.
    
    This function computes the required change in cable length for each motor to move the object
    from its current position to the target position while maintaining the same height.
    """
    global last_move
    # Extract positions
    x_current, y_current = current_position
    x_target, y_target = target_position  # h is treated as the vertical offset (kept constant)
    
    # Helper: compute cable length given motor position and object position
    def cable_length(motor_pos, x, y, h):
        # h is assumed to be constant (the vertical distance from the object to the ceiling)
        dx = x - motor_pos[0]
        dy = y - motor_pos[1]
        da = 60000 * 1/math.sqrt(h)
        print(dx*dx, dy*dy, da*da)
        return math.sqrt(dx*dx + dy*dy + da*da)
    
    # Compute current and target cable lengths for each motor
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

    # Save and send the computed movement command
    move_coord = (tip_move*factor, left_move*factor, right_move*factor, DEFAULT_MOTOR_SPEED)
    last_move = move_coord
    print("Computed move command:", move_coord)
    send_coord(move_coord)

def undo_last_move():
    """
    Sends the inverse of the last movement command.
    """
    if last_move is not None:
        undo_command = (-last_move[0], -last_move[1], -last_move[2], DEFAULT_MOTOR_SPEED)
        print("Undoing last move")
        send_coord(undo_command)

def find_aruco_markers(frame, aruco_dict_type=aruco.DICT_4X4_50, debug=True):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        for i, _ in enumerate(ids):
            x_min = int(min(corners[i][0][:, 0]))
            y_min = int(min(corners[i][0][:, 1]))
            width = int(max(corners[i][0][:, 0]) - x_min)
            height = int(max(corners[i][0][:, 1]) - y_min)
            area = width * height            
            if debug:
                for j in range(len(ids)):
                    cv2.polylines(frame, [np.int32(corners[j])], True, (0, 255, 0), 2)
                cv2.imwrite("arucos_frame.jpg", frame)
            return {"position": (x_min, y_min), "width": width, "height": height, "area": area}
    else:
        #print("No ArUco markers detected.")
        return None

def choose_random_non_black_points(mask, n=5, x_min=None, x_max=None):
    """
    Given a grayscale PIL image 'mask', randomly choose n distinct points where the pixel is black.
    Returns a list of (x, y) tuples.
    """
    mask = mask.convert("L")
    mask_np = np.array(mask)
    black_coords = np.argwhere(mask_np == 0)
    if len(black_coords) < n:
        raise ValueError(f"Not enough black pixels. Needed {n}, found {len(black_coords)}.")
    chosen_indices = random.sample(range(len(black_coords)), k=n)
    chosen_points = []
    for idx in chosen_indices:
        row, col = black_coords[idx]
        chosen_points.append((int(col), int(row)))
    return chosen_points

# --- Region parameters and valid_coords function ---
WIDTH_TOTAL = 1280
HEIGHT_TOTAL = 720

HEIGHT_MAX = HEIGHT_TOTAL/2 + 300
WIDTH_MAX = WIDTH_TOTAL - 300

HEIGHT_MIN = HEIGHT_TOTAL/2 - 300
WIDTH_MIN = 300

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
# --- End valid_coords ---

if __name__ == "__main__":
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
    if debug:
        cv2.imwrite("captured_frame.jpg", frame)

    input("Waiting for you to load mask.jpg")
    frame_depth = Image.open("mask.jpg").convert("L")
    mask_np = np.array(frame_depth)

    # Find coordinates of black pixels (pixel value == 0)
    black_coords = np.argwhere(mask_np == 0)

    # np.argwhere returns (row, col) = (y, x)
    average_y, average_x = np.mean(black_coords, axis=0)
    middle_x = int(average_x)
    middle_y = int(average_y)

    # Define a square with a chosen side length (e.g., 200 pixels)
    square_side = 200
    half_side = square_side // 2

    # Create target positions as the 4 vertices of the square, with a fixed third param of 2500
    positions = [
        (middle_x - half_side, middle_y - half_side, 2500),  # Top-left
        (middle_x + half_side, middle_y - half_side, 2500),  # Top-right
        (middle_x + half_side, middle_y + half_side, 2500),  # Bottom-right
        (middle_x - half_side, middle_y + half_side, 2500)  # Bottom-left
    ]
    print("Valid target positions:", positions)
    
    if not positions:
        print("No valid target positions found.")
        exit()

    i = 0
    target_position = positions[0]
    retry_count = 0  # Counter for invalid readings

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame from webcam.")
            break

        data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)

        # If marker is not detected or its center is not in the valid region, retry up to 3 times.
        if data is None or not valid_coords(data["position"]):
            retry_count += 1
            #print(f"Invalid state detected (marker missing or out of valid region). Retry attempt {retry_count}.")
            if retry_count >= 20:
                print("Max retries reached. Undoing last move and updating target.")
                undo_last_move()
                i += 1
                target_position = positions[i % len(positions)]
                retry_count = 0
            sleep(0.02)
            continue
        else:
            retry_count = 0  # Reset counter if reading is valid

        # If we have a valid marker, process the movement.
        area = data["area"]
        position = data["position"]

        print("status:", position + (area,))
        print("target:", target_position)
        print("\n")

        # If the current state is within tolerances of the target, select the next target.
        if (abs(area - target_position[-1]) < area_tolerance 
            and abs(target_position[0] - position[0]) < pos_tolerance 
            and abs(target_position[1] - position[1]) < pos_tolerance):
            print("Target reached!")
            i += 1
            target_position = positions[i % len(positions)]
        else:
            move(area, position, target_position[-1], target_position[:2])
            # Flush camera buffer by grabbing a few frames before next capture:
            for _ in range(5):
                cap.grab()

    cap.release()


