import json
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

# Other constants (in centimeters)
WIDTH_TOTAL = 260
HEIGHT_TOTAL = 130

WIDTH_MAX = WIDTH_TOTAL - 60
WIDTH_MIN = 60

HEIGHT_MAX = HEIGHT_TOTAL/2 + 60
HEIGHT_MIN = HEIGHT_TOTAL/2 - 60

WIDTH_TOTAL_PX = 1280
HEIGHT_TOTAL_PX = 720

WIDTH_MAX_PX = WIDTH_TOTAL_PX - 360
WIDTH_MIN_PX = 360

HEIGHT_MAX_PX = HEIGHT_TOTAL_PX/2 + 260
HEIGHT_MIN_PX = HEIGHT_TOTAL_PX/2 - 260


# --- Motor positions (in the same coordinate system as the marker detection) ---
# We now use numpy arrays to perform vector arithmetic.
MOTOR_TIP_POS = np.array([WIDTH_TOTAL, HEIGHT_TOTAL/2, 0], dtype=float)   # e.g., top center
MOTOR_LEFT_POS = np.array([0, HEIGHT_TOTAL, 0], dtype=float)                # e.g., bottom left
MOTOR_RIGHT_POS = np.array([0, 0, 0], dtype=float)                          # e.g., bottom right

# Global variable to store last movement command (for tip, left, right)
moves_summatory = np.array([0.0, 0.0, 0.0], dtype=float)

REST_POSITION_CM = np.array([40, HEIGHT_TOTAL/2, 136], dtype=float)
REST_POSITION_PX = (400, 360) # And AREA = 2500

# --- Utility functions for vector operations ---
def mag(vector):
    """Returns the magnitude (Euclidean norm) of the given vector."""
    return np.linalg.norm(vector)

# Pre-compute initial string lengths (distance between rest position and motor positions)
motor_tip_string_length = mag(REST_POSITION_CM - MOTOR_TIP_POS)
motor_left_string_length = mag(REST_POSITION_CM - MOTOR_LEFT_POS)
motor_right_string_length = mag(REST_POSITION_CM - MOTOR_RIGHT_POS)

def read_config():
    """Reads configuration parameters from config.json.
       If the file or some keys are missing, defaults are used later."""
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
    'coord' is a tuple or list of 4 values: (tip, left, right, speed).
    """
    coord_str = f"({round(coord[0],2)},{round(coord[1],2)},{round(coord[2],2)},{coord[3]})"
    try:
        response = requests.post(API_URL, params={"cmd": coord_str})
        if response.status_code != 200:
            print(f"Error: received status {response.status_code}")
    except Exception as e:
        print(f"Error sending coordinate: {e}")

def dynamic_step(error, min_step=1.0, max_step=5.0, factor=1):
    """
    Computes a dynamic step value based on error.
    The error is scaled by 'factor' (modified by a configuration parameter)
    and then clamped between 'min_step' and 'max_step'.
    """
    config = read_config()
    movement_factor = config.get("MOVEMENT_FACTOR", 1.0)
    step = error * factor * movement_factor
    step = max(min_step, min(step, max_step))
    return step

def heuristic_move(area, current_camera_position, target_area, target_position):
    """
    Adjusts motor lengths based on the difference between the current object
    (size and position) and the target parameters.
    The command sent is a 4-tuple: (tip_move, left_move, right_move, DEFAULT_MOTOR_SPEED).
    """
    config = read_config()
    DEFAULT_MOTOR_SPEED = config.get("DEFAULT_MOTOR_SPEED", 100)
    AREA_TOLERANCE = config.get("AREA_TOLERANCE", 10)
    POS_TOLERANCE = config.get("POS_TOLERANCE", 5)

    tip_move = 0.0
    left_move = 0.0
    right_move = 0.0

    area_diff = abs(area - target_area)
    diff_y = abs(current_camera_position[1] - target_position[1])  # left-right error
    diff_x = abs(current_camera_position[0] - target_position[0])  # front-back error

    # 1) Adjust based on area difference
    if area_diff > AREA_TOLERANCE:
        step = dynamic_step(area_diff, min_step=0.2, max_step=5.0, factor=0.01)
        if area < target_area:
            # Object is too far: pull rope from all motors (reduce rope length)
            tip_move = -step
            left_move = -step
            right_move = -step
        else:
            # Object is too close: give rope to all motors (increase rope length)
            tip_move = step
            left_move = step
            right_move = step

    # 2) Adjust horizontal position if area is within tolerance
    elif diff_y > POS_TOLERANCE:
        step = dynamic_step(diff_y, min_step=0.2, max_step=5.0, factor=0.03)
        if current_camera_position[1] < target_position[1]:
            # Object is too far left: pull left motor and release right motor
            left_move = -step
            right_move = step
        else:
            # Object is too far right: pull right motor and release left motor
            left_move = step
            right_move = -step

    # 3) Adjust front-back position if still needed
    elif diff_x > POS_TOLERANCE:
        step = dynamic_step(diff_x, min_step=0.2, max_step=5.0, factor=0.05)
        if current_camera_position[0] < target_position[0]:
            # Object is too low (or far): release tip motor
            tip_move = -step
        else:
            # Object is too high (or near): pull tip motor
            tip_move = step

    move_coord = (tip_move, left_move, right_move, DEFAULT_MOTOR_SPEED)
    return move_coord

def deterministic_move(target_position):
    """
    Computes rope length adjustments to move the object to a given target position
    by comparing the desired distances from each motor and updating the stored lengths.
    Returns a 4-tuple: (tip_move, left_move, right_move, DEFAULT_MOTOR_SPEED)
    """
    global moves_summatory, motor_left_string_length, motor_right_string_length, motor_tip_string_length

    config = read_config()
    DEFAULT_MOTOR_SPEED = config.get("DEFAULT_MOTOR_SPEED", 100)

    # Ensure target_position is a numpy array for vector math.
    target_position = np.array(target_position, dtype=float)

    motor_left_desired_string_length = mag(target_position - MOTOR_LEFT_POS)
    motor_left_movement = motor_left_desired_string_length - motor_left_string_length

    motor_left_string_length = motor_left_desired_string_length

    motor_right_desired_string_length = mag(target_position - MOTOR_RIGHT_POS)
    motor_right_movement = motor_right_desired_string_length - motor_right_string_length
    motor_right_string_length = motor_right_desired_string_length

    motor_tip_desired_string_length = mag(target_position - MOTOR_TIP_POS)
    motor_tip_movement = motor_tip_desired_string_length - motor_tip_string_length
    motor_tip_string_length = motor_tip_desired_string_length

    move_coord = (motor_tip_movement, motor_left_movement, motor_right_movement, DEFAULT_MOTOR_SPEED)

    # Update the global summatory (only the motor movements, not speed)
    moves_summatory += np.array([motor_tip_movement, motor_left_movement, motor_right_movement])
    return move_coord

def undo_last_movements():
    """
    Generates a movement command that undoes the last recorded motor moves.
    """
    global moves_summatory
    config = read_config()
    DEFAULT_MOTOR_SPEED = config.get("DEFAULT_MOTOR_SPEED", 100)
    
    undo_command = (-moves_summatory[0], -moves_summatory[1], -moves_summatory[2], DEFAULT_MOTOR_SPEED)
    print("Undoing last move:", undo_command)
    send_coord(undo_command)
    moves_summatory = np.array([0.0, 0.0, 0.0], dtype=float)

def valid_coords(aruco_center):
    """
    Returns True if the given aruco_center (x, y) lies inside the triangle defined by:
      (WIDTH_MIN, HEIGHT_MAX), (WIDTH_MIN, HEIGHT_MIN) and (WIDTH_MAX, HEIGHT_TOTAL/2).
    Uses the triangle-area method with a 1% tolerance.
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
    Otherwise, if exactly one marker with id 3 is found, returns its data.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        if debug:
            for i, _ in enumerate(ids):
                cv2.polylines(frame, [np.int32(corners[i])], True, (0, 255, 0), 2)
            # Draw the valid-region triangle
            v1 = (int(WIDTH_MIN_PX), int(HEIGHT_MAX_PX))
            v2 = (int(WIDTH_MIN_PX), int(HEIGHT_MIN_PX))
            v3 = (int(WIDTH_MAX_PX), int(HEIGHT_TOTAL_PX/2))
            cv2.line(frame, v1, v2, (0, 0, 255), 2)
            cv2.line(frame, v2, v3, (0, 0, 255), 2)
            cv2.line(frame, v3, v1, (0, 0, 255), 2)
            cv2.imwrite("arucos_frame.jpg", frame)

        print("Detected marker IDs:", ids)
        try:
            # Look for marker with id == 3
            ids_list = ids.tolist()
            i = [id[0] for id in ids_list].index(3)
            x_min = int(min(corners[i][0][:, 0]))
            y_min = int(min(corners[i][0][:, 1]))
            width = int(max(corners[i][0][:, 0]) - x_min)
            height = int(max(corners[i][0][:, 1]) - y_min)
            area = width * height            
            return {"position": (x_min, y_min), "width": width, "height": height, "area": area, "multiple": len(ids_list) > 1}
        except Exception as e:
            print("Error processing marker with id==3:", traceback.format_exc())
            return None
    else:
        # No markers detected.
        return None

def check_hibernation_mode(cap, selected_dict, debug=True):
    """
    Captures 5 consecutive frames and if more than one marker is detected
    in at least 80% of them, returns True to trigger hibernation mode.
    """
    retries = 5
    multiple_count = 0
    for i in range(retries):
        ret, frame = cap.read()
        if not ret:
            continue
        data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
        if data is not None and data.get("multiple", False):
            multiple_count += 1
        sleep(0.1)
    return multiple_count >= int(0.8 * retries)

'''
def load_positions():
    """
    Returns a list of target positions (in centimeters as 3D coordinates)
    for deterministic_move.
    """
    return [(30, 55, 200), (30, 75, 200), (60, 75, 210), (60, 55, 200)]
'''

def pixel_to_real(pixel):
    """
    Converts pixel coordinates from an image to real-world coordinates in cm.
    
    Parameters:
        pixel (tuple): A tuple (px, py) with the pixel coordinates.
        height_total (float): The total height in cm in the real-world space.
        
    Returns:
        tuple: Real-world coordinates (x, y) in cm.
        
    Known reference:
        - Pixel (400, 360) maps to (40, height_total/2).
        - Scale: 1 pixel equals 0.23 cm.
    """
    # Scale factor in cm/pixel
    scale = 0.23

    # Unpack the pixel coordinates
    px, py, z = pixel

    # Calculate displacements from the reference pixel
    dx_pixels = px - 400
    dy_pixels = py - 360

    # Convert displacement from pixels to centimeters
    dx_cm = dx_pixels * scale
    dy_cm = dy_pixels * scale

    # Calculate the real-world coordinates using the reference point
    x_real = max(20, 40 + dx_cm)
    y_real = max(20, (HEIGHT_TOTAL / 2) + dy_cm)

    return (x_real, y_real, z)

def load_positions():

    config = read_config()
    high_height = config.get("HIGH_HEIGHT")
    low_height  = config.get("LOW_HEIGHT")
    step_size   = config.get("STEP_SIZE", 10)
    recorrido   = config.get("RECORRIDO", "azar")
    paso        = config.get("PASO", "continuo")

    # Load mask image and convert to grayscale
    mask_img = Image.open("mask.jpg").convert("L")
    mask_np = np.array(mask_img)

    # Create a binary image where black pixels (value 0) become 1 and white become 0.
    # (Note: connectedComponentsWithStats considers zero as background, so we invert the image.)
    binary = (mask_np == 0).astype(np.uint8)

    # Find connected components (islands) with full 8-connectivity.
    # Label 0 is the background.
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)

    # For each island (labels 1..num_labels-1), randomly select STEP_SIZE points.
    islands_points = []
    for label in range(1, num_labels):
        # Get coordinates (y, x) where the label matches.
        island_coords = np.argwhere(labels == label)
        if island_coords.size == 0:
            continue
        n_points = island_coords.shape[0]
        if n_points >= step_size:
            sampled_indices = np.random.choice(n_points, size=step_size, replace=False)
            sampled_points = island_coords[sampled_indices]
        else:
            sampled_points = island_coords
        # Convert (y, x) to (x, y) and add the low_height as the z coordinate.
        points = [(int(pt[1]), int(pt[0]), low_height) for pt in sampled_points]
        islands_points.append(points)

    merged_points = []
    if recorrido == "azar":
        # Merge all islands' points and shuffle them randomly.
        for points in islands_points:
            merged_points.extend(points)
        random.shuffle(merged_points)
    elif recorrido == "secuencial":
        # For each island, compute the average x and y.
        island_averages = []
        for points in islands_points:
            if points:
                avg_x = np.mean([pt[0] for pt in points])
                avg_y = np.mean([pt[1] for pt in points])
                island_averages.append((avg_x, avg_y))
            else:
                island_averages.append((float('inf'), float('inf')))
        # Sort the islands by their average positions (x primary, y secondary).
        sorted_islands = [
            points for _, points in sorted(
                zip(island_averages, islands_points), 
                key=lambda pair: (pair[0][0], pair[0][1])
            )
        ]
        # Concatenate the points from islands in the sorted order.
        for points in sorted_islands:
            merged_points.extend(points)
    else:
        raise ValueError("Invalid value for RECORRIDO. Expected 'azar' or 'secuencial'.")
        
    # If PASO is "subiendo_bajando", insert an intermediate point after each position.
    # The intermediate point has the same (x, y) but with z = HIGH_HEIGHT.
    if paso == "subiendo_bajando":
        new_points = []
        last_pos = None
        for pos in merged_points:
            if last_pos is not None:
                new_points.append((last_pos[0], last_pos[1], high_height))
            new_points.append((pos[0], pos[1], high_height))
            new_points.append(pos)
            last_pos = pos
        new_points.append((pos[0], pos[1], high_height))
        merged_points = new_points

    # convert from pixel to cm based on calculated constant
    return [pixel_to_real(point) for point in merged_points]


def list_camera_ids():
    index = 0
    arr = []
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            break
        else:
            arr.append(index)
        cap.release()
        index += 1
    print("CAMERA IDs:", arr)

if __name__ == "__main__":
    selected_dict = cv2.aruco.DICT_4X4_50

    list_camera_ids()
    camera_id = 0
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Error: Could not open USB webcam. Check the index.")
        exit()

    debug = True
    centering = True
    ret, frame = cap.read()

    while True:
        # Always update the config before a major step.
        config = read_config()
        # First, check if we should be in hibernation mode.
        if centering or check_hibernation_mode(cap, selected_dict, debug=debug):
            print("Hibernation mode triggered. Moving to hibernation target.")
            undo_last_movements()

            # Continue in hibernation until markers are under control.
            while centering or check_hibernation_mode(cap, selected_dict, debug=debug):
                # Flush the camera buffer.
                for _ in range(5):
                    cap.grab()

                ret, frame = cap.read()

                if not ret:
                    continue
                data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
                print("Hibernation status:", data)

                # If a valid marker is found, adjust to move toward rest position.
                if data is not None:
                    current_area = data["area"]
                    current_position = data["position"]
                    # Compare current values with hibernation parameters
                    if (abs(current_area - config.get("HIBERNATION_HEIGHT", 5000)) > config.get("AREA_TOLERANCE", 10) or
                        abs(current_position[0] - REST_POSITION_PX[0]) > config.get("POS_TOLERANCE", 5) or
                        abs(current_position[1] - REST_POSITION_PX[1]) > config.get("POS_TOLERANCE", 5)):
                        
                        move_coord = heuristic_move(current_area, current_position, config.get("HIBERNATION_HEIGHT", 5000), REST_POSITION_PX)
                        print("Computed move command:", move_coord)
                        send_coord(move_coord)
                    else:
                        motor_tip_string_length = mag(REST_POSITION_CM - MOTOR_TIP_POS)
                        motor_left_string_length = mag(REST_POSITION_CM - MOTOR_LEFT_POS)
                        motor_right_string_length = mag(REST_POSITION_CM - MOTOR_RIGHT_POS)

                        centering = False
                sleep(2)
            print("Exiting hibernation mode. Resuming normal operation.")

                
        invalid_camera_count = 0
                
        while not centering:
            # Load target positions (in centimeters) for normal operation.
            positions = load_positions()
            print("Target positions:", positions)

            for target_position in positions: 
                if invalid_camera_count > 5:
                    raise Exception("Error: Failed to capture frame from webcam.")

                config = read_config()  # refresh parameters before each iteration
                ret, frame = cap.read()
                if not ret:
                    invalid_camera_count += 1
                    continue
                
                data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
                
                # If multiple markers are detected, trigger hibernation.
                if data is not None and data.get("multiple", False):
                    print("Multiple markers detected in main loop. Returning to base position.")
                    centering = True
                    break
                
                print("Current target (cm):", target_position)
                move_coord = deterministic_move(target_position)
                print("Deterministic move command:", move_coord)            
                send_coord(move_coord)

                # Flush the camera buffer.
                for _ in range(5):
                    cap.grab()

                sleep(0.02)
