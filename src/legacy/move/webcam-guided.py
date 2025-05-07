import sys
from time import sleep

import numpy as np
from PIL import Image
import cv2
import cv2.aruco as aruco

from depth import depth_map
from sam import sam

# Define a helper to append a line to coords.txt
def send_coord(coord):
    """
    coord is a tuple or list of 3 floats (tip, left, right).
    Example: (-5.,0.,+5.)
    """
    with open("coords.txt", "w") as f:
        # Convert the tuple to a string like (-5.0, 0.0, 5.0)
        f.write(f"({coord[0]},{coord[1]},{coord[2]})\n")

def coord_ready():
    with open("coords.txt", "r") as f:
        return len(f.read()) == 0

# Tolerances (tweak to your needs).
area_tolerance = 1500
pos_tolerance = 100

high_height = 8000
low_height = 5500

def dynamic_step(error, min_step=1.0, max_step=50.0, factor=0.1):
    """
    Given some 'error' (distance from target in pixels, or difference in area),
    we scale that error by 'factor' to get a step value. 
    
    We then clamp that step between 'min_step' and 'max_step'.

    Returns the computed step (float).
    """
    step = error * factor
    step = max(min_step, min(step, max_step))
    return step

def move(area, position, target_area, target_position):
    """
    Moves the object closer to 'target_position' in the image plane and
    adjusts its distance (via its observed 'area') to approach 'target_area'.
    
    Instead of sending commands to motors directly, we append a 3D coordinate
    (tip, left_base, right_base) to a file called coords.txt. 
    
    Positive means giving rope; negative means taking rope.
    """

    # TODO: fix naming of variables and comments to make sense

    # Default "no move" instruction
    tip_move = 0.
    left_move = 0.
    right_move = 0.

    # Calculate differences
    area_diff = abs(area - target_area)
    diff_y    = abs(position[1] - target_position[1])  # referencing code that uses position[1] for "left-right"
    diff_x    = abs(position[0] - target_position[0])  # referencing code that uses position[0] for "front-back"

    # 1) Check if the object is at the desired "size" (area). 
    #    - If area < target_area: object is too far => pull all ropes (negative).
    #    - If area > target_area: object is too close => give all ropes (positive).
    if abs(area - target_area) > area_tolerance:
        step = dynamic_step(area_diff, min_step=1.0, max_step=20.0, factor=0.3)

        if area < target_area:
            # Pull from all motors
            tip_move   = -step
            left_move  = -step
            right_move = -step
        else:
            # Give rope from all motors
            tip_move   = +step
            left_move  = +step
            right_move = +step

    # 2) If the area is within tolerance, fix the X-position (left-right).
    #    - If current x < target x => object is too far left => 
    #      pull from right (negative) and let rope from left (positive).
    #    - If current x > target x => object is too far right => 
    #      pull from left (negative) and let rope from right (positive).
    elif abs(position[1] - target_position[1]) > pos_tolerance:
        step = dynamic_step(diff_y, min_step=1.0, max_step=15.0, factor=0.1)
        if position[1] < target_position[1]:
            # Move object to the right
            left_move  = -step
            right_move = +step
        else:
            # Move object to the left
            left_move  = +step
            right_move = -step

    # 3) Then fix the Y-position (front-back).
    #    - If y < target y => object is "above"/"up" => let rope from tip (positive).
    #    - If y > target y => object is "below"/"down" => pull rope from tip (negative).
    elif abs(position[0] - target_position[0]) > pos_tolerance:
        step = dynamic_step(diff_x, min_step=1.0, max_step=15.0, factor=0.2)
        if position[0] < target_position[0]:
            # Let rope (object moves down)
            tip_move = -step
        else:
            # Pull rope (object moves up)
            tip_move = +step

    # 4) If everything is within tolerance, we do nothing 
    #    (the moves remain (0,0,0)).

    # Finally, append the resulting coordinate
    move_coord = (tip_move, left_move, right_move)
    send_coord(move_coord)

def find_aruco_markers(frame, aruco_dict_type=aruco.DICT_4X4_100, debug=True):
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
                # Draw the detected markers and bounding boxes on the frame
                for i in range(len(ids)):
                    cv2.polylines(frame, [np.int32(corners[i])], True, (0, 255, 0), 2)
                cv2.imwrite("arucos_frame.jpg", frame)

            return { "position": (x_min, y_min), "width": width, "height": height, "area": area }

    else:
        print("No ArUco markers detected.")

import random

import random
import numpy as np
from PIL import Image

def choose_random_non_black_points(mask, n=5, x_min=None, x_max=None):
    """
    Given a grayscale PIL image 'mask' where non-black pixels
    have a value != 0, randomly choose n distinct non-black points
    whose x-coordinates (columns) lie in [x_min, x_max], and 
    return their (x, y) coordinates in a list.

    Parameters
    ----------
    mask : PIL.Image
        Grayscale PIL image.
    n : int, optional
        Number of points to sample. Default: 5.
    x_min : int or None, optional
        Minimum x-coordinate (column) to allow. If None, no minimum is enforced.
    x_max : int or None, optional
        Maximum x-coordinate (column) to allow. If None, no maximum is enforced.

    Returns
    -------
    list of (x, y) tuples
        Randomly chosen points in the given x-range that are non-black pixels.
    """
    # Convert the PIL Image to a NumPy array
    mask = mask.convert("L")  # 1 channel (grayscale)
    mask_np = np.array(mask)   # shape: (height, width)

    # Locate all pixels whose value is not zero (non-black)
    # np.argwhere returns (row, col) in (y, x) format
    non_black_coords = np.argwhere(mask_np != 0)  # shape: (N, 2)

    # Filter by x-range if specified
    # (non_black_coords[:,1] is the col = x)
    if x_min is not None:
        non_black_coords = non_black_coords[non_black_coords[:, 0] >= x_min]
    if x_max is not None:
        non_black_coords = non_black_coords[non_black_coords[:, 0] <= x_max]

    non_black_coords = non_black_coords[non_black_coords[:, 1] <= 570]
    non_black_coords = non_black_coords[non_black_coords[:, 1] >= 150]

    # Ensure there are enough non-black pixels in the valid x range
    if len(non_black_coords) < n:
        raise ValueError(
            f"Not enough non-black pixels in the specified x-range. "
            f"Needed {n}, found {len(non_black_coords)}."
        )

    # Randomly choose n distinct indices
    chosen_indices = random.sample(range(len(non_black_coords)), k=n)

    # For each chosen index, convert (row, col) => (x, y)
    chosen_points = []
    for idx in chosen_indices:
        row, col = non_black_coords[idx]
        chosen_points.append((col, row))  # (x=col, y=row)

    return chosen_points

if __name__ == "__main__":

    selected_dict = cv2.aruco.DICT_4X4_50

    camera_id = int(sys.argv[1])

    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Error: Could not open USB webcam. Check the index.")
        exit()

    with open("coords.txt", "w") as f:
        # Convert the tuple to a string like (-5.0, 0.0, 5.0)
        f.write("")

    debug = True

    ret, frame = cap.read()
    if debug:
        cv2.imwrite("captured_frame.jpg", frame)
    frame_depth = depth_map(frame, debug=debug)
    #frame_depth = cv2.cvtColor(frame_depth, cv2.COLOR_BGR2RGB)
    image_pil = Image.fromarray(frame_depth)
    frame_depth = sam(image_pil, debug=debug)
    
    positions = []
    for position in choose_random_non_black_points(frame_depth, n=100, x_min=330, x_max=460):
        positions.append(position+(high_height,))
        positions.append(position+(low_height,))
    positions.append((600, 320, 2500))
    print(positions)

    i = 0
    target_position = positions[0]


    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame from webcam.")
            break


        data = find_aruco_markers(frame, aruco_dict_type=selected_dict, debug=debug)
        # Display the frame
        if data and coord_ready():
            area = data["area"]
            position = data["position"]

            print("status:", position + (area,))
            print("target:", target_position)
            print("\n")
            if (abs(area - target_position[-1]) < area_tolerance 
                and abs(target_position[0] - position[0]) < pos_tolerance 
                and abs(target_position[1] - position[1]) < pos_tolerance):
                print("Target reached!")
                i+=1
                target_position = positions[i%len(positions)]

            # Otherwise, keep moving
            move(area, position, target_position[-1], target_position[:2])
        else:
            sleep(1)
    cap.release()

'''
(656, 1184) 1.80487805
130 cm ancho, 260 cm de largo
'''