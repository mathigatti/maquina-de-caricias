import sys

import cv2

def shot(camera_id, debug=False):
    cap = cv2.VideoCapture(camera_id)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("Error: Could not open webcam. Check index or permissions.")
        return

    # Read one frame
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Error: Could not read frame.")
        cap.release()
        return
    
    if debug:
        cv2.imwrite("captured_frame.jpg", frame)
        print("Saved 'captured_frame.jpg'")

    cap.release()

if __name__ == "__main__":
    shot(int(sys.argv[1]),debug=True)
