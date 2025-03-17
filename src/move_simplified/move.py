import serial
import random
import threading
from time import sleep
import cv2
from fastapi import FastAPI, BackgroundTasks, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

# Configure the serial connection
arduino_port = "/dev/ttyUSB0"  # Replace with your Arduino's serial port
baud_rate = 9600  # Must match the baud rate in the Arduino sketch

X_TOTAL_CM = 180
Y_TOTAL_CM = 280
Z_TOTAL_CM = 230

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print(f"Connected to Arduino on {arduino_port}")
except Exception as e:
    print(f"Failed to connect to Arduino: {e}")
    exit()

# Global event to signal when "OK" is received from Arduino
ok_event = threading.Event()

def read_from_arduino():
    while True:
        try:
            if ser.in_waiting > 0:
                message = ser.readline().decode().strip()
                if message:
                    print(f"Received: {message}")
                    if message == "OK":
                        ok_event.set()
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
            break

def random_point(x_min, x_max, y, z_min, z_max):
    granularity = 100
    x = random.randint(0, granularity) / granularity
    z = random.randint(0, granularity) / granularity
    x = x * (x_max - x_min) + x_min
    z = z * (z_max - z_min) + z_min
    return str((float(round(x)), float(y), float(round(z))))

def valid_coords(coord):
    x, y, z = coord
    def triangle_area(x1, z1, x2, z2, x3, z3):
        return abs((x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2)) / 2.0)
    x1, z1 = 0, 0
    x2, z2 = X_TOTAL_CM, 0
    x3, z3 = X_TOTAL_CM / 2, Z_TOTAL_CM
    total_area = triangle_area(x1, z1, x2, z2, x3, z3)
    area1 = triangle_area(x, z, x2, z2, x3, z3)
    area2 = triangle_area(x1, z1, x, z, x3, z3)
    area3 = triangle_area(x1, z1, x2, z2, x, z)
    return total_area == (area1 + area2 + area3) and 0 <= y <= Y_TOTAL_CM

def send_to_arduino(message):
    try:
        ser.write(message.encode())
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error sending to Arduino: {e}")

# Predefined command handlers (for non-coordinate commands)
def handle_command_1():
    try:
        with open("coords.txt", 'r') as f:
            coords = f.read().split(" ")
        for coord in coords:
            send_to_arduino(str(coord).replace(" ", ""))
            sleep(7)
    except Exception as e:
        print(f"Error in command 1: {e}")

def handle_command_2():
    center = (90, 0, 80)
    for _ in range(12):
        coord = random_point(center[0]-30, center[0]+30, center[1]+30, center[2]-70, center[2]+40)
        send_to_arduino(str(coord))
        sleep(7)
        send_to_arduino(str(center))
        sleep(7)

def handle_command_3():
    center = (90, 0, 80)
    for _ in range(8):
        coord = random_point(center[0]-30, center[0]+30, center[1]+30, center[2]-70, center[2]+40)
        send_to_arduino(str(coord))
        sleep(4)

def handle_command_4():
    x, y, z = (90, 0, 80)
    d = 100
    for _ in range(4):
        for coord in [(x - d, y, z), (x, y, z-d), (x + d, y, z), (x, y, z+d)]:
            send_to_arduino(str(coord))
            sleep(4)

#
# Synchronously send a command and wait for an "OK" from Arduino.
#
def send_command_and_wait(cmd):
    ok_event.clear()
    send_to_arduino(cmd)
    if ok_event.wait(timeout=10):  # wait up to 10 seconds
        ok_event.clear()
        return True
    else:
        return False

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
def startup_event():
    read_thread = threading.Thread(target=read_from_arduino, daemon=True)
    read_thread.start()

@app.post("/command")
async def send_command(cmd: str, background_tasks: BackgroundTasks):
    """
    POST /command?cmd=<command>
    
    For coordinate commands (e.g. "(x,y,z,s)" where s is motor speed), the server waits for an "OK" from the Arduino.
    For predefined commands ("1", "2", "3", "4") or single key commands, it processes as before.
    """
    # Coordinate command: starts with '(' and ends with ')' and contains commas.
    if cmd.startswith("(") and cmd.endswith(")") and "," in cmd:
        command_str = cmd.replace(" ", "")
        success = send_command_and_wait(command_str)
        if success:
            return {"status": f"Coordinate command {cmd} processed successfully"}
        else:
            return {"error": "Timeout waiting for OK from Arduino"}
    
    # Predefined sequences
    if cmd == "1":
        background_tasks.add_task(handle_command_1)
        return {"status": f"Command {cmd} processing"}
    elif cmd == "2":
        background_tasks.add_task(handle_command_2)
        return {"status": f"Command {cmd} processing"}
    elif cmd == "3":
        background_tasks.add_task(handle_command_3)
        return {"status": f"Command {cmd} processing"}
    elif cmd == "4":
        background_tasks.add_task(handle_command_4)
        return {"status": f"Command {cmd} processing"}
    elif len(cmd) == 1:
        if cmd.upper() == "Z":
            print("Shutdown command received, but ignored for safety.")
            return {"status": "Shutdown command ignored for safety."}
        send_to_arduino(cmd.upper())
        return {"status": f"Command {cmd} sent"}
    else:
        return {"error": "Invalid command format"}


# Image and valid region parameters (in pixels)
WIDTH_TOTAL = 1280
HEIGHT_TOTAL = 720

HEIGHT_MAX = HEIGHT_TOTAL/2 + 300
WIDTH_MAX = WIDTH_TOTAL - 300

HEIGHT_MIN = HEIGHT_TOTAL/2 - 300
WIDTH_MIN = 300

@app.get("/", response_class=HTMLResponse)
def index():
    html_content = """
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8">
      <title>Detected Marker Overlay</title>
      <meta http-equiv="refresh" content="2">
    </head>
    <body>
      <h1>Detected Marker with Valid Region Overlay</h1>
      <img src="/overlay.jpg" alt="Overlayed Marker Image">
    </body>
    </html>
    """
    return html_content

@app.get("/overlay.jpg")
def get_overlay_image():
    # Load the image from disk
    img = cv2.imread("./arucos_frame.jpg")
    if img is None:
        return Response("Image not found", status_code=404)
    
    # Draw the valid-region triangle
    v1 = (int(WIDTH_MIN), int(HEIGHT_MAX))
    v2 = (int(WIDTH_MIN), int(HEIGHT_MIN))
    v3 = (int(WIDTH_MAX), int(HEIGHT_TOTAL/2))
    cv2.line(img, v1, v2, (0, 0, 255), 2)
    cv2.line(img, v2, v3, (0, 0, 255), 2)
    cv2.line(img, v3, v1, (0, 0, 255), 2)
    
    # Encode the image as JPEG
    ret, jpeg = cv2.imencode('.jpg', img)
    if not ret:
        return Response("Error encoding image", status_code=500)
    
    return Response(content=jpeg.tobytes(), media_type="image/jpeg")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

