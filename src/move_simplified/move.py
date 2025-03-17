import threading

import cv2
import serial
from fastapi import FastAPI, BackgroundTasks, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse

# Configure the serial connection
arduino_port = "/dev/ttyUSB0"  # Replace with your Arduino's serial port
baud_rate = 9600  # Must match the baud rate in the Arduino sketch

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

def send_to_arduino(message):
    try:
        ser.write(message.encode())
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error sending to Arduino: {e}")

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
    if len(cmd) == 1:
        if cmd.upper() == "Z":
            print("Shutdown command received, but ignored for safety.")
            return {"status": "Shutdown command ignored for safety."}
        send_to_arduino(cmd.upper())
        return {"status": f"Command {cmd} sent"}
    else:
        return {"error": "Invalid command format"}

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
        
    # Encode the image as JPEG
    ret, jpeg = cv2.imencode('.jpg', img)
    if not ret:
        return Response("Error encoding image", status_code=500)
    
    return Response(content=jpeg.tobytes(), media_type="image/jpeg")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

