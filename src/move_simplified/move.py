import threading
import cv2
import serial
import json
import os
from fastapi import FastAPI, BackgroundTasks, Response, HTTPException, UploadFile, File
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
    index_path = os.path.join(os.path.dirname(__file__), "index.html")
    if os.path.exists(index_path):
        with open(index_path, "r", encoding="utf-8") as f:
            return f.read()
    else:
        return HTMLResponse(content="<h1>index.html not found</h1>", status_code=404)

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

@app.get("/config")
def get_config():
    """
    Retrieve the current configuration from config.json.
    """
    config_path = os.path.join(os.path.dirname(__file__), "config.json")
    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        return config
    else:
        raise HTTPException(status_code=404, detail="config.json not found")

@app.put("/config")
async def update_config(new_config: dict):
    """
    Update config.json with provided fields.
    Only provided fields will be updated; other fields remain unchanged.
    """
    config_path = os.path.join(os.path.dirname(__file__), "config.json")
    # Load existing config if it exists; otherwise, start with an empty dictionary.
    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            current_config = json.load(f)
    else:
        current_config = {}
    
    # Update the config with new values (only provided keys will be updated)
    current_config.update(new_config)
    
    try:
        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(current_config, f, indent=2)
        return {"status": "Config updated successfully", "config": current_config}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/mask")
async def save_mask(file: UploadFile = File(...)):
    """
    Receives a mask image in JPG format and saves it as mask.jpg.
    The expected image should be a black and white mask (white background with black drawn parts).
    """
    contents = await file.read()
    mask_path = os.path.join(os.path.dirname(__file__), "mask.jpg")
    try:
        with open(mask_path, "wb") as f:
            f.write(contents)
        return {"status": "Mask saved successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
