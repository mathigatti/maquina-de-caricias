import serial
import random
import threading
from time import sleep
from pynput import keyboard

# Configure the serial connection
arduino_port = "/dev/ttyUSB0"  # Replace with your Arduino's serial port
baud_rate = 9600       # Must match the baud rate in the Arduino sketch

X_TOTAL_CM=180
Y_TOTAL_CM=280
Z_TOTAL_CM=230

# Create a serial connection
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print(f"Connected to Arduino on {arduino_port}")
except Exception as e:
    print(f"Failed to connect to Arduino: {e}")
    exit()

# Function to read from Arduino
def read_from_arduino():
    while True:
        try:
            if ser.in_waiting > 0:
                message = ser.readline().decode().strip()
                if message:
                    print(f"Received: {message}")
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
            break

def random_point(x_min, x_max, y, z_min, z_max):
    granularity = 100
    x = random.randint(0,granularity)/granularity
    z = random.randint(0,granularity)/granularity
    
    x = x*(x_max-x_min)+x_min
    z = z*(z_max-z_min)+z_min
    return str((float(round(x)), float(y), float(round(z))))

def valid_coords(coord):
    x,y,z = coord
    # Function to calculate the area of a triangle given its vertices
    def triangle_area(x1, z1, x2, z2, x3, z3):
        return abs((x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2)) / 2.0)

    # Vertices of the triangle
    x1, z1 = 0, 0
    x2, z2 = X_TOTAL_CM, 0
    x3, z3 = X_TOTAL_CM / 2, Z_TOTAL_CM

    # Total area of the triangle
    total_area = triangle_area(x1, z1, x2, z2, x3, z3)

    # Areas of triangles formed with the point (x, z)
    area1 = triangle_area(x, z, x2, z2, x3, z3)
    area2 = triangle_area(x1, z1, x, z, x3, z3)
    area3 = triangle_area(x1, z1, x2, z2, x, z)

    # Check if the sum of the areas matches the total area
    return total_area == (area1 + area2 + area3) and 0 <= y <= Y_TOTAL_CM

# Start the read thread
read_thread = threading.Thread(target=read_from_arduino)
read_thread.daemon = True
read_thread.start()

# Define a function to send messages to the Arduino
def send_to_arduino(message):
    try:
        ser.write(message.encode())  # Send the message
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error sending to Arduino: {e}")

# Define a callback for key presses
def on_press(key):
    try:
        if key.char == "1":
            with open("coords.txt", 'r') as f:
                coords = f.read().split(" ")
            for coord in coords:
                send_to_arduino(str(coord).replace(" ", ""))
                sleep(7)
        if key.char == "2":
            for _ in range(12):
                center = (90,0,80)
                coord = random_point(center[0]-30,center[0]+30,center[1]+30,center[2]-70,center[2]+40)
                
                send_to_arduino(str(coord))
                sleep(7)

                send_to_arduino(str(center))
                sleep(7)

        if key.char == "3":
            for _ in range(8):
                center = (90,0,80)
                coord = random_point(center[0]-30,center[0]+30,center[1]+30,center[2]-70,center[2]+40)
                
                send_to_arduino(str(coord))
                sleep(4)


        if key.char == "4":
            for _ in range(4):
                x, y, z = (90,0,80) 
                d = 100               
                for coord in [(x - d, y, z), (x, y, z-d), (x + d, y, z), (x, y, z+d)]:
                    send_to_arduino(str(coord))
                    sleep(4)


        if key.char in {'f','r','z','q', 'a', 'w', 's', 'e', 'd', 'y', 'h', 'u', 'j', 'i', 'k', 'm', 'n', 'b', 'c','v'}:
            if key.char.upper() == "Z":
                raise Exception("Bye! :)")
            send_to_arduino(key.char.upper())  # Send the command in uppercase
    except AttributeError:
        # Handle special keys (non-character keys)
        pass

# Define a callback for key releases
def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        print("Exiting...")
        return False

# Start listening to the keyboard
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

# Close the serial connection when done
if ser.is_open:
    ser.close()
