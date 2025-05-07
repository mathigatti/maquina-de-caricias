import serial
import threading
from time import sleep

# Configure the serial connection
arduino_port = "/dev/ttyUSB0"  # Replace with your Arduino's serial port
baud_rate = 9600       # Must match the baud rate in the Arduino sketch

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

while True:
    with open("coords.txt", 'r') as f:
        coord = f.read()

    if len(coord) > 0:
        send_to_arduino(str(coord).replace(" ", ""))
        sleep(4)
        with open("coords.txt", 'w') as f:
            f.write("")
            
    else:
        sleep(1)

# Close the serial connection when done
if ser.is_open:
    ser.close()
