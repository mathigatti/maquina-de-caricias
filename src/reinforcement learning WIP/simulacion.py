from vpython import *

# Set up the 3D scene (mouse controls are enabled by default)
scene = canvas(title="3D Three-Motor Suspension Simulation",
               width=800, height=600, center=vector(0,2,0),
               background=color.white)

# Create a floor (a large gray box) and a small blue rectangle (target) on top
floor = box(pos=vector(0, -0.1, 0), size=vector(20, 0.2, 20), color=color.gray(0.5))
target = box(pos=vector(0, 0.05, 0), size=vector(2, 0.1, 2), color=color.blue)

# Ceiling height for the motors
ceiling_y = 5

# Define motor positions (vertices of an equilateral triangle on the ceiling)
motor1_pos = vector(-2, ceiling_y, 0)       # Left vertex
motor2_pos = vector(2, ceiling_y, 0)        # Right vertex
motor3_pos = vector(0, ceiling_y, 3.464)     # Front vertex (~4*sqrt(3)/2)

# Create visual representations for the motors
motor_size = vector(0.5, 0.2, 0.5)
motor1 = box(pos=motor1_pos, size=motor_size, color=color.gray(0.5))
motor2 = box(pos=motor2_pos, size=motor_size, color=color.gray(0.5))
motor3 = box(pos=motor3_pos, size=motor_size, color=color.gray(0.5))

# Initial cable lengths (in meters)
cable1_length = 3.0
cable2_length = 3.0
cable3_length = 3.0

# Cable control parameters
cable_rate = 0.5         # meters per second change
min_cable_length = 1.0
max_cable_length = 100.0  # Super long cables

# Cable stiffness (spring constant)
k = 100  # N/m

# Create visual representations for the cables (as thin black cylinders)
cable_radius = 0.05
cable1 = cylinder(pos=motor1_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)
cable2 = cylinder(pos=motor2_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)
cable3 = cylinder(pos=motor3_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)

# Create the payload (red sphere) without a trail
payload = sphere(pos=vector(0, 1, 1), radius=0.2, color=color.red)
payload.mass = 0.5
payload.velocity = vector(0, 0, 0)

# Increase gravity strength so the heavy payload is pulled down noticeably
g = vector(0, -30, 0)

# Key control mapping for cable adjustments:
# Motor 1: 'w' (shorten) and 's' (lengthen)
# Motor 2: 'e' (shorten) and 'd' (lengthen)
# Motor 3: 'r' (shorten) and 'f' (lengthen)
keys_pressed = {"w": False, "s": False, "e": False, "d": False, "r": False, "f": False}

def keydown(evt):
    # Debug print: shows which key was pressed (helpful for checking if "e"/"d" are captured)
    #print("key pressed:", evt.key)
    if evt.key in keys_pressed:
        keys_pressed[evt.key] = True

def keyup(evt):
    if evt.key in keys_pressed:
        keys_pressed[evt.key] = False

scene.bind("keydown", keydown)
scene.bind("keyup", keyup)

def compute_cable_force(motor_pos, cable_length):
    """
    Compute the force exerted by a cable attached at motor_pos.
    If the payload is farther than cable_length, the cable applies a spring-like 
    force plus a damping force along its direction.
    """
    cable_vec = payload.pos - motor_pos
    distance = mag(cable_vec)
    if distance > cable_length:
        direction = norm(cable_vec)
        spring_force = -k * (distance - cable_length) * direction
        b = 20  # damping coefficient along the cable direction
        damping_force = -b * dot(payload.velocity, direction) * direction
        return spring_force + damping_force
    else:
        return vector(0, 0, 0)

dt = 0.01
in_collision = False  # tracks if collision was detected in the previous frame

while True:
    rate(100)
    
    # Update cable lengths based on key presses
    if keys_pressed["w"]:
        cable1_length = max(min_cable_length, cable1_length - cable_rate * dt)
    if keys_pressed["s"]:
        cable1_length = min(max_cable_length, cable1_length + cable_rate * dt)
    if keys_pressed["e"]:
        cable2_length = max(min_cable_length, cable2_length - cable_rate * dt)
    if keys_pressed["d"]:
        cable2_length = min(max_cable_length, cable2_length + cable_rate * dt)
    if keys_pressed["r"]:
        cable3_length = max(min_cable_length, cable3_length - cable_rate * dt)
    if keys_pressed["f"]:
        cable3_length = min(max_cable_length, cable3_length + cable_rate * dt)
    
    # Compute the total force on the payload from the three cables and gravity
    force = vector(0, 0, 0)
    force += compute_cable_force(motor1_pos, cable1_length)
    force += compute_cable_force(motor2_pos, cable2_length)
    force += compute_cable_force(motor3_pos, cable3_length)
    force += payload.mass * g  # add gravitational force

    # Update payload physics using Euler integration
    acceleration = force / payload.mass
    payload.velocity += acceleration * dt
    payload.velocity *= 0.98  # global damping to reduce oscillations
    payload.pos += payload.velocity * dt

    # Update the visual cable positions so each cable stretches from its motor to the payload
    cable1.axis = payload.pos - motor1_pos
    cable2.axis = payload.pos - motor2_pos
    cable3.axis = payload.pos - motor3_pos

    # Collision detection between the payload and the blue rectangle (target)
    # We check if the sphere (red payload) is intersecting the box (target)
    if (abs(payload.pos.x - target.pos.x) <= (target.size.x/2 + payload.radius) and
        abs(payload.pos.y - target.pos.y) <= (target.size.y/2 + payload.radius) and
        abs(payload.pos.z - target.pos.z) <= (target.size.z/2 + payload.radius)):
        if not in_collision:
            print("Red ball in contact with blue rectangle")
            in_collision = True
    else:
        in_collision = False
