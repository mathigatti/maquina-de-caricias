from vpython import mag, dot, norm, rate
from vpython import canvas, color, vector, box, cylinder, sphere

# ------------------------------
# Set up the 3D scene
# ------------------------------
scene = canvas(title="3D Three-Motor Suspension Simulation",
               width=800, height=600, center=vector(0,2,0),
               background=color.white)

# Floor and a visual target (blue rectangle for reference – not used in control now)
floor = box(pos=vector(0, -0.1, 0), size=vector(20, 0.2, 20), color=color.gray(0.5))
target_indicator = box(pos=vector(0, 0.05, 0), size=vector(0.25, 0.25, 0.25), color=color.blue)

# Ceiling height for the motors
ceiling_y = 5

# Define motor positions (forming an equilateral triangle on the ceiling)
motor1_pos = vector(-2, ceiling_y, 0)       
motor2_pos = vector(2, ceiling_y, 0)        
motor3_pos = vector(0, ceiling_y, 3.464)

# Visual representations for the motors
motor_size = vector(0.5, 0.2, 0.5)
motor1 = box(pos=motor1_pos, size=motor_size, color=color.gray(0.5))
motor2 = box(pos=motor2_pos, size=motor_size, color=color.gray(0.5))
motor3 = box(pos=motor3_pos, size=motor_size, color=color.gray(0.5))

# ------------------------------
# Initial cable and payload settings
# ------------------------------
cable1_length = 3.0
cable2_length = 3.0
cable3_length = 3.0

cable_rate = 0.5         # meters per second change for cable length adjustments
min_cable_length = 1.0
max_cable_length = 100.0

k = 100  # cable stiffness (spring constant)

# Visual representations for the cables
cable_radius = 0.05
cable1 = cylinder(pos=motor1_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)
cable2 = cylinder(pos=motor2_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)
cable3 = cylinder(pos=motor3_pos, axis=vector(0, -1, 0), radius=cable_radius, color=color.black)

# Create the payload (a red sphere)
payload = sphere(pos=vector(0, 1, 1), radius=0.2, color=color.red)
payload.mass = 0.5
payload.velocity = vector(0, 0, 0)

g = vector(0, -30, 0)  # gravitational acceleration

# ------------------------------
# Remove interactive key controls
# (The original key mapping and key event bindings have been removed)
# ------------------------------

# ------------------------------
# Define a list of target coordinates (waypoints)
# For example: 4 vertices of a square parallel to the floor, 20 cm above it.
# (You can adjust these coordinates to your needs.)
# ------------------------------

width = 0.5
x_offset, z_offset = 0, 1.5 
targets = [
    vector(1*width + x_offset, 0.3, 1*width + z_offset),
    vector(-1*width + x_offset, 0.3, 1*width + z_offset),
    vector(-1*width + x_offset, 0.3, -1*width + z_offset),
    vector(1*width + x_offset, 0.3, -1*width + z_offset)
] + [
    vector(1*width + x_offset, 0.0, 1*width + z_offset),
    vector(-1*width + x_offset, 0.0, 1*width + z_offset),
    vector(-1*width + x_offset, 0.0, -1*width + z_offset),
    vector(1*width + x_offset, 0.0, -1*width + z_offset)
]


target_index = 0
current_target = targets[target_index]
target_threshold = 0.3  # Distance threshold to decide when a target is reached

# Optionally, you could update the blue indicator to show the current target:
target_indicator.pos = current_target

# ------------------------------
# Define the cable force calculation as before
# ------------------------------
def compute_cable_force(motor_pos, cable_length):
    """
    Computes the force exerted by a cable at motor_pos.
    If the payload is farther than cable_length, the cable applies a spring and damping force.
    """
    cable_vec = payload.pos - motor_pos
    distance = mag(cable_vec) 
    if distance > cable_length:
        direction = norm(cable_vec)
        spring_force = -k * (distance - cable_length) * direction
        b = 20  # damping coefficient
        damping_force = -b * dot(payload.velocity, direction) * direction
        return spring_force + damping_force
    else:
        return vector(0, 0, 0)

# ------------------------------
# Main simulation loop
# ------------------------------
dt = 0.01

while True:
    rate(100)
    
    # Calculate the desired cable lengths from each motor to the current target.
    # We subtract a small offset (e.g., 0.1 m) from the distance in order to ensure 
    # that the cable stays taut and pulls the payload towards the target.
    
    # mag = Euclidean norm
    desired_length1 = mag(current_target - motor1_pos) - 0.01  
    desired_length2 = mag(current_target - motor2_pos) - 0.01
    desired_length3 = mag(current_target - motor3_pos) - 0.01
    
    # Ensure cable lengths stay within physical limits
    desired_length1 = max(min_cable_length, min(desired_length1, max_cable_length))
    desired_length2 = max(min_cable_length, min(desired_length2, max_cable_length))
    desired_length3 = max(min_cable_length, min(desired_length3, max_cable_length))
    
    # Function to gradually update a cable length toward its desired value
    def update_cable_length(current, desired):
        if current < desired:
            return min(current + cable_rate * dt, desired)
        elif current > desired:
            return max(current - cable_rate * dt, desired)
        else:
            return current
    
    cable1_length = update_cable_length(cable1_length, desired_length1)
    cable2_length = update_cable_length(cable2_length, desired_length2)
    cable3_length = update_cable_length(cable3_length, desired_length3)
    
    # Compute total force on the payload from the cables plus gravity
    force = vector(0, 0, 0)
    force += compute_cable_force(motor1_pos, cable1_length)
    force += compute_cable_force(motor2_pos, cable2_length)
    force += compute_cable_force(motor3_pos, cable3_length)
    force += payload.mass * g
    
    # Update payload dynamics using Euler integration
    acceleration = force / payload.mass
    payload.velocity += acceleration * dt
    payload.velocity *= 0.98  # damping to reduce oscillations
    payload.pos += payload.velocity * dt
    
    # Update the visual cables (so they run from the motor to the payload)
    cable1.axis = payload.pos - motor1_pos
    cable2.axis = payload.pos - motor2_pos
    cable3.axis = payload.pos - motor3_pos
    
    # Optionally, update the target indicator to follow the current target
    target_indicator.pos = current_target
    
    # Check if the payload is sufficiently close to the current target.
    if mag(payload.pos - current_target) < target_threshold:
        # If the current target is reached, cycle to the next target.
        target_index = (target_index + 1) % len(targets)
        current_target = targets[target_index]
        print("Reached target. Moving to next target:", current_target)
