#include <AccelStepper.h>

#define X_STEP_PIN A0
#define X_DIR_PIN A1
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 46
#define Y_DIR_PIN 48
#define Y_ENABLE_PIN A8

#define Z_STEP_PIN A6
#define Z_DIR_PIN A7
#define Z_ENABLE_PIN A2

#define X_MIN 3
#define Y_MIN 18
#define Z_MIN 14

double diametro_cm = 5;
double perimetro = PI * diametro_cm;
int steps_por_vuelta = 6400;
double cm_a_steps = steps_por_vuelta / perimetro;

AccelStepper motorX(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper motorY(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorZ(1, Z_STEP_PIN, Z_DIR_PIN);

byte ledPin = 13;
int pulseWidthMicros = 20;
int millisbetweenSteps = 20;
int steps = 200;
int microsteps = 32;

struct Coordinate4D {
  double x;
  double y;
  double z;
  double speed; // temporary motor speed for this movement
};

double movement_length = 10.0;
int defaultSpeed;  // computed in setup()

// Handles non-coordinate serial commands (e.g. single characters)
void handle_serial_command(char command) {
  switch (command) {
    case 'N': // Increase movement length
      movement_length += 1;
      if (movement_length > 100)
        movement_length = 100;
      break;
    case 'M': // Decrease movement length
      movement_length -= 1;
      if (movement_length < 1)
        movement_length = 1;
      break;
    case 'Y': // Motor X Up
      motorX.move(cm_a_steps * -movement_length);
      break;
    case 'H': // Motor X Down
      motorX.move(cm_a_steps * movement_length);
      break;
    case 'U': // Motor Y Up
      motorY.move(cm_a_steps * -movement_length);
      break;
    case 'J': // Motor Y Down
      motorY.move(cm_a_steps * movement_length);
      break;
    case 'I': // Motor Z Up
      motorZ.move(cm_a_steps * -movement_length);
      break;
    case 'K': // Motor Z Down
      motorZ.move(cm_a_steps * movement_length);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);

  pinMode(X_MIN, INPUT);
  pinMode(Y_MIN, INPUT);
  pinMode(Z_MIN, INPUT);

  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  pinMode(ledPin, OUTPUT);

  // Calculate and set default motor speeds and related parameters
  defaultSpeed = int(steps * microsteps * 20);
  
  motorX.setMaxSpeed(defaultSpeed * 2);
  motorX.setSpeed(defaultSpeed);
  motorX.setAcceleration(defaultSpeed * 4);

  motorY.setMaxSpeed(defaultSpeed * 2);
  motorY.setSpeed(defaultSpeed);
  motorY.setAcceleration(defaultSpeed * 4);

  motorZ.setMaxSpeed(defaultSpeed * 2);
  motorZ.setSpeed(defaultSpeed);
  motorZ.setAcceleration(defaultSpeed * 4);
}

//
// Parse a coordinate message assumed to be in the format "x,y,z,s"
// where s is the desired temporary motor speed.
//
Coordinate4D parse_coordinates(String message) {
  Coordinate4D coord;
  int firstComma = message.indexOf(',');
  int secondComma = message.indexOf(',', firstComma + 1);
  int thirdComma = message.indexOf(',', secondComma + 1);
  
  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
    Serial.println("Invalid coordinate format");
    coord.x = 0;
    coord.y = 0;
    coord.z = 0;
    coord.speed = defaultSpeed;
    return coord;
  }
  
  String xStr = message.substring(0, firstComma);
  String yStr = message.substring(firstComma + 1, secondComma);
  String zStr = message.substring(secondComma + 1, thirdComma);
  String sStr = message.substring(thirdComma + 1);
  
  coord.x = xStr.toFloat();
  coord.y = yStr.toFloat();
  coord.z = zStr.toFloat();
  coord.speed = sStr.toFloat();
  
  return coord;
}

//
// Execute a movement based on the parsed coordinates.
// Temporarily sets the motor speeds, maxSpeed, and acceleration to new values
// based on coord.speed during the move, then restores the original values,
// and finally prints "OK".
//
void execute_movement(Coordinate4D coord) {
  // Save old speed parameters (assumed same for all motors)
  int oldSpeed = defaultSpeed;
  
  // Calculate new speed from coordinate (if provided)
  int newSpeed = (coord.speed > 0) ? int(coord.speed) : oldSpeed;

  // Update speed, maxSpeed, and acceleration for each motor
  motorX.setSpeed(newSpeed);
  motorY.setSpeed(newSpeed);
  motorZ.setSpeed(newSpeed);
  
  motorX.setMaxSpeed(newSpeed * 2);
  motorX.setAcceleration(newSpeed * 4);
  motorY.setMaxSpeed(newSpeed * 2);
  motorY.setAcceleration(newSpeed * 4);
  motorZ.setMaxSpeed(newSpeed * 2);
  motorZ.setAcceleration(newSpeed * 4);
  
  // Command the movement
  motorX.move(cm_a_steps * coord.x);
  motorY.move(cm_a_steps * coord.y);
  motorZ.move(cm_a_steps * coord.z);
  
  // Block until all motors complete their movement
  while (motorX.distanceToGo() != 0 || motorY.distanceToGo() != 0 || motorZ.distanceToGo() != 0) {
    motorX.run();
    motorY.run();
    motorZ.run();
  }
  
  // Restore original speed, maxSpeed, and acceleration
  motorX.setSpeed(oldSpeed);
  motorY.setSpeed(oldSpeed);
  motorZ.setSpeed(oldSpeed);
  
  motorX.setMaxSpeed(oldSpeed * 2);
  motorX.setAcceleration(oldSpeed * 4);
  motorY.setMaxSpeed(oldSpeed * 2);
  motorY.setAcceleration(oldSpeed * 4);
  motorZ.setMaxSpeed(oldSpeed * 2);
  motorZ.setAcceleration(oldSpeed * 4);
  
  // Signal completion
  Serial.println("OK");
}

void loop() {
  // Run motors continuously
  motorX.run();
  motorY.run();
  motorZ.run();

  while (Serial.available()) {
    char command = Serial.read();
    if (command == '(') {
      // Begin reading the coordinate message
      String message = "";
      unsigned long startTime = millis();
      const unsigned long timeout = 1000; // 1 second timeout
      while (true) {
        if (millis() - startTime > timeout) {
          Serial.println("Timeout while reading message");
          break;
        }
        if (Serial.available()) {
          char c = Serial.read();
          if (c == ')') {
            break;
          }
          message += c;
        }
      }
      Coordinate4D coord = parse_coordinates(message);
      execute_movement(coord);
    } else {
      handle_serial_command(command);
    }
  }
}

