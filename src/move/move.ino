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

int numberOfSteps = 400;
byte ledPin = 13;
int pulseWidthMicros = 20;    // microseconds
int millisbetweenSteps = 20;  // milliseconds - or try 1000 for slower steps

int vueltas = 4;
int steps = 200;
int microsteps = 32;

// Global variables
double x_total_cm = 180;
double z_total_cm = 230;
double y_total_cm = 280;

double current_x = x_total_cm / 2.;
double current_y = 0;
double current_z = 80;

// Valores de inicio
double largo_motor_1 = 120.4;
double largo_motor_2 = 120.4;
double largo_motor_3 = 150.0;

// Struct for 3D coordinates
struct Coordinate3D {
  double x;
  double y;
  double z;
};

void debug() {
  Serial.println("Largo_motor");
  Serial.println(largo_motor_1);
  Serial.println(largo_motor_2);
  Serial.println(largo_motor_3);

  Serial.println("Current coords");
  Serial.println(current_x);
  Serial.println(current_y);
  Serial.println(current_z);
}

void update_steps(Coordinate3D coord) {
 update_steps(coord.x, coord.y, coord.z);
}

void update_steps(double x, double y, double z) {
  double largo_motor_1_viejo = largo_motor_1;
  double largo_motor_2_viejo = largo_motor_2;
  double largo_motor_3_viejo = largo_motor_3;

  largo_motor_1 = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  largo_motor_2 = sqrt(pow(x_total_cm - x, 2) + pow(y, 2) + pow(z, 2));
  largo_motor_3 = sqrt(pow(x_total_cm / 2 - x, 2) + pow(y, 2) + pow(z_total_cm - z, 2));

  double diferencia_motor_1 = largo_motor_1 - largo_motor_1_viejo;
  double diferencia_motor_2 = largo_motor_2 - largo_motor_2_viejo;
  double diferencia_motor_3 = largo_motor_3 - largo_motor_3_viejo;

  int steps_motor_1 = int(cm_a_steps * diferencia_motor_1);
  int steps_motor_2 = int(cm_a_steps * diferencia_motor_2);
  int steps_motor_3 = int(cm_a_steps * diferencia_motor_3);

  motorZ.move(steps_motor_1);
  motorY.move(steps_motor_2);
  motorX.move(steps_motor_3);

  current_x = x;
  current_y = y;
  current_z = z;
}

double movement_length = 10.;

void handle_serial_command(char command) {
  debug();
  switch (command) {
    case 'V': // Setea cero de coordenadas
      current_x = x_total_cm / 2.;
      current_y = 0;
      current_z = 80.0;
      largo_motor_1 = 120.4;
      largo_motor_2 = 120.4;
      largo_motor_3 = 150.;
      break;
    case 'B': // Va al cero de coordenadas
      current_x = x_total_cm / 2.;
      current_y = 0;
      current_z = 80.0;
      update_steps(current_x, current_y, current_z);
      break;
    case 'C': // Falopa
      current_x = x_total_cm / 2.;
      current_y = 210;
      current_z = 80.0;
      largo_motor_1 = 242.0;
      largo_motor_2 = 242.0;
      largo_motor_3 = 258.0;
      break;
    case 'N': // Moverse mas
      movement_length += 1;
      if (movement_length > 100)
        movement_length = 100;
      break;
    case 'M': // Moverse menos
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

    case 'A': // Left
      update_steps(current_x, current_y, current_z + movement_length);
      break;
    case 'D': // Right
      update_steps(current_x, current_y, current_z - movement_length);
      break;
    case 'W': // Up
      update_steps(current_x, current_y - movement_length, current_z);
      break;
    case 'S': // Down
      update_steps(current_x, current_y + movement_length, current_z);
      break;
    case 'R': // Front
      update_steps(current_x + movement_length, current_y, current_z);
      break;
    case 'F': // Back
      update_steps(current_x - movement_length, current_y, current_z);
      break;
    default:
      Serial.println("Invalid command");
  }
  debug();
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

  int speed = int(steps * microsteps * 20);
  //speed = 30000;
  
  motorY.setMaxSpeed(speed * 2);
  motorY.setSpeed(speed);
  motorY.setAcceleration(speed * 4);

  motorX.setMaxSpeed(speed * 2);
  motorX.setSpeed(speed);
  motorX.setAcceleration(speed * 4);

  motorZ.setMaxSpeed(speed * 2);
  motorZ.setSpeed(speed);
  motorZ.setAcceleration(speed * 4);


}

Coordinate3D parse_coordinates(String message) {
  // Assuming message is in the format "x,y,z"
  Coordinate3D coord;
  int firstCommaIndex = message.indexOf(',');
  int secondCommaIndex = message.indexOf(',', firstCommaIndex + 1);

  if (firstCommaIndex == -1 || secondCommaIndex == -1) {
    Serial.println("Invalid coordinate format");
    return coord;
  }

  String xString = message.substring(0, firstCommaIndex);
  String yString = message.substring(firstCommaIndex + 1, secondCommaIndex);
  String zString = message.substring(secondCommaIndex + 1);

  coord.x = xString.toFloat();
  coord.y = yString.toFloat();
  coord.z = zString.toFloat();

  // Now print the coordinates
  Serial.print("Parsed coordinates: (");
  Serial.print(coord.x);
  Serial.print(", ");
  Serial.print(coord.y);
  Serial.print(", ");
  Serial.print(coord.z);
  Serial.println(")");
  return coord;
}

void loop() {
  motorX.run();
  motorY.run();
  motorZ.run();

  while (Serial.available()) {
    char command = Serial.read();
    if (command == '(') {
      // Start reading the message
      String message = "";
      unsigned long startTime = millis(); // Get the current time
      const unsigned long timeout = 1000; // Timeout duration in milliseconds (1 second)

      while (true) {
        // Check for timeout
        if (millis() - startTime > timeout) {
          Serial.println("Timeout while reading message");
          break; // Exit the loop if timeout occurs
        }
        // Wait until data is available
        if (Serial.available()) {
          char c = Serial.read();
          if (c == ')') {
            // End of message
            break;
          }
          message += c;
        }
      }
      // Now print the full message
      Serial.println("Received message: " + message);

      // Parse the coordinates
      Coordinate3D coord = parse_coordinates(message);
      update_steps(coord);

    } else {
      handle_serial_command(command);
    }
  }
}
