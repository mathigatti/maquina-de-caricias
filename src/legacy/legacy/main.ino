#include <AccelStepper.h>

#define Y_STEP_PIN 2
#define Y_DIR_PIN 3
#define Y_ENABLE_PIN 56
#define BTN 8

int numberOfSteps = 400;
byte ledPin = 13;
int pulseWidthMicros = 20;    // microseconds
int millisbetweenSteps = 20;  // milliseconds - or try 1000 for slower steps


int vueltas = 4;
int steps = 200;
int microsteps = 32;

AccelStepper motorY(1, Y_STEP_PIN, Y_DIR_PIN);

void setup() {

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(ledPin, OUTPUT);
  /*
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
*/
  int speed = int(steps*microsteps);
  motorY.setMaxSpeed(speed*2);
  motorY.setSpeed(speed);
  motorY.setAcceleration(speed*4);
  /*
  motorZ.setMaxSpeed(2000);
  motorZ.setSpeed(8000);
 // motorZ.setAcceleration(2000);
 */
  //motorY.moveTo(1000);
  //  motorZ.moveTo(1000);
}

void loop() {
  int max_position = steps*microsteps*vueltas;
  bool btn = !digitalRead(BTN);
  motorY.run();
  if (btn) {
    if (motorY.currentPosition() == 0) {
      motorY.moveTo(-max_position);

    } else if (motorY.currentPosition() == -max_position) {
      motorY.moveTo(0);
    }
  }
}
