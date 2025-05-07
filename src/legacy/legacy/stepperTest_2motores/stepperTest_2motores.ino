#include <AccelStepper.h>

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19


int numberOfSteps = 400;
byte ledPin = 13;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 20; // milliseconds - or try 1000 for slower steps

AccelStepper motorY(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper motorZ(1, Z_STEP_PIN, Z_DIR_PIN);

void setup() {

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);


  pinMode(Y_STEP_PIN  , OUTPUT);
  pinMode(Y_DIR_PIN    , OUTPUT);
  pinMode(Y_ENABLE_PIN    , OUTPUT);

  pinMode(Z_STEP_PIN  , OUTPUT);
  pinMode(Z_DIR_PIN    , OUTPUT);
  pinMode(Z_ENABLE_PIN    , OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);


  motorY.setMaxSpeed(2000);
  motorY.setSpeed(8000);
  motorY.setAcceleration(2000);

  motorZ.setMaxSpeed(2000);
  motorZ.setSpeed(8000);
  motorZ.setAcceleration(2000);
  motorY.moveTo(1000);
  motorZ.moveTo(1000);

}

void loop() {




  motorY.run();
  motorZ.run();

  if (motorZ.distanceToGo() == 0) {
    motorZ.moveTo(
      x1000);

  }
  if (motorY.distanceToGo() == 0) {
    motorY.moveTo(-1000);

  }
}
