int pin [4] = {8, 9, 10, 11};
int pinBtnLectura [2] = {2, 3};
int quePin = 0;
int direccion = 1;
int vel = 1;

void setup() {
  Serial.begin(9600);
  pinMode (pin [0], OUTPUT);
  pinMode (pin [1], OUTPUT);
  pinMode (pin [2], OUTPUT);
  pinMode (pin [3], OUTPUT);
  pinMode (pinBtnLectura [0], INPUT_PULLUP);
  pinMode (pinBtnLectura [1], INPUT_PULLUP);
}

void loop() {
  //aca iria un swicht on off   if(digitalRead(pinBtnLectura [0]))
  if (true) {
    Serial.print(" - ");
    Serial.println(quePin);
    controlMotor();
    vel = 2; // 2 a lo que sea

    if (digitalRead(pinBtnLectura[0]))direccion = !direccion;
    if (digitalRead(pinBtnLectura[1]))direccion = !direccion;
    Serial.print(" - btn a : ");
    Serial.println(digitalRead(pinBtnLectura [0]));
    Serial.print(" - btn b : ");
    Serial.println(digitalRead(pinBtnLectura [1]));
  }
}

void controlMotor() {
  digitalWrite(pin[quePin], LOW);
  delay(vel);
  if ( direccion) quePin++;
  else quePin--;

  if (quePin > 3) quePin = 0;
  if (quePin < 0) quePin = 3;

  digitalWrite(pin[quePin], HIGH);
  delay(vel);

}
