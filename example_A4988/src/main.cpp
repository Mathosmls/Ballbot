
#include <ContinuousStepper.h>

ContinuousStepper<StepperDriver> stepperX;
ContinuousStepper<StepperDriver> stepperY;
ContinuousStepper<StepperDriver> stepperZ;

const byte stepPinX = 2;
const byte dirPinX = 5;

const byte stepPinY = 3;
const byte dirPinY = 6;

const byte stepPinZ = 4;
const byte dirPinZ = 7;

const byte enablePin = 8;

int cmd_speed=2000;
int cmd_speed2=500;
unsigned long acceleration=80000; 
// Pour le changement de direction
unsigned long previousMillis = 0;
const unsigned long interval = 500000; // 5 secondes


void setup() {
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Activer les moteurs

  stepperX.begin(stepPinX, dirPinX);
  stepperX.setAcceleration(acceleration);
  stepperX.spin(cmd_speed); // rotate at 200 steps per seconds
  stepperY.begin(stepPinY, dirPinY);
  stepperY.setAcceleration(acceleration);
  stepperY.spin(cmd_speed2); // rotate at 200 steps per seconds
  stepperZ.begin(stepPinZ, dirPinZ);
  stepperZ.setAcceleration(50000);
  stepperZ.spin(cmd_speed2); // rotate at 200 steps per seconds
}

void loop() {
  stepperX.loop(); // this function must be called as frequently as possible
  stepperY.loop();
  stepperZ.loop();
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    cmd_speed=-cmd_speed;
    stepperX.spin(cmd_speed);
    stepperY.spin(cmd_speed2);
    stepperZ.spin(cmd_speed2);
  }
}