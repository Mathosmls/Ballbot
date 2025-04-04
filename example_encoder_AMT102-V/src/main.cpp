#include <Arduino.h>
#include <SimpleFOC.h>
#include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor1(PWM_DIR, 12 ,10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 7, DIR 2 = Pin 11.



const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(2, 3, 500),
    Encoder(5, 6, 500),
    Encoder(30, 31, 192)};

// Fonctions d'interruption pour chaque encodeur
void doA0() { encoders[0].handleA(); }
void doB0() { encoders[0].handleB(); }
void doA1() { encoders[1].handleA(); }
void doB1() { encoders[1].handleB(); }
void doA2() { encoders[2].handleA(); }
void doB2() { encoders[2].handleB(); }

// Tableau de pointeurs vers les fonctions d'interruption
void (*doA[NUM_ENCODERS])() = {doA0, doA1, doA2};
void (*doB[NUM_ENCODERS])() = {doB0, doB1, doB2};

void updateEncoders()
{
  for (int j = 0; j < NUM_ENCODERS; j++)
  {
    encoders[j].update(); // Met à jour les données de l'encodeur
  }
}

void printEncoderInfo()
{
  for (int j = 0; j < NUM_ENCODERS; j++)
  {
    Serial.print("enc ");
    Serial.print(j + 1);
    Serial.print(" : ");
    Serial.print(encoders[j].getAngle());
    Serial.print("\t");
    Serial.println(encoders[j].getVelocity());
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Test Encoder AMT102-V");
  Serial.println("Setup");

  for (int i = 0; i < NUM_ENCODERS; i++) {
    encoders[i].quadrature = Quadrature::ON;
    encoders[i].pullup = Pullup::USE_EXTERN;
    encoders[i].init();

    // Attacher les interruptions matérielles
    attachInterrupt(digitalPinToInterrupt(encoders[i].pinA), doA[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoders[i].pinB), doB[i], CHANGE);
  }
  Serial.println("Encoders ready");
}

void loop()
{
  motor1.setSpeed(0);   // -225 to 255.
  motor2.setSpeed(50);  
  motor3.setSpeed(100); 
  static int i = 0;
  updateEncoders();
  if (i == 50)
  {
    printEncoderInfo();
    i = 0;
  }
  else
  {
    i++;
  }
  delay(3);
}