#include "FastAccelStepper.h"
#include <Arduino.h>

const byte stepPins[3] = {6, 7, 8};
const byte dirPins[3] = {2, 3, 4};
// Déclaration d'un tableau de moteurs
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* steppers[3] = {NULL, NULL, NULL};
byte buffer[7];


long Acc_ref = 85000;
int cmd_motors[3] = {0, 0, 0};
int prev_cmd_motors[3] = {0, 0, 0};
long acc_motors[3] = {Acc_ref, Acc_ref, Acc_ref};

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

// Fonction pour initialiser tous les moteurs
void initStepper(FastAccelStepperEngine &engine,  FastAccelStepper* &stepper, byte stepPin, byte dirPin, unsigned long acceleration, int cmd_speed)
{
    stepper = engine.stepperConnectToPin(stepPin);
    if (stepper) {
        (stepper)->setDirectionPin(dirPin);
        (stepper)->setSpeedInHz(cmd_speed);
        (stepper)->setAcceleration(acceleration);
    }
}

void updateStepper(FastAccelStepper* &stepper, int speed_hz, long acceleration, int &prev_speed_hz)
{
  // if (stepper) { // simple version without the forceStop, may cause delay
  //   stepper->moveByAcceleration(acceleration, true);
  //   stepper->setSpeedInHz(abs(speed_hz));
  //   stepper->applySpeedAcceleration();
  // }

  if (stepper) {
    stepper->moveByAcceleration(acceleration, true);
    if (speed_hz==0 || sgn(prev_speed_hz)!=sgn(speed_hz))
    {
      stepper->forceStop();
    }
    else{
    stepper->setSpeedInHz(abs(speed_hz));
    }
  }
  prev_speed_hz=speed_hz;
}

void readCmd(int (&cmd_mot)[3],long (&acceleration)[3])
{
  if (Serial1.available() >= 7)
  {                       // Vérifie si 7 octets sont disponibles (1 pour le démarrage + 6 pour les entiers)
    // Lire les 7 octets dans le buffer
    Serial1.readBytes(buffer, 7);

    // Vérifier si le bit de démarrage est bien présent
    if (buffer[0] == 0x01)
    { // Si le premier octet est le bit de démarrage
      // Convertir les octets en entiers
      cmd_mot[0] = (buffer[1] << 8) | buffer[2]; // Reconstituer l'entier a
      cmd_mot[1] = (buffer[3] << 8) | buffer[4]; // Reconstituer l'entier b
      cmd_mot[2] = (buffer[5] << 8) | buffer[6]; // Reconstituer l'entier c
    }
    for (int i = 0; i < 3; i++)
    {
      acceleration[i]=(cmd_mot[i] >= 0) ? Acc_ref : -Acc_ref;
    }
  }
}





void setup() {
  Serial.begin(230400);  // Initialisation du port série pour afficher les valeurs
  Serial1.begin(230400);  // Initialisation du port série 1 à 230400 bauds

  // Initialisation moteur
  engine.init();
  for (int i = 0; i < 3; i++)
  {
    initStepper(engine,steppers[i], stepPins[i], dirPins[i], Acc_ref, cmd_motors[i]);
  }
  Serial.println("step motors setup ended");
}

void loop() {
  readCmd(cmd_motors,acc_motors);
  for (int i = 0; i < 3; i++)
  {
    updateStepper(steppers[i], cmd_motors[i], acc_motors[i],prev_cmd_motors[i]);
  }
}