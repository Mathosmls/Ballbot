#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>
#include "imu.hpp"
#include "pid.hpp"          
#include "CytronMotorDriver.h"
#include <Arduino.h>
#include <math.h>

//---------------------------------
//ENCODEURS
//---------------------------------
const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(2, 3, 5000),
    Encoder(18, 19, 5000),
    Encoder(20, 21, 5000)
};

void doA0() { encoders[0].handleA(); }
void doB0() { encoders[0].handleB(); }
void doA1() { encoders[1].handleA(); }
void doB1() { encoders[1].handleB(); }
void doA2() { encoders[2].handleA(); }
void doB2() { encoders[2].handleB(); }

void (*doA[NUM_ENCODERS])() = { doA0, doA1, doA2 };
void (*doB[NUM_ENCODERS])() = { doB0, doB1, doB2 };

PciListenerImp* listenersA[NUM_ENCODERS];
PciListenerImp* listenersB[NUM_ENCODERS];

void updateEncoders() {
    for (int j = 0; j < NUM_ENCODERS; j++) {
        encoders[j].update(); // Met à jour les données de l'encodeur
    }
}
void printEncoderInfo() {
    for (int j = 0; j < NUM_ENCODERS; j++) {
        Serial.print("enc ");
        Serial.print(j + 1);
        Serial.print(" : ");
        Serial.print(encoders[j].getAngle());
        Serial.print("\t");
        Serial.println(encoders[j].getVelocity());
    }
}

//----------------------------------------
//IMU
//----------------------------------------
IMU myIMU(1000,4);

//----------------------------------------
//PID simple
//----------------------------------------
double Kp = 1000.0, Ki = 0.0, Kd = 100.0;
double pi = 3.14159265358979;
double setpoint = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, psi, alpha, v; // Entrée et sortie de la boucle interne
MyPID pid_alpha(Kp, Ki, Kd, &alpha, &v, &setpoint);
bool boost[3] = {false, false, false};

double get_motor_speed(int numero, double psi, double v) 
{
  double v_motor = constrain(abs(v)*sin(psi + (1 - numero)*(2./3.)*pi), -255, 255);
  if (abs(v_motor) < 30 && !boost[numero]) {
    boost[numero] = true;
    return 150*v_motor/abs(v_motor);
  }
  else if (abs(v_motor) < 30 && boost[numero]) {
    return 30*v_motor/abs(v_motor);
  }
  else {
    boost[numero] = false;
    return v_motor;
  }
}

void computePID()
{
    roll = myIMU.get_roll_rad();
    pitch = myIMU.get_pitch_rad();
    alpha = asin(sqrt(pow(sin(roll), 2) + pow(sin(pitch), 2)));
    pid_alpha.Compute();
}

//----------------------------------------
//Motors
//----------------------------------------
CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 8, 9); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 7, 11); // PWM 3 = Pin 7, DIR 3 = Pin 11.

//---------------------------------------------------------------------------


void setup() {
    Serial.begin(115200);
    Serial.println("Setup");
//---------------------------------
//ENCODEURS
//---------------------------------
    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoders[i].quadrature = Quadrature::ON;
        encoders[i].pullup = Pullup::USE_EXTERN;
        encoders[i].init();
        encoders[i].enableInterrupts(doA[i], doB[i]);
        // Associer les interruptions via PciListeners
        // listenersA[i] = new PciListenerImp(encoders[i].pinA, doA[i]);
        // listenersB[i] = new PciListenerImp(encoders[i].pinB, doB[i]);
        // PciManager.registerListener(listenersA[i]);
        // PciManager.registerListener(listenersB[i]);
    }
    Serial.println("Encoders ready");
//----------------------------------------
//IMU
//----------------------------------------
    // if (!myIMU.init()) {
    //         Serial.println("Échec de l'initialisation de l'IMU.");
    //     }
    // else 
    // {
    //     Serial.println("IMU ready");
    // }

        // myIMU.calibrate();
}

void loop() {
    static int i = 0;
    // Serial.print("hhhhhhhhhhhhh");
    // updateEncoders();
    myIMU.update();
    // // computePID();
    // motor1.setSpeed(200);   // Motor 1 runs forward at 50% speed.
    // motor2.setSpeed(200);   // Motor 1 runs forward at 50% speed.
    // motor3.setSpeed(200);   // Motor 1 runs forward at 50% speed.
    Serial.println("boucle");
    // delay(10000);
    psi = atan2(sin(roll), sin(pitch));
    motor1.setSpeed(get_motor_speed(1, psi, v));
    motor2.setSpeed(get_motor_speed(2, psi, v));
    motor3.setSpeed(get_motor_speed(3, psi, v));

    if (i == 400) {
        // printEncoderInfo();
        myIMU.printAll();
        Serial.println("==================================================");
        Serial.print("PID : V = ");
        Serial.print(abs(v));
        Serial.print(", psi = ");
        Serial.print(psi);
        Serial.print(", alpha = ");
        Serial.println(alpha);

        Serial.println("Vitesses moteurs : ");
        Serial.print("v1 = ");
        Serial.println(get_motor_speed(1, psi, v));
        Serial.print("v2 = ");
        Serial.println(get_motor_speed(2, psi, v));
        Serial.print("v3 = ");
        Serial.println(get_motor_speed(3, psi, v));
        Serial.println("==================================================");
        i = 0;
    } else {
        i++;
    }
    delay(10);
}














