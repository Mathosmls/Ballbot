#include <SimpleFOC.h>
#include <PciManager.h>
#include <PciListenerImp.h>
#include "imu.hpp"
#include "pid.hpp"          
 #include "CytronMotorDriver.h"
 #include <Arduino.h>
//---------------------------------
//ENCODEURS
//---------------------------------
const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(6, 7, 5000),
    Encoder(2, 3, 5000),
    Encoder(4, 5, 5000)
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

IMU myIMU(500,2);

//----------------------------------------
//PID
//----------------------------------------
double Kp_pitch = 2.0, Ki_pitch = 0.0, Kd_pitch = 0.0;
double Kp_roll = -2.0, Ki_roll = 0.0, Kd_roll = 0.0;
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);


void computePID()
{
    roll = myIMU.get_roll_rad();
    pitch = myIMU.get_pitch_rad();
    pid_pitch.Compute();
    pid_roll.Compute();
    vx = constrain(vx, -2, 2);  
    vy = constrain(vy, -2, 2); 
}

//----------------------------------------
//Motors
//----------------------------------------

CytronMD motor1(PWM_DIR, 10, 12);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 9, 8); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motor3(PWM_DIR, 11, 13); // PWM 2 = Pin 9, DIR 2 = Pin 10.

//---------------------------------------------------------------------------


void setup() {
    Serial.begin(115200);

//---------------------------------
//ENCODEURS
//---------------------------------
    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoders[i].quadrature = Quadrature::ON;
        encoders[i].pullup = Pullup::USE_EXTERN;
        encoders[i].init();

        // Associer les interruptions via PciListeners
        listenersA[i] = new PciListenerImp(encoders[i].pinA, doA[i]);
        listenersB[i] = new PciListenerImp(encoders[i].pinB, doB[i]);
        PciManager.registerListener(listenersA[i]);
        PciManager.registerListener(listenersB[i]);
    }
    Serial.println("Encoders ready");
//----------------------------------------
//IMU
//----------------------------------------
    if (!myIMU.init()) {
            Serial.println("Échec de l'initialisation de l'IMU.");
        }

        myIMU.calibrate();
}

void loop() {
    static int i = 0;
    // Serial.print("hhhhhhhhhhhhh");
    updateEncoders();
    myIMU.update();
    computePID();
    motor1.setSpeed(50);   // Motor 1 runs forward at 50% speed.
    motor2.setSpeed(50);   // Motor 1 runs forward at 50% speed.
    motor3.setSpeed(50);   // Motor 1 runs forward at 50% speed.
    Serial.println("ioesfbisno");
    // delay(10000);
    if (i == 1) {
        // printEncoderInfo();
        // myIMU.printAll();
        myIMU.printAngle();
        Serial.print("PID vx | vy: ");
        Serial.print(vx);
        Serial.print("\t");
        Serial.println(vy);
        i = 0;
    } else {
        i++;
    }
}

// /*******************************************************************************
//  * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
//  * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
//  * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
//  * DAMAGES, FOR ANY REASON WHATSOEVER.
//  ********************************************************************************
//  * DESCRIPTION:
//  *
//  * This example shows how to drive 2 motors using the PWM and DIR pins with
//  * 2-channel motor driver.
//  * 
//  * 
//  * CONNECTIONS:
//  * 
//  * Arduino D3  - Motor Driver PWM 1 Input
//  * Arduino D4  - Motor Driver DIR 1 Input
//  * Arduino D9  - Motor Driver PWM 2 Input
//  * Arduino D10 - Motor Driver DIR 2 Input
//  * Arduino GND - Motor Driver GND
//  *
//  *
//  * AUTHOR   : Kong Wai Weng
//  * COMPANY  : Cytron Technologies Sdn Bhd
//  * WEBSITE  : www.cytron.io
//  * EMAIL    : support@cytron.io
//  *
//  *******************************************************************************/

// #include <SimpleFOC.h>
// #include <PciManager.h>
// #include <PciListenerImp.h>
// #include "imu.hpp"
// #include "pid.hpp"          
// #include "CytronMotorDriver.h"

// // Configure the motor driver.
// CytronMD motor1(PWM_DIR, 10 ,12);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
// CytronMD motor2(PWM_DIR, 9, 8); // PWM 2 = Pin 9, DIR 2 = Pin 10.
// CytronMD motor3(PWM_DIR, 11, 13); // PWM 2 = Pin 9, DIR 2 = Pin 10.

// // The setup routine runs once when you press reset.
// void setup() {
//   Serial.begin(115200);
// }


// // The loop routine runs over and over again forever.
// void loop() {
//   motor1.setSpeed(50);   // Motor 1 runs forward at 50% speed.
// //   motor2.setSpeed(-50);  // Motor 2 runs backward at 50% speed.
// //   motor3.setSpeed(50);  // Motor 2 runs backward at 50% speed.
//   Serial.println("first");
//   delay(5000);
  
//   motor1.setSpeed(100);   // Motor 1 runs forward at full speed.
// //   motor2.setSpeed(-100);  // Motor 2 runs backward at full speed.
// //   motor3.setSpeed(100);  // Motor 2 runs backward at 50% speed.
//   Serial.println("second");
//   delay(5000);

//   motor1.setSpeed(0);     // Motor 1 stops.
// //   motor2.setSpeed(0);     // Motor 2 stops.
// //   motor3.setSpeed(0);  // Motor 2 runs backward at 50% speed.
//   Serial.println("third");
//   delay(5000);

//   motor1.setSpeed(-50);  // Motor 1 runs backward at 50% speed.
// //   motor2.setSpeed(50);   // Motor 2 runs forward at 50% speed.
// //   motor3.setSpeed(50);  // Motor 2 runs backward at 50% speed.
//   Serial.println("fourth");
//   delay(5000);
  
//   motor1.setSpeed(-100);  // Motor 1 runs backward at full speed.
// //   motor2.setSpeed(100);   // Motor 2 runs forward at full speed.
// //   motor3.setSpeed(100);  // Motor 2 runs backward at 50% speed.
//   Serial.println("fifth");
//   delay(5000);

//   motor1.setSpeed(0);     // Motor 1 stops.
//   motor2.setSpeed(0);     // Motor 2 stops.
//   motor3.setSpeed(0);  // Motor 2 runs backward at 50% speed.
//   Serial.println("last");
//   delay(5000);
// }
