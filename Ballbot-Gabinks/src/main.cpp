#include "imu.hpp"
#include "pid.hpp"          
#include "CytronMotorDriver.h"
#include <SimpleFOC.h>
#include <Arduino.h>
#include <math.h>

//----------------------------------------

#pragma region "IMU"

IMU myIMU(1000,4);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"

double Kp_pitch = 1700.0, Ki_pitch = 0.0, Kd_pitch = 100.0;
double Kp_roll = 1700.0, Ki_roll = 0.0, Kd_roll = 100.0;
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
double pi = 3.14159265358979;

// Voir fonction compute_PID()

double Kp_max = 2500., Kp_min = 1500.;
double alpha_max = pi/45, alpha_dot_max = 1.; // alpha_max (rad), alpha_dot_max(rad/s)
float lambda = 0.5; // Permet de prendre plus ou moins en compte la vitesse de rotation / l'angle (lambda est compris entre 0 et 1)

void fuzzy() {
    double newKp_pitch = Kp_max*(1./2.)*(lambda*(pitch/alpha_max) + (1 - lambda)*(pid_pitch.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    double newKp_roll = Kp_max*(1./2.)*(lambda*(roll/alpha_max) + (1 - lambda)*(pid_roll.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    pid_pitch.SetKp(constrain(newKp_pitch, Kp_min, Kp_max));
    pid_roll.SetKp(constrain(newKp_roll, Kp_min, Kp_max));
}

#pragma endregion

//----------------------------------------

#pragma region "Moteurs"

CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.

double motor1_speed, motor2_speed, motor3_speed;

double get_motor_speed(int numero, double vx, double vy) {
  double angle = (1 - numero)*(2./3.)*pi + pi;
  double v_motor = vx*sin(angle) + vy*cos(angle);
  return constrain(v_motor, -255, 255);
}

#pragma endregion

//----------------------------------------

#pragma region "Encodeurs + contrôle de la vitesse de rotation en boucle fermée"

const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(2, 3, 192),
    Encoder(5, 6, 192),
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

double Kp_odo = 3.0, Ki_odo = 0.0, Kd_odo = 0.0;
double setpoint_odo1, setpoint_odo2, setpoint_odo3;
double odo1, odo2, odo3;
double correction_odo1, correction_odo2, correction_odo3;
MyPID pid_odo1(Kp_odo, Ki_odo, Kd_odo, &odo1, &correction_odo1, &setpoint_odo1);
MyPID pid_odo2(Kp_odo, Ki_odo, Kd_odo, &odo2, &correction_odo2, &setpoint_odo2);
MyPID pid_odo3(Kp_odo, Ki_odo, Kd_odo, &odo3, &correction_odo3, &setpoint_odo3);

#pragma endregion


//----------------------------------------
//Core program
//----------------------------------------
void print_data() {
    myIMU.printAll();
    printEncoderInfo();
    Serial.println("==================================================");
    Serial.println("PID : ");
    Serial.print("Vx = ");
    Serial.print(vx);
    Serial.print("\tVy = ");
    Serial.println(vy);
    Serial.print("Kp_pitch = ");
    Serial.println(pid_pitch.GetKp());
    Serial.print("Kp_roll = ");
    Serial.println(pid_roll.GetKp());

    Serial.println("Vitesses moteurs : ");
    Serial.print("v1 = ");
    Serial.println(motor1_speed);
    Serial.print("v2 = ");
    Serial.println(motor2_speed);
    Serial.print("v3 = ");
    Serial.println(motor3_speed);

    Serial.println("Corrections odomètres : ");
    Serial.print("c1 = ");
    Serial.println(correction_odo1);
    Serial.print("c2 = ");
    Serial.println(correction_odo2);
    Serial.print("c3 = ");
    Serial.println(correction_odo3);
    Serial.println("==================================================");
}

void computePID() {
  pitch = myIMU.get_pitch_rad();
  roll = myIMU.get_roll_rad();
  pid_pitch.Compute();
  pid_roll.Compute();

  // motor1_speed = get_motor_speed(1, vx, vy);
  // motor2_speed = get_motor_speed(2, vx, vy);
  // motor3_speed = get_motor_speed(3, vx, vy);

  motor1_speed = 0;
  motor2_speed = 0;
  motor3_speed = 100;

  odo1 = encoders[0].getVelocity();
  odo2 = encoders[1].getVelocity();
  odo3 = encoders[2].getVelocity();
  setpoint_odo1 = (-16./255.)*motor1_speed;
  setpoint_odo2 = (-16./255.)*motor2_speed;
  setpoint_odo3 = (-16./255.)*motor3_speed;
  pid_odo1.Compute();
  pid_odo2.Compute();
  pid_odo3.Compute();
}

void setup() {
    Serial.begin(115200);

    if (!myIMU.init()) {
        Serial.println("Échec de l'initialisation de l'IMU.");
    }
    myIMU.calibrate();

    for (int i = 0; i < NUM_ENCODERS; i++) {
        encoders[i].quadrature = Quadrature::ON;
        encoders[i].pullup = Pullup::USE_EXTERN;
        encoders[i].init();

        // Attacher les interruptions matérielles
        attachInterrupt(digitalPinToInterrupt(encoders[i].pinA), doA[i], CHANGE);
        attachInterrupt(digitalPinToInterrupt(encoders[i].pinB), doB[i], CHANGE);
    }
}

// void loop() {
//     static int i = 0;

//     myIMU.update();
//     updateEncoders();
//     fuzzy();
//     computePID();

//     motor1.setSpeed(motor1_speed - correction_odo1);
//     motor2.setSpeed(motor2_speed - correction_odo2);
//     motor3.setSpeed(motor3_speed - correction_odo3);

//     if (i == 2500) {
//         print_data();
//         i = 0;
//     } else {
//         i++;
//     }
// }

void loop() {
    static int i = 0;

    myIMU.update();
    updateEncoders();
    // fuzzy();
    computePID();

    motor1.setSpeed(motor1_speed); // - correction_odo1);
    motor2.setSpeed(motor2_speed); // - correction_odo2);
    motor3.setSpeed(motor3_speed); // - correction_odo3);
    
    if (i == 2000) {
        printEncoderInfo();
        Serial.println("==================================================");
        Serial.println("Corrections odomètres : ");
        Serial.print("c1 = ");
        Serial.println(correction_odo1);
        Serial.print("c2 = ");
        Serial.println(correction_odo2);
        Serial.print("c3 = ");
        Serial.println(correction_odo3);
        Serial.println("==================================================");
        i = 0;
    } else {
        i++;
    }
}