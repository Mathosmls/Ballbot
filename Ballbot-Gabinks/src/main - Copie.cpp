// #include "imu.hpp"
// #include "pid.hpp"          
// #include "CytronMotorDriver.h"
// #include <SimpleFOC.h>
// #include <Arduino.h>
// #include <math.h>

// //----------------------------------------
// //IMU
// //----------------------------------------
// IMU myIMU(1000,4);

// //----------------------------------------
// //PID simple
// //----------------------------------------
// double Kp = 1500.0, Ki = 75.0, Kd = 125.0;
// double pi = 3.14159265358979;
// double setpoint = 0.;        // Angle cible calculé par la boucle externe
// double roll, pitch, psi, alpha, v; // Entrée et sortie de la boucle interne
// MyPID pid_alpha(Kp, Ki, Kd, &alpha, &v, &setpoint);
// // bool boost[3] = {false, false, false};
// int minspeed = 30, blockspeed = 10;

// double get_motor_speed(int numero, double psi, double v) 
// {
//   double v_motor = constrain(abs(v)*sin(psi + (1 - numero)*(2./3.)*pi), -255, 255);
//   if (abs(v_motor) < blockspeed) {
//     // boost[numero] = false;
//     return 0;
//   }
//   // else if (abs(v_motor) < minspeed && !boost[numero]) {
//   //   boost[numero] = true;
//   //   return 100*v_motor/abs(v_motor);
//   // }
//   // else if (abs(v_motor) < minspeed && boost[numero]) {
//   //   return minspeed*v_motor/abs(v_motor);
//   // }
//   else {
//     return v_motor;
//   }
// }

// double prev_alpha, alpha_dot;
// double Kp_max = 2500., Kp_min = 1000.;
// double alpha_max = pi/30, alpha_dot_max = 1.; // alpha_max (rad), alpha_dot_max(rad/s)
// float lambda = 0.5; // Permet de prendre plus ou moins en compte la vitesse de rotation / l'angle (lambda est compris entre 0 et 1)

// void fuzzy() {
//   double newKp = Kp_max*(1./2.)*(lambda*(alpha/alpha_max) + (1 - lambda)*(alpha_dot + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
//   pid_alpha.SetKp(constrain(newKp, Kp_min, Kp_max));
// }

// void computePID()
// {
//     roll = myIMU.get_roll_rad();
//     pitch = myIMU.get_pitch_rad();
//     alpha = asin(sqrt(pow(sin(roll), 2) + pow(sin(pitch), 2)));
//     alpha_dot = pid_alpha.Get_derivative();
//     pid_alpha.Compute();
// }

// //----------------------------------------
// //Motors
// //----------------------------------------
// CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
// CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
// CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.

// //----------------------------------------
// //Encoders
// //----------------------------------------
// const int NUM_ENCODERS = 3;
// Encoder encoders[NUM_ENCODERS] = {
//     Encoder(2, 3, 500),
//     Encoder(5, 6, 500),
//     Encoder(30, 31, 500)};

// // Fonctions d'interruption pour chaque encodeur
// void doA0() { encoders[0].handleA(); }
// void doB0() { encoders[0].handleB(); }
// void doA1() { encoders[1].handleA(); }
// void doB1() { encoders[1].handleB(); }
// void doA2() { encoders[2].handleA(); }
// void doB2() { encoders[2].handleB(); }

// // Tableau de pointeurs vers les fonctions d'interruption
// void (*doA[NUM_ENCODERS])() = {doA0, doA1, doA2};
// void (*doB[NUM_ENCODERS])() = {doB0, doB1, doB2};

// void updateEncoders()
// {
//   for (int j = 0; j < NUM_ENCODERS; j++)
//   {
//     encoders[j].update(); // Met à jour les données de l'encodeur
//   }
// }

// void printEncoderInfo()
// {
//   for (int j = 0; j < NUM_ENCODERS; j++)
//   {
//     Serial.print("enc ");
//     Serial.print(j + 1);
//     Serial.print(" : ");
//     Serial.print(encoders[j].getAngle());
//     Serial.print("\t");
//     Serial.println(encoders[j].getVelocity());
//   }
// }

// //----------------------------------------
// //Core program
// //----------------------------------------
// void print_data() {
//     myIMU.printAll();
//     printEncoderInfo();
//     Serial.println("==================================================");
//     Serial.println("PID : ");
//     Serial.print("V = ");
//     Serial.println(abs(v));
//     Serial.print("psi = ");
//     Serial.println(psi);
//     Serial.print("alpha = ");
//     Serial.println(alpha);
//     Serial.print("Kp = ");
//     Serial.println(pid_alpha.GetKp());

//     Serial.println("Vitesses moteurs : ");
//     Serial.print("v1 = ");
//     Serial.println(get_motor_speed(1, psi, v));
//     Serial.print("v2 = ");
//     Serial.println(get_motor_speed(2, psi, v));
//     Serial.print("v3 = ");
//     Serial.println(get_motor_speed(3, psi, v));
//     Serial.println("==================================================");
// }

// void setup() {
//     Serial.begin(115200);

//     if (!myIMU.init()) {
//         Serial.println("Échec de l'initialisation de l'IMU.");
//     }
//     myIMU.calibrate();

//     for (int i = 0; i < NUM_ENCODERS; i++) {
//         encoders[i].quadrature = Quadrature::ON;
//         encoders[i].pullup = Pullup::USE_EXTERN;
//         encoders[i].init();

//         // Attacher les interruptions matérielles
//         attachInterrupt(digitalPinToInterrupt(encoders[i].pinA), doA[i], CHANGE);
//         attachInterrupt(digitalPinToInterrupt(encoders[i].pinB), doB[i], CHANGE);
//     }
// }

// void loop() {
//     static int i = 0;

//     myIMU.update();
//     updateEncoders();
//     // fuzzy();
//     computePID();

//     psi = atan2(sin(roll), sin(pitch));
//     motor1.setSpeed(get_motor_speed(1, psi, v));
//     motor2.setSpeed(get_motor_speed(2, psi, v));
//     motor3.setSpeed(get_motor_speed(3, psi, v));

//     if (i == 2500) {
//         print_data();
//         i = 0;
//     } else {
//         i++;
//     }
// }