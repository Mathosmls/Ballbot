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
























// #include "imu.hpp"
// #include "pid.hpp"          
// #include "CytronMotorDriver.h"
// #include <SimpleFOC.h>
// #include <Arduino.h>
// #include <math.h>
// #include <vector>
// #include <algorithm>  // Pour std::clamp
// using namespace std;
// //----------------------------------------

// #pragma region "IMU"

// // IMU myIMU(1000,4);
// IMU myIMU(500, 2);

// #pragma endregion

// //----------------------------------------

// #pragma region "PID de contrôle de l'angle (boucle externe)"

// // double Kp_pitch = 1700.0, Ki_pitch = 0.0, Kd_pitch = 100.0;
// // double Kp_roll = 1700.0, Ki_roll = 0.0, Kd_roll = 100.0;
// double Kp_pitch = 67.0, Ki_pitch = 0.0, Kd_pitch = 3.;
// double Kp_roll = Kp_pitch, Ki_roll = Ki_pitch, Kd_roll = Kd_pitch;
// double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
// double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
// double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
// MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
// MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
// double offset_roll,offset_pitch;

// // Voir fonction compute_PID()

// // double Kp_max = 2500., Kp_min = 1500.;
// // double alpha_max = M_PI/45, alpha_dot_max = 1.; // alpha_max (rad), alpha_dot_max(rad/s)
// // float lambda = 0.5; // Permet de prendre plus ou moins en compte la vitesse de rotation / l'angle (lambda est compris entre 0 et 1)

// // void fuzzy() {
// //     double newKp_pitch = Kp_max*(1./2.)*(lambda*(pitch/alpha_max) + (1 - lambda)*(pid_pitch.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
// //     double newKp_roll = Kp_max*(1./2.)*(lambda*(roll/alpha_max) + (1 - lambda)*(pid_roll.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
// //     pid_pitch.SetKp(constrain(newKp_pitch, Kp_min, Kp_max));
// //     pid_roll.SetKp(constrain(newKp_roll, Kp_min, Kp_max));
// // }




// #pragma endregion

// //----------------------------------------

// #pragma region "Moteurs"

// CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
// CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
// CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.
// vector<CytronMD> motors ={motor1,motor2,motor3};
// double motor1_speed, motor2_speed, motor3_speed;

// // double get_motor_speed(int numero, double vx, double vy) {
// //   double angle = (1 - numero)*(2./3.)*M_PI + M_PI;
// //   double v_motor = vx*sin(angle) + vy*cos(angle);
// //   return constrain(v_motor, -255, 255);
// // }

// void set_setpoints(vector<double> &set, double vx, double vy) {
//   for (int i=0;i<(set.size());i++)
//   {
//     double angle = (1 - (i+1))*(2./3.)*M_PI + M_PI;
//     double v_motor = -(vx*sin(angle) + vy*cos(angle));
//     set[i]=constrain(v_motor, -10., 10.);
//   }
// }

// #pragma endregion

// //----------------------------------------

// #pragma region "Encodeurs + contrôle de la vitesse de rotation en boucle fermée"

// // const int NUM_ENCODERS = 3;
// // Encoder encoders[NUM_ENCODERS] = {
// //     Encoder(2, 3, 500),
// //     Encoder(5, 6, 500),
// //     Encoder(30, 31, 500)};

// // // Fonctions d'interruption pour chaque encodeur
// // void doA0() { encoders[0].handleA(); }
// // void doB0() { encoders[0].handleB(); }
// // void doA1() { encoders[1].handleA(); }
// // void doB1() { encoders[1].handleB(); }
// // void doA2() { encoders[2].handleA(); }
// // void doB2() { encoders[2].handleB(); }

// // // Tableau de pointeurs vers les fonctions d'interruption
// // void (*doA[NUM_ENCODERS])() = {doA0, doA1, doA2};
// // void (*doB[NUM_ENCODERS])() = {doB0, doB1, doB2};

// // void updateEncoders() {
// //   for (int j = 0; j < NUM_ENCODERS; j++) {
// //     encoders[j].update(); // Met à jour les données de l'encodeur
// //   }
// // }

// // void printEncoderInfo() {
// //   for (int j = 0; j < NUM_ENCODERS; j++) {
// //     Serial.print("enc ");
// //     Serial.print(j + 1);
// //     Serial.print(" : ");
// //     Serial.print(encoders[j].getAngle());
// //     Serial.print("\t");
// //     Serial.println(encoders[j].getVelocity());
// //   }
// // }

// // double Kp_odo = 3.0, Ki_odo = 0.0, Kd_odo = 0.0;
// // double setpoint_odo1, setpoint_odo2, setpoint_odo3;
// // double odo1, odo2, odo3;
// // double correction_odo1, correction_odo2, correction_odo3;
// // MyPID pid_odo1(Kp_odo, Ki_odo, Kd_odo, &odo1, &correction_odo1, &setpoint_odo1);
// // MyPID pid_odo2(Kp_odo, Ki_odo, Kd_odo, &odo2, &correction_odo2, &setpoint_odo2);
// // MyPID pid_odo3(Kp_odo, Ki_odo, Kd_odo, &odo3, &correction_odo3, &setpoint_odo3);

// #pragma endregion


// //----------------------------------------
// //Core program
// //----------------------------------------
// // void print_data() {
// //     myIMU.printAll();
// //     printEncoderInfo();
// //     Serial.println("==================================================");
// //     Serial.println("PID : ");
// //     Serial.print("Vx = ");
// //     Serial.print(vx);
// //     Serial.print("\tVy = ");
// //     Serial.println(vy);
// //     Serial.print("Kp_pitch = ");
// //     Serial.println(pid_pitch.GetKp());
// //     Serial.print("Kp_roll = ");
// //     Serial.println(pid_roll.GetKp());

// //     Serial.println("Vitesses moteurs : ");
// //     Serial.print("v1 = ");
// //     Serial.println(motor1_speed);
// //     Serial.print("v2 = ");
// //     Serial.println(motor2_speed);
// //     Serial.print("v3 = ");
// //     Serial.println(motor3_speed);

// //     Serial.println("Corrections odomètres : ");
// //     Serial.print("c1 = ");
// //     Serial.println(correction_odo1);
// //     Serial.print("c2 = ");
// //     Serial.println(correction_odo2);
// //     Serial.print("c3 = ");
// //     Serial.println(correction_odo3);
// //     Serial.println("==================================================");
// // }

// void computePID(vector<double> &set, double &pitch_b, double &roll_b, int &i_b ) {
//   pitch = pitch_b/i_b-offset_pitch;
//   roll =roll_b/i_b-offset_roll;
//   i_b=0;
//   roll_b=0.;
//   pitch_b=0.;
//   pid_pitch.Compute();
//   pid_roll.Compute();
//   set_setpoints(set, vx, vy);

// }

// void set_speed_motors(vector<double> &pids_wheels, vector<double> &cmd_motors) {
//   for (int i =0;i<3;i++)
//   {
//     cmd_motors[i] = pids_wheels[i];
//     // pids_wheels[i].Compute();

//   }
// }

// // void get_speed_motors(vector<double> &last_pos, vector<double> &speeds, double dt)
// // {
// //   //return speed in rad/s
// //   for (int j = 0; j < size(encoders); j++) {
// //     speeds[j]=(last_pos[j] -encoders[j].getAngle())/(dt/1000);
// //     last_pos[j]=encoders[j].getAngle();
// //   }
// // }

// void setup() {
//     Serial.begin(115200);

//     if (!myIMU.init()) {
//         Serial.println("Échec de l'initialisation de l'IMU.");
//     }
//     myIMU.calibrate();
//     Serial.println("end setup");


//     // for (int i = 0; i < NUM_ENCODERS; i++) {
//     //     encoders[i].quadrature = Quadrature::ON;
//     //     encoders[i].pullup = Pullup::USE_EXTERN;
//     //     encoders[i].init();

//     //     // Attacher les interruptions matérielles
//     //     attachInterrupt(digitalPinToInterrupt(encoders[i].pinA), doA[i], CHANGE);
//     //     attachInterrupt(digitalPinToInterrupt(encoders[i].pinB), doB[i], CHANGE);
//     // }
// }

// vector<double> cmd_motors={0.,0.,0.};
// vector<double> setpoints={5,5,5};

// // double setpoints[3] = {4, 4, 4}; // vitesse visée pour chaque moteur
// // int cmd_motors[3] = {0, 0, 0};       // vitesse visée pour chaque moteur

// vector<double> prev_cmd_motors={0.,0.,0.};


// // vector<double> speed_motors={0.,0.,0.};
// // vector<double> last_pos_motors={0.,0.,0.};

// const unsigned long interval = 1./200.*1000000.0; // 5000 µs = 5 ms → 200 Hz

// double roll_buff=0;
// double pitch_buff=0;
// int i_buff=0;

// // double Kp_wheel= 2.2, Ki_wheel= 0.0, Kd_wheel= 4.0;
// // double Kp_wheel= 3., Ki_wheel= 0.0, Kd_wheel= 8.0;

// // double Kp_wheel= 3.5, Ki_wheel= 0.0, Kd_wheel= 20.0;
// // double Kp_wheel= 2.5, Ki_wheel= 0.0, Kd_wheel= 10.0;

// // MyPID pid_wheel1(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[0], &cmd_motors[0], &setpoints[0]);
// // MyPID pid_wheel2(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[1], &cmd_motors[1], &setpoints[1]);
// // MyPID pid_wheel3(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[2], &cmd_motors[2], &setpoints[2]);
// // vector<MyPID> pids_wheels = {pid_wheel1,pid_wheel2,pid_wheel3};

// void loop() {
//     static unsigned long previousTime = 0;
//     unsigned long currentTime = micros();

//     myIMU.update_all();
//     roll_buff+=myIMU.get_roll_rad();
//     pitch_buff+=myIMU.get_pitch_rad();
//     i_buff++;

//     // updateEncoders();
//     // // fuzzy();


//     if (currentTime - previousTime >= interval) 
//     {
//         previousTime = currentTime;
//         int j = 0;

//         computePID(setpoints,pitch_buff,roll_buff,i_buff);

//         // get_speed_motors(last_pos_motors,speed_motors,interval);

//         set_speed_motors(setpoints, cmd_motors);

//         for (int i=0;i<3;i++)
//         {
//           double cmd=(prev_cmd_motors[i]+cmd_motors[i]);
//           cmd= clamp(cmd, -255., 255.);
//           // Serial.println(cmd);
//           motors[i].setSpeed(cmd);
//           prev_cmd_motors[i]=cmd;
//         }

//         // Serial.print("cmd:");
//         // Serial.println(setpoints[0]);
//         // Serial.print("speed:");
//         // Serial.println(speed_motors[0]);
//         // j++;
//         // if (j>0.3/(interval/1000))
//         // {
//         //   if (setpoints[2]<9)
//         //   setpoints[2]+=5;
//         //   else 
//         //   setpoints[2]-=10;
//         //   j=0;
//         // }
        

        
        
//         // // motor1_speed = 50;
//         // // motor2_speed = 50;
//         // // motor3_speed = 100;
        
//         // motor1.setSpeed(motor1_speed); // - correction_odo1);
//         // motor2.setSpeed(motor2_speed); // - correction_odo2);
//         // motor3.setSpeed(motor3_speed); // - correction_odo3);
        
//         if (j == 100) {
            
//             // // printEncoderInfo();
//             // String output = "cal Speeds: " + String(speed_motors[0]) + " "+ String(speed_motors[1]) + " " + String(speed_motors[2]);   
//             // Serial.println(output);   
//             // output = "set Speeds: " + String(setpoints[0]) + " "+ String(setpoints[1]) + " " + String(setpoints[2]);   
//             // Serial.println(output);  
//             // output = "cmd motors: " + String(cmd_motors[0]) + " "+ String(cmd_motors[1]) + " " + String(cmd_motors[2]);   
//             // Serial.println(output);
//             // myIMU.printAngle();
//             myIMU.printAll();
//             // output = "enc Speeds: " + String(encoders[0].getVelocity()) + " "+ String(encoders[1].getVelocity()) + " " + String(encoders[2].getVelocity());   
//             // Serial.println(output); 
//             // Serial.println("==================================================");
//             // Serial.println("Corrections odomètres : ");
//             // Serial.print("c1 = ");
//             // Serial.println(correction_odo1);
//             // Serial.print("c2 = ");
//             // Serial.println(correction_odo2);
//             // Serial.print("c3 = ");
//             // Serial.println(correction_odo3);
//             // Serial.println("==================================================");
//             j = 0;
//         } else {
//             j++;
//         }
//     }
// }