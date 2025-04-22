#include "imu.hpp"
#include "pid.hpp"          
#include "CytronMotorDriver.h"
#include <SimpleFOC.h>
#include <Arduino.h>
#include <math.h>
#include <vector>
#include <algorithm>  // Pour std::clamp
using namespace std;
//----------------------------------------

#pragma region "IMU"

IMU myIMU(500, 2);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"

double Kp_pitch =2800, Ki_pitch = 0.0, Kd_pitch = 0.01;
double Kp_roll = Kp_pitch, Ki_roll = Ki_pitch, Kd_roll = Kd_pitch;
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
double offset_roll,offset_pitch;

#pragma endregion

//----------------------------------------

#pragma region "Moteurs"

CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.
vector<CytronMD> motors ={motor1,motor2,motor3};
double motor1_speed, motor2_speed, motor3_speed;

void set_setpoints(double (&cmd_rad)[3], double vx, double vy) {
  for (int i=0;i<3;i++)
  {
    double angle = (1 - (i+1))*(2./3.)*M_PI + M_PI;
    double v_motor = vx*sin(angle) + vy*cos(angle);
    cmd_rad[i]=constrain(v_motor, -255., 255.);
  }
}

#pragma endregion

//----------------------------------------
//Core program
//----------------------------------------
// void print_data() {
//     myIMU.printAll();
//     printEncoderInfo();
//     Serial.println("==================================================");
//     Serial.println("PID : ");
//     Serial.print("Vx = ");
//     Serial.print(vx);
//     Serial.print("\tVy = ");
//     Serial.println(vy);
//     Serial.print("Kp_pitch = ");
//     Serial.println(pid_pitch.GetKp());
//     Serial.print("Kp_roll = ");
//     Serial.println(pid_roll.GetKp());

//     Serial.println("Vitesses moteurs : ");
//     Serial.print("v1 = ");
//     Serial.println(motor1_speed);
//     Serial.print("v2 = ");
//     Serial.println(motor2_speed);
//     Serial.print("v3 = ");
//     Serial.println(motor3_speed);

//     Serial.println("Corrections odomètres : ");
//     Serial.print("c1 = ");
//     Serial.println(correction_odo1);
//     Serial.print("c2 = ");
//     Serial.println(correction_odo2);
//     Serial.print("c3 = ");
//     Serial.println(correction_odo3);
//     Serial.println("==================================================");
// }

void computePID(double (&cmd_mot)[3], double &pitch_b, double &roll_b, int &i_b ) {
  pitch = pitch_b/i_b-offset_pitch;
  roll =roll_b/i_b-offset_roll;
  i_b=0;
  roll_b=0.;
  pitch_b=0.;
  pid_pitch.Compute();
  pid_roll.Compute();
  set_setpoints(cmd_mot, vx, vy);

}

void setup() {
    Serial.begin(115200);

    if (!myIMU.init()) {
        Serial.println("Échec de l'initialisation de l'IMU.");
    }
    myIMU.calibrate();
    Serial.println("end setup");

}

// vector<double> cmd_motors={0.,0.,0.};
// vector<double> setpoints={5,5,5};

double setpoints[3] = {4, 4, 4}; // vitesse visée pour chaque moteur
int cmd_motors[3] = {0, 0, 0};       // vitesse visée pour chaque moteur

vector<double> prev_cmd_motors={0.,0.,0.};


// vector<double> speed_motors={0.,0.,0.};
// vector<double> last_pos_motors={0.,0.,0.};

const unsigned long interval = 1./200.*1000000.0; // 5000 µs = 5 ms → 200 Hz

double roll_buff=0;
double pitch_buff=0;
int i_buff=0;

// double Kp_wheel= 2.2, Ki_wheel= 0.0, Kd_wheel= 4.0;
// double Kp_wheel= 3., Ki_wheel= 0.0, Kd_wheel= 8.0;

// double Kp_wheel= 3.5, Ki_wheel= 0.0, Kd_wheel= 20.0;
// double Kp_wheel= 2.5, Ki_wheel= 0.0, Kd_wheel= 10.0;

// MyPID pid_wheel1(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[0], &cmd_motors[0], &setpoints[0]);
// MyPID pid_wheel2(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[1], &cmd_motors[1], &setpoints[1]);
// MyPID pid_wheel3(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[2], &cmd_motors[2], &setpoints[2]);
// vector<MyPID> pids_wheels = {pid_wheel1,pid_wheel2,pid_wheel3};

void loop() {
    static unsigned long previousTime = 0;
    unsigned long currentTime = micros();

    myIMU.update_all();
    roll_buff+=myIMU.get_roll_rad();
    pitch_buff+=myIMU.get_pitch_rad();
    i_buff++;

    if (currentTime - previousTime >= interval) 
    {
        previousTime = currentTime;
        int j = 0;

        computePID(setpoints,pitch_buff,roll_buff,i_buff);

        for (int i=0;i<3;i++)
        {
          double cmd=(setpoints[i]);
          cmd= clamp(cmd, -255., 255.);
          // Serial.println(cmd);
          motors[i].setSpeed(cmd);
          prev_cmd_motors[i]=cmd;
        }
        myIMU.printAll();
        if (j == 2) {
            
            // String output = "cal Speeds: " + String(speed_motors[0]) + " "+ String(speed_motors[1]) + " " + String(speed_motors[2]);   
            // Serial.println(output);   
            // output = "set Speeds: " + String(setpoints[0]) + " "+ String(setpoints[1]) + " " + String(setpoints[2]);   
            // Serial.println(output);  
            // output = "cmd motors: " + String(cmd_motors[0]) + " "+ String(cmd_motors[1]) + " " + String(cmd_motors[2]);   
            // Serial.println(output);
            // myIMU.printAngle();
            myIMU.printAll();
            j = 0;
        } else {
            j++;
        }
    }
}