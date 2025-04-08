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

IMU myIMU(1000,4);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"

// double Kp_pitch = 1700.0, Ki_pitch = 0.0, Kd_pitch = 100.0;
// double Kp_roll = 1700.0, Ki_roll = 0.0, Kd_roll = 100.0;
double Kp_pitch = 67.0, Ki_pitch = 0.0, Kd_pitch = 3.;
double Kp_roll = Kp_pitch, Ki_roll = Ki_pitch, Kd_roll = Kd_pitch;
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);

double pi = 3.14159265358979;


//PID pour la vitesse des roues
// double Kp_wheel= 2.2, Ki_wheel= 0.0, Kd_wheel= 4.0;
// double Kp_wheel= 3., Ki_wheel= 0.0, Kd_wheel= 8.0;

double Kp_wheel= 3.5, Ki_wheel= 0.0, Kd_wheel= 20.0;
// double Kp_wheel= 2.5, Ki_wheel= 0.0, Kd_wheel= 10.0;
MyPID pid_wheel1(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[0], &cmd_motors[0], &setpoints[0]);
MyPID pid_wheel2(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[1], &cmd_motors[1], &setpoints[1]);
MyPID pid_wheel3(Kp_wheel, Ki_wheel, Kd_wheel, &speed_motors[2], &cmd_motors[2], &setpoints[2]);
vector<MyPID> pids_wheels = {pid_wheel1,pid_wheel2,pid_wheel3};

#pragma endregion

//----------------------------------------

#pragma region "Moteurs"

CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.
vector<CytronMD> motors ={motor1,motor2,motor3};

double get_motor_speed(int numero, double vx, double vy) {
  double angle = (1 - numero)*(2./3.)*pi + pi;
  double v_motor = vx*sin(angle) + vy*cos(angle);
  return constrain(v_motor, -255, 255);
}

void set_setpoints(vector<double> &set, double vx, double vy) {
  for (int i=0;i<set.size();i++)
  {
    double angle = (1 - (i+1))*(2./3.)*pi + pi;
    double v_motor = vx*sin(angle) + vy*cos(angle);
    set[i]=constrain(v_motor, -8., 8.);
  }
}

void set_speed_motors(vector<MyPID> pids_wheels) {
  for (int i =0;i<pids_wheels.size();i++)
  {
    pids_wheels[i].Compute();

  }
}

void get_speed_motors(vector<double> &last_pos, vector<double> &speeds, double dt)
{
  //return speed in rad/s
  for (int j = 0; j < size(encoders); j++) {
    speeds[j]=(last_pos[j] -encoders[j].getAngle())/(dt/1000);
    last_pos[j]=encoders[j].getAngle();
  }
}

#pragma endregion

//----------------------------------------

#pragma region "Encodeurs + contrôle de la vitesse de rotation en boucle fermée"

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

void computePID(vector<double> &set) {
  pitch = myIMU.get_pitch_rad();
  roll = myIMU.get_roll_rad();
  pid_pitch.Compute();
  pid_roll.Compute();
  set_setpoints(set, vx, vy);

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

vector<double> cmd_motors={0.,0.,0.}; //cmd actuelle de la vitesse de chaque moteur (pwm -255 à 255), géré dans le PID
vector<double> setpoints={5,5,5}; //vitesse visée pour chaque moteur 
vector<double> prev_cmd_motors={0.,0.,0.}; 
vector<double> speed_motors={0.,0.,0.}; //vitesse mesurée des moteurs
vector<double> last_pos_motors={0.,0.,0.}; //dernière position des moterus, utilisés pour calculer la vitesse
unsigned long previousTime = 0;
double interval = 10.; // dt en millisecondes (10 ms = 0.01s). Permet de régler la fréquence de lecture des odomètres et de l'envoie de la commande


void loop() {
    static int i = 0;
    static int j = 0;
    myIMU.update();
    updateEncoders();
    computePID(setpoints);

    unsigned long currentTime = millis();
    if (currentTime - previousTime >= interval) {
        Serial.print(">dt:");
        Serial.println(currentTime - previousTime);
        previousTime = currentTime;
        get_speed_motors(last_pos_motors,speed_motors,interval);
        set_speed_motors(pids_wheels);
        for (int i=0;i<motors.size();i++)
        {
          double cmd=(prev_cmd_motors[i]+cmd_motors[i]);
          cmd= clamp(cmd, -255., 255.);
          motors[i].setSpeed(cmd);
          prev_cmd_motors[i]=cmd;
        }
        Serial.print(">cmd:");
        Serial.println(setpoints[0]);
        Serial.print(">speed:");
        Serial.println(speed_motors[0]);
        
    }

    
    
  
    
    if (i == 100) {
        String output = "cal Speeds: " + String(speed_motors[0]) + " "+ String(speed_motors[1]) + " " + String(speed_motors[2]);   
        Serial.println(output);   
        output = "set Speeds: " + String(setpoints[0]) + " "+ String(setpoints[1]) + " " + String(setpoints[2]);   
        Serial.println(output);  
        output = "cmd motors: " + String(cmd_motors[0]) + " "+ String(cmd_motors[1]) + " " + String(cmd_motors[2]);   
        Serial.println(output);
        myIMU.printAngle();

        i = 0;
    } else {
        i++;
    }
}