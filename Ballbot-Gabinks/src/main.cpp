#include "imu.hpp"
#include "pid.hpp"          
#include "CytronMotorDriver.h"
#include <SimpleFOC.h>
#include <Arduino.h>
#include <math.h>
#include <vector>

using namespace std;

//----------------------------------------

#pragma region "IMU"

IMU myIMU(1000,4);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"

double Kp_pitch = 60.0, Ki_pitch = 0.0, Kd_pitch = 3.0;
double Kp_roll = 60.0, Ki_roll = 0.0, Kd_roll = 0.0;
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
double pi = 3.14159265358979;

// Voir fonction compute_PID()

double Kp_max = 80., Kp_min = 40.;
double alpha_max = pi/60, alpha_dot_max = 1.; // alpha_max (rad), alpha_dot_max(rad/s)
float lambda = 0.5; // Permet de prendre plus ou moins en compte la vitesse de rotation / l'angle (lambda est compris entre 0 et 1)

void fuzzy() {
    double newKp_pitch = Kp_max*(1./2.)*(lambda*(pitch/alpha_max) + (1 - lambda)*(pid_pitch.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    double newKp_roll = Kp_max*(1./2.)*(lambda*(roll/alpha_max) + (1 - lambda)*(pid_roll.Get_derivative() + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    pid_pitch.SetKp(constrain(newKp_pitch, Kp_min, Kp_max));
    pid_roll.SetKp(constrain(newKp_roll, Kp_min, Kp_max));
}

#pragma endregion

//----------------------------------------

#pragma region "Encodeurs"

int TicksPerRevolution = 500; // Nombre de ticks par révolution

const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(2, 3, TicksPerRevolution),
    Encoder(5, 6, TicksPerRevolution),
    Encoder(30, 31, TicksPerRevolution)};

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

#pragma endregion

//----------------------------------------

#pragma region "Moteurs + contrôle de la vitesse de rotation en boucle fermée"

CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.
vector<CytronMD> motors = {motor1,motor2,motor3};

vector<double> motors_cmd = {0., 0., 0.};
vector<double> motors_setpoints = {12.5, 6.25, 3.125};
vector<double> motors_prev_cmd = {0., 0., 0.};
vector<double> motors_speed = {0., 0., 0.};
vector<double> last_pos_motors = {0., 0., 0.};

float maxspeed = 8.; // Vitesse maximale des moteurs (en rad/s)
double Kp_motors = 3.5, Ki_motors = 0.0, Kd_motors = 20.0;
MyPID pid_motor1(Kp_motors, Ki_motors, Kd_motors, &motors_speed[0], &motors_cmd[0], &motors_setpoints[0]);
MyPID pid_motor2(Kp_motors, Ki_motors, Kd_motors, &motors_speed[1], &motors_cmd[1], &motors_setpoints[1]);
MyPID pid_motor3(Kp_motors, Ki_motors, Kd_motors, &motors_speed[2], &motors_cmd[2], &motors_setpoints[2]);
vector<MyPID> pids_motors = {pid_motor1, pid_motor2, pid_motor3};

void set_motors_setpoints(vector<double> &set, double vx, double vy) {
  for (int i = 0 ; i < set.size() ; i++) {
    double angle = (1 - (i + 1))*(2./3.)*pi + pi;
    double v_motor = vx*sin(angle) + vy*cos(angle);
    set[i] = constrain(v_motor, -maxspeed, maxspeed);
  }
}

void get_speed_motors(vector<double> &last_pos, vector<double> &speeds, double dt) {
  //return speed in rad/s
  for (int j = 0 ; j < NUM_ENCODERS ; j++) {
    double current_angle = encoders[j].getAngle();
    speeds[j] = (last_pos[j] - current_angle)/(dt/1000);
    last_pos[j] = current_angle;
  }
}

#pragma endregion

//----------------------------------------

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
    Serial.println(motors_speed[0]);
    Serial.print("v2 = ");
    Serial.println(motors_speed[1]);
    Serial.print("v3 = ");
    Serial.println(motors_speed[2]);

    Serial.println("Corrections odomètres : ");
    Serial.print("c1 = ");
    Serial.println(correction_odo1);
    Serial.print("c2 = ");
    Serial.println(correction_odo2);
    Serial.print("c3 = ");
    Serial.println(correction_odo3);
    Serial.println("==================================================");
}

void print_data_Teleplot() {
  Serial.println("==================================================");
  Serial.print(">Sortie PID pitch:");
  Serial.println(vx);
  Serial.print(">Sortie PID roll:");
  Serial.println(vy);
  Serial.print(">Kp du PID pitch:");
  Serial.println(pid_pitch.GetKp());
  Serial.print(">Kp du PID roll:");
  Serial.println(pid_roll.GetKp());
  Serial.print(">Vitesse moteur 1:");
  Serial.println(motors_speed[0]);
  Serial.print(">Vitesse moteur 2:");
  Serial.println(motors_speed[1]);
  Serial.print(">Vitesse moteur 3:");
  Serial.println(motors_speed[2]);
  Serial.print(">Sortie PID moteur 1:");
  Serial.println(motors_cmd[0]);
  Serial.print(">Sortie PID moteur 2:");
  Serial.println(motors_cmd[1]);
  Serial.print(">Sortie PID moteur 3:");
  Serial.println(motors_cmd[2]);
  Serial.print(">Commande moteur 1:");
  Serial.println(motors_prev_cmd[0]);
  Serial.print(">Commande moteur 2:");
  Serial.println(motors_prev_cmd[1]);
  Serial.print(">Commande moteur 3:");
  Serial.println(motors_prev_cmd[2]);
  Serial.print(">Setpoint moteur 1:");
  Serial.println(motors_setpoints[0]);
  Serial.print(">Setpoint moteur 2:");
  Serial.println(motors_setpoints[1]);
  Serial.print(">Setpoint moteur 3:");
  Serial.println(motors_setpoints[2]);
  Serial.print(">Pitch:");
  Serial.println(pitch);
  Serial.print(">Roll:");
  Serial.println(roll);
  Serial.println("==================================================");
}

void computePID(vector<double> &set) {
  pitch = myIMU.get_pitch_rad();
  roll = myIMU.get_roll_rad();
  pid_pitch.Compute();
  pid_roll.Compute();
  // set_motors_setpoints(set, vx, vy);
}

void set_speed_motors(vector<MyPID> pids_wheels) {
  for (int i = 0 ; i < pids_wheels.size() ; i++) {
    pids_wheels[i].Compute();
  }
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

int plot_data = 0;
unsigned long currentTime = millis(), previousTime = millis();
double dt = 10.;

void loop() {

    myIMU.update();
    updateEncoders();
    // fuzzy();
    computePID(motors_setpoints);

    currentTime = millis();
    if (currentTime - previousTime >= dt) {
      previousTime = currentTime;
      get_speed_motors(last_pos_motors, motors_speed, dt);
      set_speed_motors(pids_motors);
      for (int i = 0 ; i < motors.size() ; i++) {
          double cmd = motors_prev_cmd[i] + motors_cmd[i];
          cmd = clamp(cmd, -255., 255.);
          // Serial.println(cmd);
          motors[i].setSpeed(cmd);
          motors_prev_cmd[i] = cmd;
      }

      // Envoie les données à Teleplot
      if (plot_data%10 == 0) {
        print_data_Teleplot();
      }

      if (plot_data == 100) {
          print_data();
          plot_data = 0;
      } else {
          plot_data++;
      }
    }
}