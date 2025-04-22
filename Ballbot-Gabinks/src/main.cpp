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

IMU myIMU(500, 2);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"

<<<<<<< HEAD
double Kp_pitch = 5, Ki_pitch = 0.0, Kd_pitch = 0.01;
double Kp_roll = Kp_pitch, Ki_roll = Ki_pitch, Kd_roll = Kd_pitch;
=======
double Kp_pitch = 60.0, Ki_pitch = 0.0, Kd_pitch = 3.0;
double Kp_roll = 60.0, Ki_roll = 0.0, Kd_roll = 0.0;
>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e
double setpoint_pitch = 0.;        // Angle cible calculé par la boucle externe
double setpoint_roll = 0.;        // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy; // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
<<<<<<< HEAD
double offset_roll,offset_pitch;
=======
double pi = 3.14159265358979;

// Voir fonction compute_PID()

double Kp_max = 80., Kp_min = 40.;
double alpha_max = pi/60, alpha_dot_max = 1.; // alpha_max (rad), alpha_dot_max(rad/s)
float lambda = 0.5; // Permet de prendre plus ou moins en compte la vitesse de rotation / l'angle (lambda est compris entre 0 et 1)

void fuzzy() {
    double newKp_pitch = Kp_max*(1./2.)*(lambda*(abs(pitch)/alpha_max) + (1 - lambda)*(abs(pid_pitch.Get_derivative()) + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    double newKp_roll = Kp_max*(1./2.)*(lambda*(abs(roll)/alpha_max) + (1 - lambda)*(abs(pid_roll.Get_derivative()) + alpha_dot_max)/(2*alpha_dot_max)); // Mériterait d'être autre chose que linéaire
    pid_pitch.SetKp(constrain(newKp_pitch, Kp_min, Kp_max));
    pid_roll.SetKp(constrain(newKp_roll, Kp_min, Kp_max));
}
>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e

#pragma endregion

//----------------------------------------

#pragma region "Encodeurs"

<<<<<<< HEAD
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
=======
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
>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e
  }
}

#pragma endregion

//----------------------------------------
<<<<<<< HEAD
=======

#pragma region "Moteurs + contrôle de la vitesse de rotation en boucle fermée"

CytronMD motor1(PWM_DIR, 12, 10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 7, 11); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 8, 9); // PWM 3 = Pin 7, DIR 3 = Pin 11.
vector<CytronMD> motors = {motor1,motor2,motor3};

vector<double> motors_cmd = {0., 0., 0.};
vector<double> motors_setpoints = {0., 0., 0.};
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
>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e
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

<<<<<<< HEAD
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

=======
    Serial.println("Vitesses moteurs : ");
    Serial.print("v1 = ");
    Serial.println(motors_speed[0]);
    Serial.print("v2 = ");
    Serial.println(motors_speed[1]);
    Serial.print("v3 = ");
    Serial.println(motors_speed[2]);

    Serial.println("Corrections odomètres : ");
    Serial.print("c1 = ");
    Serial.println(motors_cmd[0]);
    Serial.print("c2 = ");
    Serial.println(motors_cmd[1]);
    Serial.print("c3 = ");
    Serial.println(motors_cmd[2]);
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
  set_motors_setpoints(set, vx, vy);
}

void set_speed_motors(vector<MyPID> pids_wheels) {
  for (int i = 0 ; i < pids_wheels.size() ; i++) {
    pids_wheels[i].Compute();
  }
}

>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e
void setup() {
    Serial.begin(115200);

    if (!myIMU.init()) {
        Serial.println("Échec de l'initialisation de l'IMU.");
    }
    myIMU.calibrate();
    Serial.println("end setup");

}

<<<<<<< HEAD
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
          double cmd=(prev_cmd_motors[i]+setpoints[i]);
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
=======
int plot_data = 0;
unsigned long currentTime = millis(), previousTime = millis();
double dt = 10.;

void loop() {

    myIMU.update();
    updateEncoders();
    fuzzy();
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
>>>>>>> 8b251ebc0d2cc3446c5f210a3df649b26fbe176e
    }
}