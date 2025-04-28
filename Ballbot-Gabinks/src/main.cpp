#include "imu.hpp"
#include "pid.hpp"
#include <Arduino.h>
#include <math.h>
using namespace std;
//----------------------------------------

template <typename T>
T clamp(T val, T min_val, T max_val)
{
  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

#pragma region "IMU"

IMU myIMU(500, 4);

#pragma endregion

//----------------------------------------

#pragma region "PID de contrôle de l'angle (boucle externe)"
// double Kp_pitch = 137.0, Ki_pitch = 0.6, Kd_pitch =1.4;
double Kp_pitch = 90.0, Ki_pitch = 0., Kd_pitch =0.;
double Kp_roll = Kp_pitch, Ki_roll = Ki_pitch, Kd_roll = Kd_pitch;
double setpoint_pitch = radians(0.0); // Angle cible calculé par la boucle externe
double setpoint_roll = radians(0.);  // Angle cible calculé par la boucle externe
double roll, pitch, vx, vy;           // Entrée et sortie de la boucle interne
MyPID pid_pitch(Kp_pitch, Ki_pitch, Kd_pitch, &pitch, &vx, &setpoint_pitch);
MyPID pid_roll(Kp_roll, Ki_roll, Kd_roll, &roll, &vy, &setpoint_roll);
double offset_roll, offset_pitch;

#pragma endregion

//----------------------------------------

#pragma region "Moteurs"

void cmd_rot_speeds(double (&cmd_rad)[3], double vx, double vy)
{
  for (int i = 0; i < 3; i++)
  {
    double angle = (1 - (i + 1)) * (2. / 3.) * M_PI + M_PI;
    double v_motor = -(vx * sin(angle) + vy * cos(angle));
    cmd_rad[i] = constrain(v_motor, -16., 16.);
  }
}

void set_motors_speed(double (&cmd_rad)[3], int (&cmd_mot)[3], int maxi)
{
  for (int i = 0; i < 3; i++)
  {
    int cmd_step_mot = int(float(cmd_rad[i]) / (2. * M_PI) * 200. * 10. * 4.); // rad/s->tr/s->tick/s->etage de reduction->microstep
    cmd_step_mot = constrain(cmd_step_mot, -maxi, maxi);
    cmd_mot[i] = cmd_step_mot;
  }
}

void print_cmd_speed(double cmd_speed[3])
{
  for (int i = 0; i < 3; i++)
  {
    String output = ">cmd speed motor " + String(i + 1) + " : ";
    Serial.print(output);
    Serial.println(cmd_speed[i]);
  }
}

void print_cmd_motors(int cmd_mot[3])
{
  for (int i = 0; i < 3; i++)
  {
    String output = ">cmd tick motor " + String(i + 1) + " : ";
    Serial.print(output);
    Serial.println(cmd_mot[i]);
  }
}

void send_cmd_mot(byte (&buf)[7], const int (&cmd_mot)[3])
{
  buf[0] = 0x01; // Byte de démarrage

  // Convertir chaque entier cmd_mot[i] en 2 octets (big endian)
  buf[1] = (cmd_mot[0] >> 8) & 0xFF; // Octet de poids fort de cmd_mot[0]
  buf[2] = cmd_mot[0] & 0xFF;        // Octet de poids faible de cmd_mot[0]

  buf[3] = (cmd_mot[1] >> 8) & 0xFF; // Octet de poids fort de cmd_mot[1]
  buf[4] = cmd_mot[1] & 0xFF;        // Octet de poids faible de cmd_mot[1]

  buf[5] = (cmd_mot[2] >> 8) & 0xFF; // Octet de poids fort de cmd_mot[2]
  buf[6] = cmd_mot[2] & 0xFF;        // Octet de poids faible de cmd_mot[2]

  // Envoyer le tableau buf avec les 7 octets
  Serial1.write(buf, sizeof(buf));
}

#pragma endregion

//----------------------------------------
// Core program
//----------------------------------------

void computePID(double (&cmd_rad)[3], double pitch_b, double roll_b, int &i_b)
{
  pitch = pitch_b - offset_pitch;
  roll = roll_b  - offset_roll;
  if (abs(pitch)<radians(0.01))
  {
    pitch=0.0;
  }
  if (abs(roll)<radians(0.01))
  {
    roll=0.0;
  }
  pid_pitch.Compute();
  pid_roll.Compute();
  cmd_rot_speeds(cmd_rad, vx, vy);
}

void setup()
{
  Serial.begin(230400);
  Serial1.begin(230400);

  if (!myIMU.init())
  {
    // Serial.println("Échec de l'initialisation de l'IMU.");
  }
  myIMU.calibrate();
  Serial.println("end setup");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

double cmd_speed_rad[3] = {4, 4, 4}; // vitesse visée pour chaque moteur
int cmd_motors[3] = {0, 0, 0};       // vitesse visée pour chaque moteur

unsigned long previousTime = 0;
const unsigned long interval = 1. / 250. * 1000000.0; // 5000 µs = 5 ms → 200 Hz
byte buffer_cmd_mot[7];
double roll_buff = 0;
double pitch_buff = 0;
int i_buff = 0;
void loop()
{
  static unsigned long lastTime = 0;

  myIMU.update_all();
  unsigned long now = micros();
  if (now - lastTime >= interval)
  {
    lastTime = now;

    static int i = 0;
    // unsigned long t0 = micros();

    computePID(cmd_speed_rad, myIMU.get_pitch_rad(), myIMU.get_roll_rad(), i_buff);
    set_motors_speed(cmd_speed_rad, cmd_motors, 16000);
    // cmd_motors[0]=2000+i;
    // cmd_motors[1]=2000+i;
    // cmd_motors[2]=2000+i;
    send_cmd_mot(buffer_cmd_mot, cmd_motors);

    if (i == 10)
    {
      print_cmd_speed(cmd_speed_rad);
      print_cmd_motors(cmd_motors);
      // myIMU.printAngle();
      // unsigned long now = micros();
      Serial.print(">roll:");
      Serial.println(degrees(roll));
      Serial.print(">pitch:");
      Serial.println(degrees(pitch));
      // unsigned long t1 = micros();
      // Serial.print("IMU update took: ");
      // Serial.println(t1 - now);
      // Serial.println(" us");
      // Serial.println(interval);

      Serial.print(">roll_kalman:");
      Serial.println(myIMU.get_kalman_roll_deg());
      Serial.print(">pitch_kalman:");
      Serial.println(myIMU.get_kalman_pitch_deg());

      // Serial.print(">roll_madgwick:");
      // Serial.println(myIMU.get_madgwick_roll_deg());
      // Serial.print(">pitch_madgwick:");
      // Serial.println(myIMU.get_madgwick_pitch_deg());

      Serial.print(">accX:");
      Serial.println(myIMU.filtered_accel.accelX);
      Serial.print(">accY:");
      Serial.println(myIMU.filtered_accel.accelY);
      Serial.print(">accZ:");
      Serial.println(myIMU.filtered_accel.accelZ);

      Serial.print(">gyroX:");
      Serial.println(myIMU.gyroData.gyroX);
      Serial.print(">gyroY:");
      Serial.println(myIMU.gyroData.gyroY);
      Serial.print(">gyroZ:");
      Serial.println(myIMU.gyroData.gyroZ);

      i = 0;
    }
    else
    {
      i++;
    }
    //   unsigned long t1 = micros();
    // Serial.print("IMU update took: ");
    // Serial.println(t1 - now);
    // Serial.println(" us");
  }
}