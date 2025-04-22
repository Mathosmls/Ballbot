#ifndef IMU_H
#define IMU_H

#include "FastIMU.h"
#include <Wire.h>
#include <Arduino.h>

// #define PERFORM_CALIBRATION
#define IMU_ADDRESS 0x68 // Adresse de l'IMU
// #define PERFORM_CALIBRATION

class IMU
{
private:
    MPU6050 imu;             // Instanciation de l'IMU
    calData calibrationData; // Données de calibration
    AccelData bias_accelData;     // Données de l'accéléromètre
    GyroData bias_gyroData;       // Données du gyroscope
    double bias_roll_rad=0.;
    double bias_pitch_rad=0.;
    int _gyro_range;
    int _acc_range;
    unsigned long lastUpdateTime = 0;
    unsigned long lastUpdateTime_kalman = 0;
    unsigned long lastUpdateTime_mad=0;
    float dt = 0.0025; //400hz
    double filtered_pitch;
    double filtered_roll;
    // float alpha = 0.98;
    // float alpha = 0.989;
    float alpha = 0.992;


    // Kalman pour pitch et roll
    float kalman_pitch = 0.0;
    float kalman_roll = 0.0;
    float kalman_bias_pitch = 0.0;
    float kalman_bias_roll = 0.0;

    float P_pitch[2][2] = {{1, 0}, {0, 1}};
    float P_roll[2][2] = {{1, 0}, {0, 1}};

    float Q_angle = 0.02;  // Bruit de processus (angle)
    float Q_bias = 0.003;   // Bruit de processus (biais gyroscope)
    float R_measure = 0.003; // Bruit de mesure (accéléro)

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Quaternion [w, x, y, z]
    float beta = 0.75f; // Gain du filtre Madgwick
    float zeta = 1.f; // Optionnel : gain pour compensation du biais gyroscopique
    float madgwick_pitch = 0.0f;
    float madgwick_roll = 0.0f;

    void apply_offset();

public:
    IMU(int gyro_range, int acc_range); // Constructeur
    AccelData accelData;     // Données de l'accéléromètre
    GyroData gyroData;       // Données du gyroscope
    // Méthodes publiques
    bool init();      // Initialise l'IMU
    void calibrate(); // Calibre l'IMU
    void update_all();    // Met à jour les données de l'IMU
    void printAll();  // Affiche les données sur le port série
    void printAngle();
    AccelData get_acc() const;
    GyroData get_gyro() const;
    double get_pitch_deg() const;
    double get_roll_deg() const;
    double get_pitch_rad() const;
    double get_roll_rad() const;
    void complementary_filter();
    void kalman_filter();
    double get_kalman_pitch_deg() const;
    double get_kalman_roll_deg() const;
    double degToRad(double degrees) const;
    void madgwick_filter();
    double get_madgwick_roll_deg() const;
    double get_madgwick_pitch_deg() const;
};

#endif // IMU_H
