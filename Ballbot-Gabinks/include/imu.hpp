#ifndef IMU_H
#define IMU_H

#include "FastIMU.h"
#include <Wire.h>
#include <Arduino.h>
// #define PERFORM_CALIBRATION
#define IMU_ADDRESS 0x68 // Adresse de l'IMU


class IMU {
private:
    MPU6050 imu;               // Instanciation de l'IMU
    calData calibrationData;   // Données de calibration
    AccelData accelData;       // Données de l'accéléromètre
    GyroData gyroData;         // Données du gyroscope
    int _gyro_range;
    int _acc_range;
    unsigned long lastUpdateTime = 0; 
    float dt =0.01;
    float filtered_pitch;
    float filtered_roll;
    float alpha =0.98;
    float angleGyroX;
    float angleGyroY;

public:
    IMU(int gyro_range, int acc_range);              // Constructeur

    // Méthodes publiques
    bool init();               // Initialise l'IMU
    void calibrate();          // Calibre l'IMU
    void update();             // Met à jour les données de l'IMU
    void printAll();          // Affiche les données sur le port série
    void printAngle();
    AccelData get_acc() const;
    GyroData get_gyro() const;
    float get_pitch_deg() const;
    float get_roll_deg() const;
    float get_pitch_rad() const;
    float get_roll_rad() const;
    void complementary_filter();
    float degToRad(float degrees) const;
};

#endif // IMU_H
