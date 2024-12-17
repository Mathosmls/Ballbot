#include "imu.hpp"

// Constructeur
IMU::IMU(int gyro_range, int acc_range) : _gyro_range(gyro_range),  _acc_range(acc_range){
    memset(&calibrationData, 0, sizeof(calData));
}

// Méthode d'initialisation
bool IMU::init() {
    Wire.begin();
    Wire.setClock(400000); // Horloge I2C à 400 kHz

    int err = imu.init(calibrationData, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Erreur d'initialisation de l'IMU : ");
        Serial.println(err);
        return false;
    }

    Serial.println("IMU initialisée avec succès.");

    // Configuration des plages
    err = imu.setGyroRange(_gyro_range);
    err |= imu.setAccelRange(_acc_range);
    if (err != 0) {
        Serial.print("Erreur de configuration de plage : ");
        Serial.println(err);
        return false;
    }

    Serial.print("Plages configurées : Gyro ±");
    Serial.print(_gyro_range);
    Serial.print("DPS, Accel ±");
    Serial.println(_acc_range);
    return true;
}

// Méthode de calibration
void IMU::calibrate() {
#ifdef PERFORM_CALIBRATION

    Serial.println("Gardez l'IMU à plat pour la calibration Accel/Gyro.");
    delay(8000);
    imu.calibrateAccelGyro(&calibrationData);
    Serial.println("Calibration Accel/Gyro terminée.");
    Serial.println("Accel biases X/Y/Z: ");
    Serial.print(calibrationData.accelBias[0]);
    Serial.print(", ");
    Serial.print(calibrationData.accelBias[1]);
    Serial.print(", ");
    Serial.println(calibrationData.accelBias[2]);
    Serial.println("Gyro biases X/Y/Z: ");
    Serial.print(calibrationData.gyroBias[0]);
    Serial.print(", ");
    Serial.print(calibrationData.gyroBias[1]);
    Serial.print(", ");
    Serial.println(calibrationData.gyroBias[2]);

    // Réinitialiser l'IMU avec les données calibrées
    imu.init(calibrationData, IMU_ADDRESS);

#endif
}

// Méthode pour mettre à jour les données
void IMU::update() {
    imu.update();
    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);
    complementary_filter();
}

// Méthode pour afficher les données
void IMU::printAll() {
    Serial.print("Accélération (g) : X=");
    Serial.print(accelData.accelX);
    Serial.print(" Y=");
    Serial.print(accelData.accelY);
    Serial.print(" Z=");
    Serial.println(accelData.accelZ);

    Serial.print("Gyroscope (DPS) : X=");
    Serial.print(gyroData.gyroX);
    Serial.print(" Y=");
    Serial.print(gyroData.gyroY);
    Serial.print(" Z=");
    Serial.println(gyroData.gyroZ);

    Serial.print("Angles (°) : pitch=");
    Serial.print(filtered_pitch);
    Serial.print(" roll=");
    Serial.println(filtered_roll);
}
void IMU::printAngle() {
    Serial.print("Angles (rad) : pitch=");
    Serial.print(degToRad(filtered_pitch));
    Serial.print(" roll=");
    Serial.println(degToRad(filtered_roll));
}

AccelData IMU::get_acc() const
{
    return accelData;
}

GyroData IMU::get_gyro() const
{
    return gyroData;
}

void IMU::complementary_filter()
{

unsigned long currentTime = millis(); // Temps actuel
  if (currentTime - lastUpdateTime >= dt * 1000) { // Vérifie si dt (en ms) est écoulé
    lastUpdateTime = currentTime; // Met à jour le dernier temps de cycle

    // 2. Calcul des angles via l'accéléromètre
    auto angleAccX = atan2(accelData.accelY, accelData.accelZ) * 180 / PI; // Tangage (pitch)
    auto angleAccY = atan2(-accelData.accelX, accelData.accelZ) * 180 / PI; // Roulis (roll)
    // Note : angleAccZ est inutile ici pour le lacet (yaw).

    // 3. Intégration des vitesses angulaires pour obtenir les angles gyroscopiques
    angleGyroX += gyroData.gyroX * dt; // Tangage
    angleGyroY += gyroData.gyroY * dt; // Roulis

    // 4. Application du filtre complémentaire
    filtered_pitch = alpha * (filtered_pitch + gyroData.gyroY * dt) + (1 - alpha) * angleAccY;
    filtered_roll = alpha * (filtered_roll + gyroData.gyroX * dt) + (1 - alpha) * angleAccX;
  }
}

float IMU::get_pitch_deg() const
{
    return filtered_pitch;
}

float IMU::get_roll_deg() const
{
    return filtered_roll;
}

float IMU::get_pitch_rad() const
{
    return degToRad(filtered_pitch);
}

float IMU::get_roll_rad() const
{
    return degToRad(filtered_roll);
}

float IMU::degToRad(float degrees) const {
    // Conversion des degrés en radians
    float radians = degrees * M_PI / 180.0;

    // Normalisation dans l'intervalle [-π, π]
    radians = fmod(radians, 2 * M_PI); // Rendre l'angle périodique sur [-2π, 2π]
    if (radians > M_PI) {
        radians -= 2 * M_PI; // Ajuster pour qu'il soit dans [-π, π]
    } else if (radians < -M_PI) {
        radians += 2 * M_PI; // Ajuster pour qu'il soit dans [-π, π]
    }

    return radians;
}