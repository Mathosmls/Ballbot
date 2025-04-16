#include "imu.hpp"

float normalizeAngle(float angle)
{
    // Normalisation de l'angle entre -pi et pi
    angle = fmod(angle, 2 * M_PI); // Ramène l'angle dans l'intervalle [-2pi, 2pi]
    if (angle > M_PI)
    {
        angle -= 2 * M_PI; // Si l'angle est supérieur à pi, le ramener dans l'intervalle [-pi, pi]
    }
    else if (angle < -M_PI)
    {
        angle += 2 * M_PI; // Si l'angle est inférieur à -pi, le ramener dans l'intervalle [-pi, pi]
    }
    return angle;
}

// Constructeur
IMU::IMU(int gyro_range, int acc_range) : _gyro_range(gyro_range), _acc_range(acc_range)
{
    memset(&calibrationData, 0, sizeof(calData));
}

// Méthode d'initialisation
bool IMU::init()
{
    Wire.begin();
    Wire.setClock(400000); // Horloge I2C à 400 kHz

    int err = imu.init(calibrationData, IMU_ADDRESS);
    if (err != 0)
    {
        Serial.print("Erreur d'initialisation de l'IMU : ");
        Serial.println(err);
        return false;
    }

    Serial.println("IMU initialisée avec succès.");

    // Configuration des plages
    err = imu.setGyroRange(_gyro_range);
    err |= imu.setAccelRange(_acc_range);
    if (err != 0)
    {
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
void IMU::calibrate()
{

    int max_mes = 200;
    Serial.println("Phase de stabilisation du filtre (gyro,acc) (0.2 sec)...");
    for (int j = 0; j < max_mes; j++)
    {
        imu.update();
    }
    for (int j = 0; j < max_mes; j++)
    {
        imu.update();
        imu.getAccel(&accelData);
        imu.getGyro(&gyroData);

        bias_accelData.accelX += accelData.accelX;
        bias_accelData.accelY += accelData.accelY;
        bias_accelData.accelZ += accelData.accelZ - 1.;
        bias_gyroData.gyroX += gyroData.gyroX;
        bias_gyroData.gyroY += gyroData.gyroY;
        bias_gyroData.gyroZ += gyroData.gyroZ;

        // delay(1); // 2 secondes de chauffe à ~1 kHz
    }
    bias_accelData.accelX /= max_mes;
    bias_accelData.accelY /= max_mes;
    bias_accelData.accelZ /= max_mes;
    bias_gyroData.gyroX /= max_mes;
    bias_gyroData.gyroY /= max_mes;
    bias_gyroData.gyroZ /= max_mes;

    max_mes = 800;
    Serial.println("Phase de stabilisation (roll, pitch)du filtre (2 sec)...");
    for (int j = 0; j < max_mes; j++)
    {
        update_all();
    }
    max_mes=100;
    Serial.println("Mesure des offsets...");
    for (int j = 0; j < max_mes; j++)
    {
        update_all();
        bias_roll_rad += get_roll_rad();
        bias_pitch_rad += get_pitch_rad();
    }

    bias_roll_rad /= max_mes;
    bias_pitch_rad /= max_mes;
}
void IMU::apply_offset()
{
    accelData.accelX -= bias_accelData.accelX;
    accelData.accelY -= bias_accelData.accelY;
    accelData.accelZ -= bias_accelData.accelZ;

    gyroData.gyroX -= bias_gyroData.gyroX;
    gyroData.gyroY -= bias_gyroData.gyroY;
    gyroData.gyroZ -= bias_gyroData.gyroZ;
}
// Méthode pour mettre à jour les données
void IMU::update_all()
{
    imu.update();
    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);
    apply_offset();
    complementary_filter();
    // kalman_filter(); // ← nouveau
    // madgwick_filter();
}

// Méthode pour afficher les données
void IMU::printAll()
{
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
    Serial.println();
}
void IMU::printAngle()
{
    Serial.print("Angles (deg) : pitch=");
    Serial.print((filtered_pitch));
    Serial.print(" roll=");
    Serial.println((filtered_roll));
    Serial.println();
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
    if (currentTime - lastUpdateTime >= dt * 1000)
    {
        double real_dt = (currentTime - lastUpdateTime) / 1000.; // Vérifie si dt (en ms) est écoulé
        lastUpdateTime = currentTime;                            // Met à jour le dernier temps de cycle

        // 2. Calcul des angles via l'accéléromètre
        auto angleAccX = atan2(accelData.accelY, accelData.accelZ) * 180 / PI;  // Tangage (pitch)
        auto angleAccY = atan2(-accelData.accelX, accelData.accelZ) * 180 / PI; // Roulis (roll)
        // Note : angleAccZ est inutile ici pour le lacet (yaw).

        // 4. Application du filtre complémentaire
        filtered_pitch = alpha * (filtered_pitch + gyroData.gyroY * real_dt) + (1 - alpha) * angleAccY-bias_pitch_rad;
        filtered_roll = alpha * (filtered_roll + gyroData.gyroX * real_dt) + (1 - alpha) * angleAccX-bias_roll_rad;
        normalizeAngle(filtered_pitch);
        normalizeAngle(filtered_roll);
    }
}

double IMU::get_pitch_deg() const
{
    return filtered_pitch;
}

double IMU::get_roll_deg() const
{
    return filtered_roll;
}

double IMU::get_pitch_rad() const
{
    return degToRad(filtered_pitch);
}

double IMU::get_roll_rad() const
{
    return degToRad(filtered_roll);
}

double IMU::degToRad(double degrees) const
{
    // Conversion des degrés en radians
    double radians = degrees * M_PI / 180.0;

    // Normalisation dans l'intervalle [-π, π]
    radians = fmod(radians, 2 * M_PI); // Rendre l'angle périodique sur [-2π, 2π]
    if (radians > M_PI)
    {
        radians -= 2 * M_PI; // Ajuster pour qu'il soit dans [-π, π]
    }
    else if (radians < -M_PI)
    {
        radians += 2 * M_PI; // Ajuster pour qu'il soit dans [-π, π]
    }

    return radians;
}

void IMU::kalman_filter()
{
    unsigned long currentTime = millis(); // Temps actuel
    if (currentTime - lastUpdateTime_kalman >= dt * 1000)
    { // Vérifie si dt (en ms) est écoulé
        double real_dt = (currentTime - lastUpdateTime) / 1000.;
        lastUpdateTime_kalman = currentTime;
        float angleAccPitch = atan2(-accelData.accelX, accelData.accelZ) * 180 / PI;
        float angleAccRoll = atan2(accelData.accelY, accelData.accelZ) * 180 / PI;

        // === PITCH ===
        // 1. Prediction
        float rate_pitch = gyroData.gyroY - kalman_bias_pitch;
        kalman_pitch += real_dt * rate_pitch;

        P_pitch[0][0] += real_dt * (real_dt * P_pitch[1][1] - P_pitch[1][0] - P_pitch[0][1] + Q_angle);
        P_pitch[0][1] -= real_dt * P_pitch[1][1];
        P_pitch[1][0] -= real_dt * P_pitch[1][1];
        P_pitch[1][1] += Q_bias * real_dt;

        // 2. Update
        float y_pitch = angleAccPitch - kalman_pitch;
        float S_pitch = P_pitch[0][0] + R_measure;
        float K_pitch[2];
        K_pitch[0] = P_pitch[0][0] / S_pitch;
        K_pitch[1] = P_pitch[1][0] / S_pitch;

        kalman_pitch += K_pitch[0] * y_pitch;
        kalman_bias_pitch += K_pitch[1] * y_pitch;

        float P00_temp = P_pitch[0][0], P01_temp = P_pitch[0][1];

        P_pitch[0][0] -= K_pitch[0] * P00_temp;
        P_pitch[0][1] -= K_pitch[0] * P01_temp;
        P_pitch[1][0] -= K_pitch[1] * P00_temp;
        P_pitch[1][1] -= K_pitch[1] * P01_temp;

        // === ROLL ===
        float rate_roll = gyroData.gyroX - kalman_bias_roll;
        kalman_roll += real_dt * rate_roll;

        P_roll[0][0] += real_dt * (real_dt * P_roll[1][1] - P_roll[1][0] - P_roll[0][1] + Q_angle);
        P_roll[0][1] -= real_dt * P_roll[1][1];
        P_roll[1][0] -= real_dt * P_roll[1][1];
        P_roll[1][1] += Q_bias * real_dt;

        float y_roll = angleAccRoll - kalman_roll;
        float S_roll = P_roll[0][0] + R_measure;
        float K_roll[2];
        K_roll[0] = P_roll[0][0] / S_roll;
        K_roll[1] = P_roll[1][0] / S_roll;

        kalman_roll += K_roll[0] * y_roll;
        kalman_bias_roll += K_roll[1] * y_roll;

        float P00_temp_r = P_roll[0][0], P01_temp_r = P_roll[0][1];

        P_roll[0][0] -= K_roll[0] * P00_temp_r;
        P_roll[0][1] -= K_roll[0] * P01_temp_r;
        P_roll[1][0] -= K_roll[1] * P00_temp_r;
        P_roll[1][1] -= K_roll[1] * P01_temp_r;
    }
}

double IMU::get_kalman_pitch_deg() const
{
    return kalman_pitch;
}

double IMU::get_kalman_roll_deg() const
{
    return kalman_roll;
}

void IMU::madgwick_filter()
{
    unsigned long currentTime = millis(); // Temps actuel
    if (currentTime - lastUpdateTime_mad >= dt * 1000)
    {
        double real_dt = (currentTime - lastUpdateTime) / 1000.;
        lastUpdateTime_mad = currentTime; // Met à jour le dernier temps de cycle
        float ax = accelData.accelX;
        float ay = accelData.accelY;
        float az = accelData.accelZ;
        float gx = gyroData.gyroX * DEG_TO_RAD;
        float gy = gyroData.gyroY * DEG_TO_RAD;
        float gz = gyroData.gyroZ * DEG_TO_RAD;

        float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
        float norm;                                               // vector norm
        float f1, f2, f3;                                         // objetive funcyion elements
        float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
        float qDot1, qDot2, qDot3, qDot4;
        float hatDot1, hatDot2, hatDot3, hatDot4;
        float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz; // gyro bias error

        // Auxiliary variables to avoid repeated arithmetic
        float _halfq1 = 0.5f * q1;
        float _halfq2 = 0.5f * q2;
        float _halfq3 = 0.5f * q3;
        float _halfq4 = 0.5f * q4;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _2q4 = 2.0f * q4;
        float _2q1q3 = 2.0f * q1 * q3;
        float _2q3q4 = 2.0f * q3 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f)
            return; // handle NaN
        norm = 1.0f / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Compute the objective function and Jacobian
        f1 = _2q2 * q4 - _2q1 * q3 - ax;
        f2 = _2q1 * q2 + _2q3 * q4 - ay;
        f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
        J_11or24 = _2q3;
        J_12or23 = _2q4;
        J_13or22 = _2q1;
        J_14or21 = _2q2;
        J_32 = 2.0f * J_14or21;
        J_33 = 2.0f * J_11or24;

        // Compute the gradient (matrix multiplication)
        hatDot1 = J_14or21 * f2 - J_11or24 * f1;
        hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
        hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
        hatDot4 = J_14or21 * f1 + J_11or24 * f2;

        // Normalize the gradient
        norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
        hatDot1 /= norm;
        hatDot2 /= norm;
        hatDot3 /= norm;
        hatDot4 /= norm;

        // Compute estimated gyroscope biases
        gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
        gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
        gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

        // Compute and remove gyroscope biases
        gbiasx += gerrx * real_dt * zeta;
        gbiasy += gerry * real_dt * zeta;
        gbiasz += gerrz * real_dt * zeta;
        gx -= gbiasx;
        gy -= gbiasy;
        gz -= gbiasz;

        // Compute the quaternion derivative
        qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
        qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
        qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
        qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

        // Compute then integrate estimated quaternion derivative
        q1 += (qDot1 - (beta * hatDot1)) * real_dt;
        q2 += (qDot2 - (beta * hatDot2)) * real_dt;
        q3 += (qDot3 - (beta * hatDot3)) * real_dt;
        q4 += (qDot4 - (beta * hatDot4)) * real_dt;

        // Normalize the quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;
        madgwick_pitch = atan2(2.0f * (q1 * q2 + q3 * q4), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
        madgwick_roll = asin(2.0f * (q1 * q3 - q4 * q2)) * RAD_TO_DEG;
    }
}

double IMU::get_madgwick_pitch_deg() const
{
    return madgwick_pitch;
}

double IMU::get_madgwick_roll_deg() const
{
    return madgwick_roll;
}