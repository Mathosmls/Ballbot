#include "pid.hpp"
#include <Arduino.h>

// Constructeur pour initialiser les gains et la direction du MyPID
MyPID::MyPID(double Kp, double Ki, double Kd, double *input, double *output, double *setpoint)
    : Kp(Kp), Ki(Ki), Kd(Kd), input(input), output(output), setpoint(setpoint), lastError(0.0), integral(0.0), sampleTime(100) {
    lastTime = millis();
}

// Fonction pour configurer l'intervalle de calcul du MyPID (en ms)
void MyPID::SetSampleTime(unsigned long time) {
    sampleTime = time;
}

// Fonction pour calculer la sortie MyPID
void MyPID::Compute() {
    unsigned long currentTime = millis();
    unsigned long timeChange = currentTime - lastTime;

    if (timeChange >= sampleTime) {
        // Calcul de l'erreur
        double error = *setpoint - *input;

        // Calcul de l'intégrale et de la dérivée
        integral += error * (timeChange / 1000.0);
        double derivative = (error - lastError) / (timeChange / 1000.0);

        // Calcul de la sortie MyPID
        *output = Kp * error + Ki * integral + Kd * derivative;

        // Sauvegarder les valeurs pour la prochaine itération
        lastError = error;
        lastTime = currentTime;
    }
}
