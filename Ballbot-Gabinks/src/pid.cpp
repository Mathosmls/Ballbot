#include "pid.hpp"
#include <Arduino.h>

// Constructeur pour initialiser les gains et la direction du MyPID
MyPID::MyPID(double Kp, double Ki, double Kd, double *input, double *output, double *setpoint)
    : Kp(Kp), Ki(Ki), Kd(Kd), input(input), output(output), setpoint(setpoint), lastError(0.0), integral(0.0), sampleTime(5000) {
    lastTime = micros();
}

// Fonction pour configurer l'intervalle de calcul du MyPID (en us)
void MyPID::SetSampleTime(unsigned long time) {
    sampleTime = time;
}

double MyPID::GetKp() {
    return Kp;
}

double MyPID::GetKd() {
    return Kd;
}
 
double MyPID::GetKi() {
    return Ki;
}

void MyPID::SetKp(double newKp){
    Kp = newKp;
}

void MyPID::SetKd(double newKd){
    Kd = newKd;
}

void MyPID::SetKi(double newKi){
    Ki = newKi;
}

double MyPID::Get_derivative() {
    return derivative;
}

// Fonction pour calculer la sortie MyPID
void MyPID::Compute() {
    unsigned long currentTime = micros();
    unsigned long timeChange = currentTime - lastTime;

    if (timeChange >= sampleTime) {
        // Calcul de l'erreur
        double error = *setpoint - *input;
        // if (abs(error)<radians(0.5))
        // {
        //     error=0.;
        // }

        // Calcul de l'intégrale et de la dérivée
        integral += error * (timeChange / 1000000.0);
        derivative = (error - lastError) / (timeChange / 1000000.0);
        derivative = 0.6 * lastDerivative + 0.4 * derivative; // Filtre simple
        lastDerivative = derivative;

        // Calcul de la sortie MyPID
        *output = Kp * error + Ki * integral + Kd * derivative;

        // Sauvegarder les valeurs pour la prochaine itération
        lastError = error;
        lastTime = currentTime;
    }
}