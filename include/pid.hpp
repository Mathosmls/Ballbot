#ifndef MyPID_H
#define MyPID_H

class MyPID {
public:
    // Constructeur pour initialiser les gains et la direction du MyPID
    MyPID(double Kp, double Ki, double Kd, double *input, double *output, double *setpoint);

    // Fonction pour configurer l'intervalle de calcul du MyPID (en ms)
    void SetSampleTime(unsigned long time);

    // Fonction pour calculer la sortie MyPID
    void Compute();

private:
    double Kp, Ki, Kd;      // Gains MyPID
    double *input, *output, *setpoint;  // Variables d'entrée, de sortie et de setpoint
    double lastError;       // Dernière erreur calculée
    double integral;        // Somme de l'intégrale
    unsigned long lastTime; // Dernier temps de calcul
    unsigned long sampleTime; // Temps d'échantillonnage en ms
};

#endif // MyPID_H
