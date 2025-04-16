#ifndef MyPID_H
#define MyPID_H

class MyPID {
public:
    // Constructeur pour initialiser les gains et la direction du MyPID
    MyPID(double Kp, double Ki, double Kd, double *input, double *output, double *setpoint);

    // Fonction pour configurer l'intervalle de calcul du MyPID (en ms)
    void SetSampleTime(unsigned long time);

    // Fonctions pour configurer les coefficients du MyPID
    double GetKp();
    double GetKd();
    double GetKi();
    void SetKp(double newKp);
    void SetKd(double newKd);
    void SetKi(double newKi);

    double Get_derivative();

    // Fonction pour calculer la sortie MyPID
    void Compute();

private:
    double Kp, Ki, Kd;      // Gains MyPID
    double *input, *output, *setpoint;  // Variables d'entrée, de sortie et de setpoint
    double lastError;       // Dernière erreur calculée
    double integral;        // Somme de l'intégrale
    unsigned long lastTime; // Dernier temps de calcul
    unsigned long sampleTime; // Temps d'échantillonnage en us
    double derivative; // Erreur dérivée
    double lastDerivative;
};

#endif // MyPID_H
