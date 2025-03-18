// #include <Arduino.h>
// #include <SBUS.h>

// // Définition du port série SBUS (RX sur Pin 0 de la Teensy 4.1)
// SBUS sbus(Serial1);

// // Variables SBUS
// uint16_t channels[16];  // Tableau pour stocker les 16 canaux SBUS
// bool failsafe = false;
// bool lostFrame = false;

// // Paramètres de conversion
// const float joystickMin = 172;   // Valeur SBUS joystick au minimum
// const float joystickMax = 1811;  // Valeur SBUS joystick au maximum
// const float angleMin = 0.0;      // Angle minimum (0 rad)
// const float angleMax = PI / 18;  // Angle maximum (pi/18 rad)

// // Canal utilisé pour récupérer la valeur du joystick droit (axe vertical)
// const int CHANNEL_JOYSTICK = 2;  // En général, canal 2 = pitch (joystick droit, axe vertical)

// void setup() {
//     Serial.begin(115200);
//     sbus.begin();
//     Serial.println("Taranis Q X7 SBUS -> Angle Mapping Ready!");
// }

// void loop() {
//     // Lire les données SBUS
//     if (sbus.read(&channels[0], &failsafe, &lostFrame)) {
//         // Lire la valeur du joystick droit (axe vertical)
//         float joystickValue = channels[CHANNEL_JOYSTICK];

//         // Normalisation entre 0 et 1
//         float normalizedValue = (joystickValue - joystickMin) / (joystickMax - joystickMin);
//         normalizedValue = constrain(normalizedValue, 0.0, 1.0); // Clamp entre 0 et 1

//         // Mapping vers l'angle entre 0 et pi/18
//         float targetAngle = angleMin + normalizedValue * (angleMax - angleMin);

//         // Affichage des données
//         Serial.print("Joystick Value: "); Serial.print(joystickValue);
//         Serial.print(" -> Normalized: "); Serial.print(normalizedValue);
//         Serial.print(" -> Target Angle: "); Serial.println(targetAngle);
//     }

//     delay(20);  // 50Hz (Fréquence SBUS)
// }
