#include "imu.hpp"
#include <Arduino.h>
//IMU
//----------------------------------------

IMU myIMU(500,2);


void setup() {
    Serial.println("simple_example_9050 started...");
    Serial.begin(115200);
    Serial.println("Setup");
    if (!myIMU.init()) {
            Serial.println("Échec de l'initialisation de l'IMU.");
        }
    else 
    {
        Serial.println("IMU ready");
    }

    // myIMU.calibrate();
}

void loop() {
    static int i = 0;
    
    myIMU.update();
    // Serial.println("boucle");
    if (i == 100) {
        myIMU.printAll();
        Serial.print(">roll:");
        Serial.println(myIMU.get_roll_deg());
        Serial.print(">pitch:");
        Serial.println(myIMU.get_pitch_deg());
        // myIMU.printAngle();
        i = 0;
    } else {
        i++;
    }
}





