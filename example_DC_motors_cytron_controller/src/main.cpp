

// /*******************************************************************************
//  * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
//  * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
//  * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
//  * DAMAGES, FOR ANY REASON WHATSOEVER.
//  ********************************************************************************
//  * DESCRIPTION:
//  *
//  * This example shows how to drive 2 motors using the PWM and DIR pins with
//  * 2-channel motor driver.
//  * 
//  * 
//  * CONNECTIONS:
//  * 
//  * Arduino D3  - Motor Driver PWM 1 Input
//  * Arduino D4  - Motor Driver DIR 1 Input
//  * Arduino D9  - Motor Driver PWM 2 Input
//  * Arduino D10 - Motor Driver DIR 2 Input
//  * Arduino GND - Motor Driver GND
//  *
//  *
//  * AUTHOR   : Kong Wai Weng
//  * COMPANY  : Cytron Technologies Sdn Bhd
//  * WEBSITE  : www.cytron.io
//  * EMAIL    : support@cytron.io
//  *
//  *******************************************************************************/

        
#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motor1(PWM_DIR, 12 ,10);  // PWM 1 = Pin 12, DIR 1 = Pin 10.
CytronMD motor2(PWM_DIR, 8, 9); // PWM 2 = Pin 8, DIR 2 = Pin 9.
CytronMD motor3(PWM_DIR, 7, 11); // PWM 2 = Pin 7, DIR 2 = Pin 11.

// The setup routine runs once when you press reset.
void setup() {
  Serial.begin(115200);
}


// The loop routine runs over and over again forever.
void loop() {
  motor1.setSpeed(50);   // -225 to 255.
  motor2.setSpeed(50);  
  motor3.setSpeed(50);  
  Serial.println("slow");
  delay(5000);
  
  motor1.setSpeed(255);   
  motor2.setSpeed(255);  
  motor3.setSpeed(255);  
  Serial.println("fast");
  delay(5000);

  motor1.setSpeed(0);     
  motor2.setSpeed(0);     
  motor3.setSpeed(0);  
  Serial.println("first stop");
  delay(5000);

  motor1.setSpeed(-50);  
  motor2.setSpeed(-50);   
  motor3.setSpeed(-50);  
  Serial.println("backward slow");
  delay(5000);
  
  motor1.setSpeed(-255);  
  motor2.setSpeed(-255);   
  motor3.setSpeed(-255);  
  Serial.println("backward fast");
  delay(5000);

  motor1.setSpeed(0);     
  motor2.setSpeed(0);     
  motor3.setSpeed(0);  
  Serial.println("last stop");
  delay(5000);
}
