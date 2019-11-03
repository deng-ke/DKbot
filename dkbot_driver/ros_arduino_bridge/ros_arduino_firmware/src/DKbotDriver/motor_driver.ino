/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

/* Include the Pololu library */
#include "Make4e2ndChassis.h"

/* Create the motor driver object */
Make4e2ndChassis drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
  Serial.println("init Make4e2ndChassis");
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) 
  {
    drive.setM1Speed(spd);
  }
  else
  {
    drive.setM2Speed(spd);
  }
 
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
//  Serial.print("leftPWM:");
//  Serial.print(leftSpeed);
//  Serial.print(" rightPWM:");
//  Serial.println(rightSpeed);
  
}


