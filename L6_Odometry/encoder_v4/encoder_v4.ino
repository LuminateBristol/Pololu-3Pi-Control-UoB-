/*Lab 6 - Odometry - 21/10/21

   NOTES:
   This lab is based on a pre-written encoder source file:
   - The encoder source file sets up the encoders and counts the encoder counts per revolution
   - It then uses the counts to create a  binary number called a decimal, the decimal
     relates to the "state" of the encoder.
   - At each state, the count either does nothing, increases, or decreases
   - The binary number is created using bitwise operations which are a function of the
     encoder registering a forward and/or backward movement
   - e0 is RHS wheel, e1 is LHS wheel

   Kinematics:
   - The conversion from wheel rotations to actual robot position in a global coordinate system
*/

#include "encoders.h"
#include "kinematics_v3.h"
#include "motors_v2.h"

Motors_c motors;
int MIN_SPEED = 0;
int MAX_SPEED = 50;
int pwm = 25;
int lpwm = pwm;
int rpwm = pwm;

Kinematics_c kinematics;
int timestep = 100;
unsigned long current_time = millis();
unsigned long elapsed_time;

void setup() {
  Serial.begin(9600);
 // while(!Serial);

  motors.initiate(MIN_SPEED, MAX_SPEED);

  setupEncoder0();
  setupEncoder1();
}

void loop() {
  kinematics.updateposition();
  Serial.println(kinematics.theta);
  Serial.println(kinematics.x_global);
  Serial.println(kinematics.y_global); 
  Serial.println("");
  delay(20);

 
}
