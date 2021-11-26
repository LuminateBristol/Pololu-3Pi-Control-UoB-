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
#include "kinematics.h"

Kinematics_c kinematics;
int timestep = 100;
unsigned long current_time = millis();
unsigned long elapsed_time;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  setupEncoder0();
  setupEncoder1();
}

void loop() {
  elapsed_time = millis() - current_time;
  if (elapsed_time >= timestep) {
    kinematics.updateposition();
    current_time = millis();
    Serial.println("");
    Serial.println(kinematics.x_global);
    Serial.println(kinematics.y_global);
    Serial.println(kinematics.theta_global); //This is an alternative to using pointer arrays to get
                                             //*mulitple* arguments from a library function
                                             
                                             //The neater option would be define an array in this file
                                             //then use a pointer to pass it back and forth between
                                             //here and the library. More complex though.
  }  
}
