/*Lab 7 - PID - v1 - 28.10.21
 * This version does a return home but only using basic odometry, v6 to update to include
 * better quality odometry.

   */

#include "encoders.h"
#include "motors_v2.h"
#include "pid.h"
#include "kinematics_v2.h"

Motors_c motors;
PID_c pidl;
PID_c pidr;
PID_c pidtheta;
Kinematics_c kinematics;

//Motor parameters
int MAX_SPEED = 100;
int MIN_SPEED = -1;
float lpwm = 20;
float rpwm = 20;
float pwm = 20;

//Time parameters
unsigned long current_time = millis();
unsigned long current_time2 = millis();
unsigned long timestep = 20;
unsigned long oldtime = 0;
unsigned long timenow;

//Kinematics parameters
long countnow_e0;
long countnow_e1;
long countold_e0 = 0;
long countold_e1 = 0;
float phir;                //radians right wheel turned per timestep
float phil;                //radians left wheel turned per timestep
float omegar;              //radians per second right wheel turned per timestep
float omegal;              //radians per second left wheel turned per timestep

//Wheel parameters
int wheel_r = 32/2;        //mm wheel radius - needs measuring to confirm
int l = 43;                //mm distance between wheel centrelines - needs measuring to confirm
float cpr = 358.3;         //counts per rev from pololu documentation
float radpc = (2*PI)/cpr;  //radians per count 

//PID parameters
float speed_demand = 0;
float angle_demand = 0;
float PID_r;
float PID_l;
float PID_theta;
float K_P_motor = 0.3;
float K_I_motor = 0.001;
float K_D_motor = 0;
float K_P_theta = 1;
float K_I_theta = 0.001;
float K_D_theta = 0;
float i = 0 ;


void setup() {
  Serial.begin(9600);
  delay(500);

  motors.initiate(MIN_SPEED, MAX_SPEED);
  pidl.initiate();
  pidr.initiate();
  pidtheta.initiate();
  setupEncoder0();
  setupEncoder1();
}


void loop() {

  /*
  kinematics.updateposition();
  float homeangle = PI+(atan2(kinematics.y_global, kinematics.x_global));
  Serial.println(kinematics.x_global);
  Serial.println(kinematics.y_global);
  Serial.println(homeangle/(2*PI));
  Serial.println("");
  delay(20);
  */
  


  
  motors.setMotorPower(20,20);
  kinematics.updateposition();
  delay(1000);
  motors.setMotorPower(-20,20);
  kinematics.updateposition();
  delay(500);
  motors.setMotorPower(20,20);
  kinematics.updateposition();
  delay(1000);
  motors.setMotorPower(0,0);
  kinematics.updateposition();
  delay(500);

  kinematics.updateposition();
  float homeangle = PI+(atan2(kinematics.y_global, kinematics.x_global));
  
  lpwm = 0;
  rpwm = 0;
  pidr.resetvariables();
  pidl.resetvariables();
  pidtheta.resetvariables();
  
  turntoangle(homeangle, 0.01);

  motors.stopMotors();
  delay(4000);
  
  
}

void turntoangle (float angle, float sensitivity) {  
  kinematics.updateposition();

  while (kinematics.theta_global < (angle-sensitivity) || kinematics.theta_global > (angle+sensitivity)) {
    if (millis() - current_time > 10) {
      
      kinematics.updateposition();
      PID_theta = pidtheta.update_pid(angle, kinematics.theta_global, K_P_theta, K_I_theta, K_D_theta);
     
      updateomega();
      PID_r = pidr.update_pid(0, (omegar+PID_theta), K_P_motor, K_I_motor, K_D_motor);
      PID_l = pidl.update_pid(0, (omegal-PID_theta), K_P_motor, K_I_motor, K_D_motor);
      lpwm = lpwm + PID_l;
      rpwm = rpwm + PID_r;
      Serial.println("NOW");
      Serial.println(PID_theta);
      Serial.println(lpwm);
      motors.setMotorPower(lpwm, rpwm);
        
      current_time = millis();      
    }
  }
}

void updateomega() { 
      countnow_e0 = count_e0;
      countnow_e1 = count_e1;
      timenow = millis();
      
      phir = (countnow_e0 - countold_e0)*radpc;
      phil = (countnow_e1 - countold_e1)*radpc;      
   
      omegar = (phir / (timenow-oldtime))*1000;
      omegal = (phil / (timenow-oldtime))*1000;

      countold_e0 = countnow_e0;
      countold_e1 = countnow_e1;
      oldtime = timenow;          
}

void gostraight(float _speed) {
  angle_demand = 0;
  motors.setMotorPower(lpwm, rpwm);

  if (millis() - current_time > timestep) {
    kinematics.updateposition();
    PID_theta = pidtheta.update_pid(angle_demand, kinematics.theta_global, K_P_theta, K_I_theta, K_D_theta);
     
    updateomega();
    PID_r = pidr.update_pid(_speed, (omegar+PID_theta), K_P_motor, K_I_motor, K_D_motor);
    PID_l = pidl.update_pid(_speed, (omegal-PID_theta), K_P_motor, K_I_motor, K_D_motor);
    lpwm = lpwm + PID_l;
    rpwm = rpwm + PID_r;
        
    current_time = millis();
  }  
}

void speedplot() {
    Serial.print(omegal);
    Serial.print(",");
    Serial.print(omegar);
    Serial.print(",");
    Serial.print(pidl.D);
    Serial.print(",");
    Serial.print(10);
    Serial.print(",");
    Serial.println(-10);
}
