/*Lab 7 - PID - v1 - 28.10.21
 * 

   */

#include "encoders.h"
#include "motors_v2.h"
#include "pid.h"

Motors_c motors;
PID_c pid;

//Motor parameters
int MAX_SPEED = 100;
int MIN_SPEED = -1;
float lpwm = 20;
float rpwm = 20;

//Time parameters
unsigned long current_time = millis();
unsigned long current_time2 = millis();
unsigned long timestep = 20;
unsigned long oldtime = 0;
unsigned long timenow;

//Encoder parameters
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
float K_p = 0.25;
float demand = 20;
float p_r;
float p_l;

void setup() {
  Serial.begin(9600);
  delay(500);

  motors.initiate(MIN_SPEED, MAX_SPEED);
  pid.initiate();
  setupEncoder0();
  setupEncoder1();
}


void loop(){

  motors.setMotorPower(lpwm, rpwm);
  Serial.println(omegal);
  Serial.println(omegar);

  if (millis() - current_time > timestep) {
    updateomega();
    p_r = pid.update_p(demand, omegar, K_p);
    p_l = pid.update_p(demand, omegal, K_p);
    lpwm = lpwm + p_l;
    rpwm = rpwm + p_r;
    current_time = millis();
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
