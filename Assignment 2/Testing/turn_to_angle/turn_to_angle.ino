

//Include relevent class files
#include "encoders.h"
#include "motors_v2.h"
#include "pid.h"
#include "kinematics_v2.h"

//Setup relevent classes
Motors_c motors;
PID_c pidl;
PID_c pidr;
PID_c pidtheta;
Kinematics_c kinematics;

//Motor parameters
int MAX_SPEED = 100;
int MIN_SPEED = -1;

//Time parameters
unsigned long current_time = millis();
unsigned long timestep = 20;
unsigned long oldtime = 0;

//Kinematics parameters
long countnow_e0;
long countnow_e1;
long countold_e0 = 0;
long countold_e1 = 0;
float phir;                //radians right wheel turned per timestep
float phil;                //radians left wheel turned per timestep
float omegar;              //radians per second right wheel turned per timestep
float omegal;              //radians per second left wheel turned per 

//Wheel parameters
int wheel_r = 32 / 2;      //mm wheel radius - needs measuring to confirm
int l = 43;                //mm distance between wheel centrelines - needs measuring to confirm
float cpr = 358.3;         //counts per rev from pololu documentation
float radpc = (2 * PI) / cpr; //radians per count

//PID parameters
float speed_demand = 0;
float angle_demand = 0;
float PID_r;
float PID_l;
float PID_theta;
float PID_line;
float K_P_motor = 0.3;
float K_I_motor = 0.001;
float K_D_motor = 10;
float K_P_theta = 2;
float K_I_theta = 0.001;
float K_D_theta = 1;

//FSM setup
int state = 0;


void setup() {
  Serial.begin(9600);

  //Initiate classes
  motors.initiate(MIN_SPEED, MAX_SPEED);

  pidl.initiate();
  pidr.initiate();
  pidtheta.initiate();

  setupEncoder0();
  setupEncoder1();
}

void loop() {
  if (millis() - current_time > timestep) {
    
    turntoangle(PI, 0.01);
    current_time = millis();
  }
}

void turntoangle(float angle, float sensitivity) {
  /* Function to turn to a specific angle in the global coordinate system.
     This function matches an angle (in radians) to a given sensitivity.
     Dependancies:
     - kinematics_v2.h
     - motors_v2.h
     - PID.h - three PID controllers needed - heading, left wheel, right wheel
  */
  kinematics.updateposition();
  
  while (kinematics.theta < (angle - sensitivity) || kinematics.theta > (angle + sensitivity)) {
    if (millis() - current_time > 20) {
      kinematics.updateposition();
      PID_theta = pidtheta.update_pid(angle, kinematics.theta, K_P_theta, K_I_theta, K_D_theta);

      updateomega();
      PID_r = pidr.update_pid(0, (omegar + PID_theta), K_P_motor, K_I_motor, K_D_motor);
      PID_l = pidl.update_pid(0, (omegal - PID_theta), K_P_motor, K_I_motor, K_D_motor);
      lpwm = lpwm + PID_l;
      rpwm = rpwm + PID_r;
      motors.setMotorPower(lpwm, rpwm);

      current_time = millis();
    }
  }
  lpwm = pwm_norm;
  rpwm = pwm_norm;
}
