/*Follow the Line!
   Version 1 Solution to the line-following challenge updated to test turning on the spot
   11/11/2021
   Henry Hickson
*/

//Include relevent class files
#include "encoders.h"
#include "motors_v2.h"
#include "pid.h"
#include "kinematics_v2.h"
#include "IRSense.h"

//Setup relevent classes
Motors_c motors;
PID_c pidl;
PID_c pidr;
PID_c pidtheta;
PID_c pidline;
Kinematics_c kinematics;
IRSense linesensor;

//Motor parameters
int MAX_SPEED = 200;
int MIN_SPEED = -1;
float pwm_norm = 15;
float lpwm = 15;
float rpwm = 15;
int targetspeed = 4;

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
float omegal;              //radians per second left wheel turned per timestep

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

//PID motor
float K_P_motor = 0.3;
float K_I_motor = 0.001;
float K_D_motor = 50;

//PID angle
float K_P_theta = 1;
float K_I_theta = 0.001;
float K_D_theta = 200;

//PID line follow
float K_P_line = 30;
float K_I_line = 0.01;
float K_D_line = 0;

//Linesensor Parameters
byte sensor_select = 3;
byte EMIT_STATE = 1;
int IR_time_out = 5000;
float IR_output[5];
bool line_stat = false;
float gain = 50;
float IR_sensitivity = 0.01;

//FSM Parameters
int state = 1;
int line_gap = 12; //Length of the gap in the line challenge
float max_x_global = 1000; //Distance that the end is from the start
float x_local = 0;

//Reading values for testing angles
float x;
float y;
float theta;

void setup() {
  Serial.begin(9600);
  //while(!Serial);

  //Initiate classes
  motors.initiate(MIN_SPEED, MAX_SPEED);

  pidl.initiate();
  pidr.initiate();
  pidtheta.initiate();
  pidline.initiate();

  setupEncoder0();
  setupEncoder1();

  resetpid();
}

void loop() {
  if (millis() - current_time > timestep) {
    if (state == 1) {
      turntoangle(2*PI, 0.01);
      motors.stopMotors();
      kinematics.updateposition();
      x = kinematics.x_global;
      y = kinematics.y_global;
      theta = PI - kinematics.theta;
      state = 2;
      Serial.println("state complete!!");
      delay(1000);
    }
    if (state == 2) {
      Serial.println(x);
      Serial.println(y);
      Serial.println(theta);
      Serial.println("");
      delay(1000);
    }

   current_time = millis();
  }
}

void updatestate() {
  /*A function to update the Finite State Machine for the line following challenge.
    State 1: Find the line
    State 2: Follow the line
    State 3: Lost the line - re-find the line
    State 4: Lost the line - go home
    State 5: Found home - stop
  */
  if (state == 1) {
    findtheline();
    
  } else if (state == 2) {
    followtheline(gain, IR_sensitivity);    
    linesensor.IRread(IR_output);
    
    if (IR_output[1] < 0.1 && IR_output[2] < 0.2 && IR_output[3] < 0.2 && kinematics.x_global < max_x_global) {
      resetpid();
      x_local = 0;
      state = 3;
    } else if (IR_output[1] < 0.2 && IR_output[2] < 0.2 && IR_output[3] < 0.2 && kinematics.x_global >= max_x_global) {
      resetpid();
      state = 4;
    }
    
  } else if (state == 3) {    
      findtheline();
      kinematics.updateposition();   
      x_local = x_local + kinematics.x_local;

    if (x_local > line_gap) {
      kinematics.updateposition();
      turntoangle(kinematics.theta - PI, 0.05);
      state = 1;
    }

  } else if (state == 4) {
    kinematics.updateposition();
    float homeangle = PI + (atan2(kinematics.y_global, kinematics.x_global));
    
    motors.stopMotors();
    resetpid();
    turntoangle(homeangle, 0.01);
    lpwm = pwm_norm;
    rpwm = pwm_norm;
    motors.setMotorPower(pwm_norm, pwm_norm);

    resetpid();
    state = 5;
    
  } else if  (state == 5) {
    while (kinematics.x_global > 0) {
      kinematics.updateposition();
      moveforward(targetspeed+10);
      delay(20);
    }
    motors.stopMotors();
  }

}

void findtheline() {
  /* Function to find the line in the initial instance
  */
  moveforward(targetspeed);
  linesensor.IRread(IR_output);
  if (IR_output[1] > 0.3 || IR_output[2] > 0.3 || IR_output[3] > 0.3) {
    followtheline(gain, IR_sensitivity);
    state = 2;
  }  
}

void moveforward(int omega_target) {
  /* Function that uses motor PID controllers to ensure movement in a straight line
      Depenencies:
      - motors_v2.h
      - PID.h
  */
  updateomega();
  PID_r = pidr.update_pid(omega_target, omegar, K_P_motor, K_I_motor, K_D_motor);
  PID_l = pidl.update_pid(omega_target, omegal, K_P_motor, K_I_motor, K_D_motor);
  lpwm = lpwm + PID_l;
  rpwm = rpwm + PID_r;
  motors.setMotorPower(lpwm, rpwm);
}

void followtheline(float gain, float IR_sensitivity) {
  /* Function to follow a dark line on a lighter surface.
     The function identifies when the robot is moving away from the line, and adjusts
     motor power to reposition. Forward movement is included. pwm_norm must be set globally.
     Dependancies:
     - IRSense.h
     - motors_v2.h
     - PID.h
  */
  float e_l = linesensor.eline(); //weighted error from left to right side
  updateomega();
  PID_line = pidline.update_pid(0, e_l, K_P_line, K_I_motor, K_D_motor);
  PID_r = pidr.update_pid(targetspeed + PID_line*targetspeed, omegar, K_P_motor, K_I_motor, K_D_motor);
  PID_l = pidl.update_pid(targetspeed - PID_line*targetspeed, omegal, K_P_motor, K_I_motor, K_D_motor);
  rpwm = PID_r;
  lpwm = PID_l;
  motors.setMotorPower(lpwm, rpwm);
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
      Serial.println(PID_theta);
      
      updateomega();
      PID_r = pidr.update_pid(0, (omegar + PID_theta), K_P_motor, K_I_motor, K_D_motor);
      PID_l = pidl.update_pid(0, (omegal - PID_theta), K_P_motor, K_I_motor, K_D_motor);
      lpwm = lpwm + PID_l;
      rpwm = rpwm + PID_r;
      motors.setMotorPower(lpwm, rpwm);

      current_time = millis();
    }
  }
}

void updateomega() {
  countnow_e0 = count_e0;
  countnow_e1 = count_e1;
  unsigned long timenow = millis();

  phir = (countnow_e0 - countold_e0) * radpc;
  phil = (countnow_e1 - countold_e1) * radpc;
  omegar = (phir / (timenow - oldtime)) * 1000;
  omegal = (phil / (timenow - oldtime)) * 1000;

  countold_e0 = countnow_e0;
  countold_e1 = countnow_e1;
  oldtime = timenow;
}

void resetpid() {
  pidtheta.resetvariables();
  pidr.resetvariables();
  pidl.resetvariables();
  lpwm = 0;
  rpwm = 0;
}
