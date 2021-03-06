/*Robotic Systems Lab 4 - Line Following
 * This is a weighted controller method.
 * 
 * Initial Calibration values:
 * Black line: >=700
 * White paper: <=700
 */

#include "IRSense.h"
#include "motors_v2.h"

//Setup IR Sensors
byte sensor_select = 2;
byte EMIT_STATE = 1;
int time_out = 5000;
int IR_output[5];
float IR_sensitivity = 0.05;
unsigned long ls_ts;
unsigned long LS_MAX_TIME = 10;
bool line_stat = false;
IRSense linesensor;

//Setup motors
int MIN_SPEED = 0;
int MAX_SPEED = 200;
float pwm_norm = 20;
float gain = 150;

Motors_c motors;

void setup() { 
  Serial.begin(9600);
  Serial.println("ready");
         
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
  motors.initiate(MIN_SPEED, MAX_SPEED);
}

void loop() {
  unsigned long loop_time = micros();  
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - ls_ts;

  while (line_stat == false) {
    motors.setMotorPower(pwm_norm, pwm_norm);
    linesensor.IRread(IR_output);
    linestatus();
  }

  if (elapsed_time > LS_MAX_TIME) {
    lineFollowingBehaviour(gain, IR_sensitivity); 
    ls_ts = millis();
  }    
}

void linestatus() {
  if (IR_output[1] > 700 || IR_output[2] > 700 || IR_output[3] > 700) {
    line_stat = true;
  }
}

float eline() {
  linesensor.IRread(IR_output);
  int total_activation = IR_output[1] + IR_output[2] + IR_output[3]; //Total of three foward sensor readings
  float w_L = (IR_output[1] + (IR_output[2]*0.5)) / total_activation; //Weight left side sensors
  float w_R = (IR_output[3] + (IR_output[2]*0.5))/total_activation; //Weight right side sensors
  float e_l = w_R - w_L;
  return e_l;
}

void lineFollowingBehaviour(float gain, float IR_sensitivity) {
  float x = eline();
  float turn_pwm = x*gain;
  
  if (x > IR_sensitivity) {
    motors.setMotorPower((0 + turn_pwm), 0);
  }
  else if (x < -IR_sensitivity) {
    motors.setMotorPower(0, (0 - turn_pwm*8));      
  }
  else {
    motors.setMotorPower(pwm_norm, pwm_norm);
  }
}
