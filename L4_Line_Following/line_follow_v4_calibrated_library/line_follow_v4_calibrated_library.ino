/*Robotic Systems Lab 4 - Line Following
 * 
 * This version uses the updated IRSense.h class which includes:
 * - Weighted front sensors
 * - Calibration of front sensors
 * - Internal IRReading
 * 
 * Some tips for line following:
 * - Gennerally a slow speed works better as the calculation speed cannot keep up otherwise
 * - If the robot is struggling with tight corners, increase gain
 * - Increasing gain will decrease the 'inside' wheel in a turn
 * - Decreasing the inside wheel, even to negative, will improve tight turning performance
 * - In daylight with a black line on white background, speed = 25-30, gain = 60-80 seems to work well
 * - The difference between the speed and the gain is determines how quickly the robot will turn
 */

#include "IRSense.h"
#include "motors_v2.h"


//Setup IR Sensors
byte sensor_select = 3;
byte EMIT_STATE = 1;
int time_out = 5000;
int IR_output[5];
bool line_stat = false;
IRSense linesensor;

//Setup motors
int MIN_SPEED = 0;
int MAX_SPEED = 100;
int Lpwm;
int Rpwm;
Motors_c motors;

//Control Parameters
unsigned long LS_MAX_TIME = 5; //time of the loop - lower means quicker reactions
float IR_sensitivity = 0.05;    //sensitivity of the IR sensors to changes in IR, lower is more sensitive
float pwm_norm = 30;             //standard power to motors
float gain = 80;                //gain for the reduction in power when turning due to weighted IR
unsigned long ls_ts;

void setup() { 
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("ready");
         
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
  motors.initiate(MIN_SPEED, MAX_SPEED);
  delay(500);
  linesensor.calibrateIR();
  delay(500);
}

void loop() {
  unsigned long loop_time = micros();  
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - ls_ts;


  if (elapsed_time > LS_MAX_TIME) {
    lineFollowingBehaviour(gain, IR_sensitivity); 
    ls_ts = millis();
  }   
}

void lineFollowingBehaviour(float gain, float IR_sensitivity) {
  float e_l = linesensor.eline(); //weighted error from left to right side
  float turn_pwm = e_l*gain;
  
  if (e_l > IR_sensitivity) {
    Lpwm = pwm_norm;
    Rpwm = pwm_norm-turn_pwm;
  }
  else if (e_l < -IR_sensitivity) {
    Lpwm = pwm_norm+turn_pwm;
    Rpwm = pwm_norm;     
  }
  else {
    Lpwm = pwm_norm;
    Rpwm = pwm_norm;
  }
  Serial.println(e_l);
  Serial.println(Lpwm);
  Serial.println(Rpwm);
  Serial.println(" ");
  motors.setMotorPower(Lpwm,Rpwm);
}
