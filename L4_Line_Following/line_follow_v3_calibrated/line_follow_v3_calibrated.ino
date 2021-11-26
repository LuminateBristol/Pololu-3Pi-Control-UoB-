/*Robotic Systems Lab 4 - Line Following
 * This is a weoghted model that also features calibration of the sensors to ensure even performance
 * when turning in both directions.
 * 
 * TRY CS 50 MIT COURSE!!!
 */

#include "IRSense.h"
#include "motors_v2.h"
#include "buzz.h"

//Setup buzzer
buzz buzz;

//Setup IR Sensors
byte sensor_select = 3;
byte EMIT_STATE = 1;
int time_out = 5000;
int IR_output[5];
bool line_stat = false;
IRSense linesensor;

//Setup calibration
float IR_calibrated[5];
float si[3];
int di[3];
int n = 50;

//Setup motors
int MIN_SPEED = 0;
int MAX_SPEED = 100;
int Lpwm;
int Rpwm;
Motors_c motors;

//Control Parameters
unsigned long LS_MAX_TIME = 100;
float IR_sensitivity = 0.05;
float pwm_norm = 80;
float gain = 60;
unsigned long ls_ts;

void setup() { 
  Serial.begin(9600);
  while(!Serial);
  Serial.println("ready");
         
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
  motors.initiate(MIN_SPEED, MAX_SPEED);
  delay(500);
  calibrateIR();
}

void loop() {
  unsigned long loop_time = micros();  
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - ls_ts;


  if (elapsed_time > LS_MAX_TIME) {
    
    linesensor.IRread(IR_output);

    IR_calibrated[1] = (IR_output[1] - di[0])*si[0];
    IR_calibrated[2] = (IR_output[2] - di[1])*si[1];
    IR_calibrated[3] = (IR_output[3] - di[2])*si[2];

    /*Serial.print(IR_calibrated[1],3);
    Serial.print(",");
    Serial.print(IR_calibrated[2],3);
    Serial.print(",");
    Serial.print(IR_calibrated[3],3);
    Serial.println(" ");*/

    lineFollowingBehaviour(gain, IR_sensitivity); 

    ls_ts = millis();
  }   
}

void calibrateIR() {
  //Calibration specifically for line reading - includes 'Find the line' protocol
  //1 Start on floor surface and get 100 readings
  int floor_IR_1[n];
  int floor_IR_2[n];
  int floor_IR_3[n];
  for (int i = 0; i < n; i++) {
    linesensor.IRread(IR_output);
    floor_IR_1[i] = IR_output[1];
    floor_IR_2[i] = IR_output[2];
    floor_IR_3[i] = IR_output[3];
  }
  Serial.println("Floor_done - move to line");
  //buzz.beep_once(1000);
  delay(5000);

  //2 Move to line surface (the line) and repeat
  int line_IR_1[n];
  int line_IR_2[n];
  int line_IR_3[n];
  for (int i = 0; i < n; i++) {
    linesensor.IRread(IR_output);
    line_IR_1[i] = IR_output[1];
    line_IR_2[i] = IR_output[2];
    line_IR_3[i] = IR_output[3];
  }
  Serial.println("Line done");
  //buzz.beep_once(500);

  //3 Derive the offset bias from the minimum white light reading
  di[0] = minarray(floor_IR_1);
  di[1] = minarray(floor_IR_2);
  di[2] = minarray(floor_IR_3);

  //4 Derive the scaling factor from the reading range (difference between min and max readings)
  si[0] = 1.0000/(average(line_IR_1)-average(floor_IR_1));
  si[1] = 1.0000/(average(line_IR_2)-average(floor_IR_2));
  si[2] = 1.0000/(average(line_IR_3)-average(floor_IR_3));
}

float average(int myarray[50]) {
  int total = 0;
  for (int i = 0; i < (sizeof(myarray) / sizeof(myarray[0])); i++) {
     total += myarray[i];
  }
  return total/(sizeof(myarray) / sizeof(myarray[0]));
}

int minarray(int myarray[50]) {
  int minVal = myarray[0];
  for (int i = 0; i < (sizeof(myarray) / sizeof(myarray[0])); i++) {
    if (myarray[i] < minVal) {
      minVal = myarray[i];
    }
  }
  return minVal;
}

void findtheline() {
  while (line_stat == false) {
    motors.setMotorPower(pwm_norm, pwm_norm);
    linesensor.IRread(IR_output);
    if (IR_output[1] > 800 && IR_output[2] > 800 && IR_output[3] > 800) {
      line_stat = true;
      motors.setMotorPower(0 , 0);
    }
  }
}

float eline() {
  linesensor.IRread(IR_output);
  int total_activation = IR_calibrated[1] + IR_calibrated[2] + IR_calibrated[3]; //Total of three forward sensor readings
  float w_L = (IR_calibrated[1] + (IR_calibrated[2]*0.5)) / total_activation; //Weight left side sensors
  float w_R = (IR_calibrated[3] + (IR_calibrated[2]*0.5))/total_activation; //Weight right side sensors
  float e_l = w_R - w_L;
  return e_l;
}

void lineFollowingBehaviour(float gain, float IR_sensitivity) {
  float e_l = eline();
  float turn_pwm = e_l*gain;
  //Serial.println(turn_pwm);
  
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
  /*Serial.println(Lpwm);
  Serial.println(Rpwm);
  Serial.println("");*/
  motors.setMotorPower(Lpwm,Rpwm);
}
