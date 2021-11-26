/*Robotic Systems Lab 4 - Line Following
 * This is a bang-bang controller method where the robot first drives until it finds the 
 * line and then it goes into bang-bang mode. In this mode, the robot uses sensors 1 and 3 
 * and when either of those goes over the threshold, it turns the other way to get back 
 * on the line.
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
unsigned long ls_ts;
unsigned long LS_MAX_TIME = 10;
bool line_stat = false;
IRSense linesensor;

//Setup motors
int MIN_SPEED = 0;
int MAX_SPEED = 100;
float Rpwm_norm = 25;
float Lpwm_norm = 25;
float Rpwm = Rpwm_norm;
float Lpwm = Lpwm_norm;
char dir = 'F';

Motors_c motors;

void setup() { 
  Serial.begin(9600);
         
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
  motors.initiate(MIN_SPEED, MAX_SPEED);
  motors.setMotorPower(50,-50);
  delay(200);
  motors.stopMotors();
  delay(500);
}

void loop() {
  unsigned long loop_time = micros();  
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - ls_ts;

  if (elapsed_time > LS_MAX_TIME) {

    motors.setMotorPower(Lpwm, Rpwm);
    
    while (line_stat == false) { //Wait until we find the line
      linesensor.IRread(IR_output);
      linestatus();
    }
    
    linesensor.IRread(IR_output);

    if (IR_output[1] >= 700) {
      Rpwm = 60;
      Lpwm = 10;
      dir = 'L';      
    }
    if (IR_output[3] >= 700) {
      Rpwm = 10;
      Lpwm = 60;
      dir = 'R';
    }
    if (IR_output[2] >= 700) {
      Rpwm = Rpwm_norm;
      Lpwm = Lpwm_norm;
    }
    
    Serial.println(IR_output[1]);
    Serial.println(IR_output[2]);
    Serial.println(IR_output[3]);
    Serial.println(" ");
    ls_ts = millis();
  }
}

void linestatus() {
  if (IR_output[1] > 700 || IR_output[2] > 700 || IR_output[3] > 700) {
    line_stat = true;
  }
}
