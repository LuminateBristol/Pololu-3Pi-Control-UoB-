/*Robotic Systems Lab 3 - Macbook version - v2 - class test
 * Other laptop is drying out...
 */

#include "IRSense.h"
byte sensor_select = 3;
byte EMIT_STATE = 1;
int time_out = 5000;
int IR_output[5];
unsigned long ls_ts;
unsigned long LS_MAX_TIME = 500;

IRSense linesensor;

void setup() { 
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
}

void loop() {
  unsigned long loop_time = micros();
  
  unsigned long current_ts = millis();
  unsigned long elapsed_time = current_ts - ls_ts;

  if (elapsed_time > LS_MAX_TIME) {
    linesensor.IRread(IR_output);
    Serial.println(IR_output[0]);
    ls_ts = millis();
  }
}
