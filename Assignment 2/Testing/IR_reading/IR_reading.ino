#include "IRsense_v3.h"
#include "buzz.h"
IRSense frontsensor;

//Time parameters
unsigned long current_time = millis();
unsigned long timestep = 100;
unsigned long oldtime = 0;

//Linesensor Parameters
byte sensor_select = 2;
byte EMIT_STATE = 0;
int IR_time_out = 5000;
float IRF_output[2];

void setup() {
  Serial.begin(9600);
  frontsensor.initiate(sensor_select, EMIT_STATE, IR_time_out);
}

void loop() {
  if (millis() - current_time > timestep) {
    frontsensor.IRFread(IRF_output);
    printoutput();
    current_time = millis();
  }
}

void printoutput() {
  Serial.print(IRF_output[0]);
  Serial.print(",");
  Serial.print(IRF_output[1]);
  Serial.println(""); 
}
