/*Robotic Systems Lab 5 - Finite State Machine
 * The following code uses the existing Polo libraries to integrate an FSM to
 * search for a line and stop when one is found.
 * 
 * This also integrates several buzzers so that the operator can use complete the actions
 * without input from Serial.
 * 
 * 1. Turn on
 * 2. Calibrate - leave on floor surface until first beep
 *              - move front three sensors over line surface and leave until second beep
 *              - move to start position and wait for three happy beeps
 * 3. Find the line - robot will drive fwds until line is found
 * 4. Line found routine - three happy beeps, stop and spin
 * 
 * There is also a timeout feature which will reset to step 2 if no line is found.
 * 
 * Fixes needed:
 * - Currently the buzzer is initiated from within the IRSense library as I can't
 * get it to iniate both there and within the main script
 * - The IRSense could do with a tidy up in the IRread area as there are multiple options there
 * at the moment with ints/floats all over the place! * 
 */

#include "IRSense.h"
#include "motors_v2.h"
#include "buzz.h" //THIS ISN'T WORKING BECAUSE IT IS DEFINED IN IRSENSE - WHY?

//Setup IR Sensors
byte sensor_select = 3;
byte EMIT_STATE = 1;
int time_out = 5000;
float IR_output[5];
float calibrated_IR_output[3];
bool line_stat = false;
IRSense linesensor;

//Setup motors
int MIN_SPEED = 0;
int MAX_SPEED = 100;
int pwm_norm = 25;
int Lpwm = pwm_norm;
int Rpwm = pwm_norm;
Motors_c motors;

//Control Parameters
unsigned long LS_MAX_TIME = 5;              
unsigned long ls_ts;

//Setup FSM states
#define STATE_INITIAL 0
#define STATE_FIND_LINE 1
#define STATE_FOUND_LINE 2
int state;
unsigned long start_time = millis();
unsigned long end_time;
int state_time_out = 30000;

void setup() { 
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("ready");
         
  linesensor.initiate(sensor_select, EMIT_STATE, time_out);
  motors.initiate(MIN_SPEED, MAX_SPEED);

  state = STATE_INITIAL;
}

void loop() {
  if (state == STATE_INITIAL) {
    linesensor.calibrateIR();
    Serial.println("Move to start position");
    delay(1000);
    happy_beep();
    state = 1;
  }
  else if (state == STATE_FIND_LINE) {
    motors.setMotorPower(Lpwm, Rpwm);
  }
  else if (state == STATE_FOUND_LINE) {
    motors.setMotorPower(Lpwm, -Rpwm); 
    Serial.println("FOUND IT");
  }
  else {
    Serial.println("Error: state not found");
    sad_beep();
  }

  updateState();
  Serial.println(state);
}

void updateState() {
  linesensor.IRread(IR_output);
  
  if (state == 2 && millis() - start_time < state_time_out) {
   //Do nothing
  }
  else if (IR_output[1] > 0.5 || IR_output[2] > 0.5 || IR_output[3] > 0.5) {
    motors.stopMotors();    
    happy_beep();
    state = 2;
  }
  else if (millis() - start_time > state_time_out) {  
    sad_beep();
    state = 1;
  }
  
}

void happy_beep() {
  for (int i=0; i<=2; i++) {
    linesensor.beep_once(500, 200);
    delay(200);    
  }
}

void sad_beep() {
  linesensor.beep_once(1000,500);
}
