#include "encoders.h"
#include "motors.h"
#include "pid.h"
#include "kinematics.h"
#include "irsensor.h"
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define PID_UPDATE 5

Kinematics_c robot_kinematics;
irSensors_c ir_sensors;
Motor_c motors;
PID_c left_controller;
PID_c right_controller;

long last_e_left;
long last_e_right;
float last_v_left = 0;
float last_v_right = 0;

float left_drive = 0;
float right_drive = 0;

unsigned long kinematics_ts;
unsigned long pid_ts;
unsigned long elapsed_t;

float heading_demand = 2*3.1415926;
float drive_demand = 20;

unsigned long intensity_readings[180];
unsigned long elapsed_times[2];


void setup() {
  left_controller.initialise(0.02, 0.8, 0.05);
  right_controller.initialise(0.02, 0.8, 0.05);
  motors.initialise();
  ir_sensors.initialise();
  
  setupEncoderRight();
  setupEncoderLeft();
  last_e_right = count_e_right;
  last_e_left = count_e_left;
  Serial.begin(9600);
  delay(5000);
  if( SERIAL_ACTIVE )Serial.println("***RESET***");

  pid_ts = millis();
  
}

void loop() {

  unsigned long current_ts;
  current_ts = millis();

  long pid_check = current_ts - pid_ts;

  update_kinematics();

  float current_theta = robot_kinematics.theta;

  if(abs(heading_demand - current_theta) > 0.025) {

    if(pid_check > PID_UPDATE) {
  
      left_drive = left_controller.update(-1*drive_demand, last_v_left, pid_check);
      right_drive = right_controller.update(drive_demand, last_v_right, pid_check);
    
      last_v_left = left_drive;
      last_v_right = right_drive;
      
      motors.setMotorPower("left", left_drive);
      motors.setMotorPower("right", right_drive);
    
      pid_ts = millis();

      // Converts theta in radians to measurement index- 5.7926 = 36/2pi
      int current_theta_int = int(current_theta*5.7296*5);
      Serial.println(current_theta_int);
      
      //if(current_theta_int % 10 == 0 || current_theta_int % 10 == 1) {

      if (not intensity_readings[current_theta_int]) {
          
        Serial.println("----");
          
        ir_sensors.readAllSensors(elapsed_times);
        intensity_readings[current_theta_int] = elapsed_times[0] + elapsed_times[1];
      }
      //}
    }
  }

  else {

    delay(25);

    left_drive = 0;
    right_drive = 0;

    motors.setMotorPower("left", left_drive);
    motors.setMotorPower("right", right_drive);
    
    update_kinematics();
    
    while(true) {
      
      reportResultsOverSerial();
      delay(1000);
      
    }
  }

    delay(1);

}

void update_kinematics() {
  
    long instantaneous_e_right = count_e_right;
    long instantaneous_e_left = count_e_left;
    long delta_e_right = instantaneous_e_right - last_e_right;
    long delta_e_left = instantaneous_e_left - last_e_left;
    last_e_right = instantaneous_e_right;
    last_e_left = instantaneous_e_left;
  
    robot_kinematics.update(delta_e_right, delta_e_left);
}

void reportResultsOverSerial() {

  // Print millis for debug so we can 
  // validate this is working in real
  // time, and not glitched somehow
  if( SERIAL_ACTIVE ) Serial.print( "Time(ms): " );
  if( SERIAL_ACTIVE ) Serial.println( millis() );
  delay(1);


  // Loop through array to print all 
  // results collected

  if( SERIAL_ACTIVE ) {
    for(int i = 0; i < 36*5; i++) {
      Serial.print(intensity_readings[i]);
      Serial.print(", ");
    }
    Serial.println("");
  }

  if( SERIAL_ACTIVE ) Serial.println( "---End of Results ---\n\n" ); 

}
