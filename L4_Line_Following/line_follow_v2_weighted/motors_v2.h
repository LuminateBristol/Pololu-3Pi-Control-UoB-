/*Class file for Pololu Motor Control
Author: Henry Hickson
Date: 04/10/21
References: Robotic Systems Lab Sheet 2
*/

#ifndef _MOTORS_H
#define _MOTORS_H

class Motors_c {
   public:
   
   #define L_PWM_PIN 10
   #define L_DIR_PIN 16
   #define R_PWM_PIN 9
   #define R_DIR_PIN 15
   byte LDIR = 0;
   byte RDIR = 0;
   int _MIN_SPEED;
   int _MAX_SPEED;
      
      Motors_c() {
      }
      
      void initiate(int MIN_SPEED, int MAX_SPEED) {
      	pinMode(L_PWM_PIN, OUTPUT);
      	pinMode(L_DIR_PIN, OUTPUT);
      	pinMode(R_PWM_PIN, OUTPUT);
      	pinMode(R_DIR_PIN, OUTPUT);  
      	_MIN_SPEED = MIN_SPEED;
      	_MAX_SPEED = MAX_SPEED;
      }
      
      void setMotorPower(float Lpwm, float Rpwm) { //This function turns the motors, L and R
  	if (abs(Lpwm) < _MIN_SPEED || abs(Lpwm) > _MAX_SPEED || abs(Rpwm) < _MIN_SPEED || abs(Rpwm) > _MAX_SPEED) {
    	   Serial.println ("Error! Outside of acceptable motor range!");
    	   return;
    	}  
  	if (abs(Lpwm) != Lpwm) {
   	  LDIR = 255;
  	}
  	else if (abs(Lpwm) == Lpwm) {
  	  LDIR = 0;
  	} 	
  	if (abs(Rpwm) != Rpwm) {
   	  RDIR = 255;
  	}
  	else if (abs(Rpwm) == Rpwm) {
  	  RDIR = 0;
  	}
  	analogWrite(L_DIR_PIN, LDIR);
  	analogWrite(R_DIR_PIN, RDIR);
   	analogWrite(L_PWM_PIN, abs(Lpwm));
  	analogWrite(R_PWM_PIN, abs(Rpwm));
      }
      
      void stopMotors() {
  	analogWrite(L_PWM_PIN, 0);
  	analogWrite(R_PWM_PIN, 0);
      }
 };
 
 #endif
