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
   byte DIR = 0;
   int _MIN_SPEED;
   int _MAX_SPEED;
      
      Motors_c() {
      }
      
      void init(int MIN_SPEED, int MAX_SPEED) {
      	pinMode(L_PWM_PIN, OUTPUT);
      	pinMode(L_DIR_PIN, OUTPUT);
      	pinMode(R_PWM_PIN, OUTPUT);
      	pinMode(R_DIR_PIN, OUTPUT);  
      	_MIN_SPEED = MIN_SPEED;
      	_MAX_SPEED = MAX_SPEED;
      }
      
      void setMotorPower(float pwm, char side) { //This function turns one of the motors, L or R
  	if (abs(pwm) <= _MIN_SPEED || abs(pwm) >= _MAX_SPEED) {
    	   Serial.println ("Error! Outside of acceptable motor range!");
    	   return;
    	}
  
  	if (abs(pwm) != pwm) {
   	  DIR = 255;
  	}
  	else if (abs(pwm) == pwm) {
  	  DIR = 0;
  	}

  	if (side == 'R') {
  	  analogWrite(R_DIR_PIN, DIR);
  	  analogWrite(R_PWM_PIN, abs(pwm));
  	}
  	else if (side == 'L') {
  	  analogWrite(L_DIR_PIN, DIR);
	  analogWrite(L_PWM_PIN, abs(pwm));
  	}
  	else if (side == 0) {
  	  analogWrite(L_DIR_PIN, DIR);
  	  analogWrite(R_DIR_PIN, DIR);
   	  analogWrite(L_PWM_PIN, abs(pwm));
  	  analogWrite(R_PWM_PIN, abs(pwm));
  	}
      }
      
      void setDoublePower(float Lpwm, float Rpwm) { //Function to set both motor speeds and dirs 
  	setMotorPower(Lpwm, 'L');
  	setMotorPower(Rpwm, 'R');
      }

      void setSinglePower(float pwm) { //Function to set same speed and dir for both motors
  	setMotorPower(pwm, 0);
      }
      
      void stopMotors() {
  	analogWrite(L_PWM_PIN, 0);
  	analogWrite(R_PWM_PIN, 0);
      }
 };
 
 #endif
