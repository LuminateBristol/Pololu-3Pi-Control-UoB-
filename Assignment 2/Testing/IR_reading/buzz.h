/*Class file for Pololu IR Sensor Control
Author: Henry Hickson
Date: 14/10/21
References: Robotic Systems Lab Sheet 1
*/

#ifndef _buzz_h
#define _buzz_h

class buzz {
   public:
      #define BUZZER_PIN 6
      
      buzz() {
        pinMode(BUZZER_PIN, OUTPUT);
      }
      
      void beep(int toggle_duration) {
  	    digitalWrite(BUZZER_PIN, HIGH);
  	    delayMicroseconds(toggle_duration);
  	    digitalWrite(BUZZER_PIN, LOW);
  	    delayMicroseconds(toggle_duration);
      }
  	
  	void beep_once(int pitch, int beep_length) {
  	  for (int i = 0; i < beep_length; i++) {
  	     beep(pitch);
  	  }
  	}
 };

 #endif
