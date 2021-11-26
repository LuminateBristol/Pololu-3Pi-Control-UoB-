/*Class file for Pololu IR Sensor Control
Author: Henry Hickson
Date: 07/10/21
References: Robotic Systems Lab Sheet 3
Notes:
- IR sensors work through a capacitor discharge method
- The capacitor is first loaded with charge (digitalWrite HIGH)
- It then discharges at a rate determined by the IR sensor
- More IR light, faster discharge
- There is a discharge threshold of 1.5v, at which point the digital switches to LOW
- This is analogue to digital conversion
- By measuring time taken to switch, we have a measure of IR reflected

To initiate the user must first specify the following:
- Sensor_select - choose option 1, 2 or 3 for amount of sensors to use
- EMIT_STATE - turn IR LEDs on or off
- time_out - time out time on readings

To read the user must provide a array[5] to be filled with the outputted data
- IRread(array[5])
*/

#ifndef _IRSense_h
#define _IRSense_h

class IRSense {
   public:
       #define IR_PIN_1 A11
       #define IR_PIN_2 A0
       #define IR_PIN_3 A2
       #define IR_PIN_4 A3
       #define IR_PIN_5 A4
       #define EMIT_PIN 11

       byte _NB_IR_PINS = 5;
       int _time_out;
       int ls_pin[5]; //Note that we could use a pointer and the new command for dynamic memory instead
                      //here but for the sake of two integers it's not really needed so instead use max
       unsigned long end_time[5];
       byte which;

      IRSense() {
      }

      void initiate(byte sensor_select, byte EMIT_STATE, int time_out) {
          Serial.begin(9600);
          while(!Serial);
          //Wait for Serial to come online

          if(sensor_select == 1) {
            ls_pin[0] = IR_PIN_1;
            ls_pin[4] = IR_PIN_5;
            Serial.println("Outer sensors only ready to go");
          } else if (sensor_select == 2) {
            ls_pin[1] = IR_PIN_2;
            ls_pin[2] = IR_PIN_3;
            ls_pin[3] = IR_PIN_4;
            Serial.println("Inner sensors only ready to go");
          } else if (sensor_select == 3) {
            ls_pin[0] = IR_PIN_1;
            ls_pin[1] = IR_PIN_2;
            ls_pin[2] = IR_PIN_3;
            ls_pin[3] = IR_PIN_4;
            ls_pin[4] = IR_PIN_5;
            Serial.println("All sensors ready to go.");
          } else {
            Serial.println("Error: Sensor select must be 1 (outer sensors only), 2 (middle sensors only) or 3 (all sensors)");
            return;
          }

          _time_out = time_out;

          pinMode(11,OUTPUT);
          if (EMIT_STATE == 0 || EMIT_STATE == 1) {
            digitalWrite(11,EMIT_STATE);
          } else {
            Serial.println("Error: EMIT_STATE must be 0 (IR LED switched off) or 1 (IR LED switched on)");
          }
      }

      int* IRread(int ARRAY[5]) { //By using a pointer function we can update the existing array IR_output
           bool done = false;
           unsigned long start_time = micros();

           for (which = 0; which < _NB_IR_PINS; which++) {
            if (ls_pin[which] != 0) {
                pinMode(ls_pin[which], OUTPUT);
                digitalWrite(ls_pin[which], HIGH);
                pinMode(ls_pin[which], INPUT);
                end_time[which] = start_time;
            }
           }

           delayMicroseconds(10);

           while (done == false) {
            for (which = 0; which < _NB_IR_PINS; which++) {
                if (digitalRead(ls_pin[which]) == 0 && end_time[which] == start_time) {
                    end_time[which] = micros();
                }
            }

           if (end_time[0] != start_time && end_time[1] != start_time && end_time[2] != start_time && end_time[3] != start_time && end_time[4] != start_time) {
             done=true;
           }

           if ((micros()-start_time) >= _time_out) {
             Serial.println("ERROR: Timeout....");
             done=true;
           }
           }

           ARRAY[0] = (end_time[0]-start_time);
           ARRAY[1] = (end_time[1]-start_time);
           ARRAY[2] = (end_time[2]-start_time);
           ARRAY[3] = (end_time[3]-start_time);
           ARRAY[4] = (end_time[4]-start_time);
           return ARRAY;
        }
 };

 #endif
