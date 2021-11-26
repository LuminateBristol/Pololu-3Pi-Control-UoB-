/*Class file for Pololu IR Sensor Control - v2
Author: Henry Hickson
Date: 14/10/21
References: Robotic Systems Lab Sheet 3 / 4
Notes:
- Updated to include calibration 14/10/21
- IR sensors work through a capacitor discharge method
- The capacitor is first loaded with charge (digitalWrite HIGH)
- It then discharges at a rate determined by the IR sensor
- More IR light, faster discharge
- There is a discharge threshold of 1.5v, at which point the digital switches to LOW
- This is analogue to digital conversion
- By measuring time taken to switch, we have a measure of IR reflected

Calibration:
- This works through the use of the IR read:
- The robot reads 50 values on the floor, then beeps
- On the first beep, the user should move the three sensors to over the line
- It will read 50 values on the line then beep again
- After the second beep it does the maths to produce si and bi
- the readings are then calibrated by minusing the bi (the baseline) and then multiplying by
  si

To initiate the user must first specify the following:
- Sensor_select - choose option 1, 2 or 3 for amount of sensors to use
- EMIT_STATE - turn IR LEDs on or off
- time_out - time out time on readings

Outputting e_l:
- e_l is a weighting from -1 to 1 which is the weighting of IR left to right of the front three IRs

Outputting direct readings:
- it is also possible to output direct readings for each sensor
- An input must be provided for this! An array of size 5 :)
*/

#ifndef _IRSense_h
#define _IRSense_h

class IRSense {
   public:
       #include "buzz.h"
       buzz buzz;
       
       #define IR_PIN_1 A11
       #define IR_PIN_2 A0
       #define IR_PIN_3 A2
       #define IR_PIN_4 A3
       #define IR_PIN_5 A4
       #define EMIT_PIN 11

       //Setup IR reading parameters
       byte _NB_IR_PINS = 5;
       int _IR_output[5];
       int _time_out;
       int ls_pin[5]; 
       unsigned long end_time[5];
       byte which;

       //Setup Calibration parameters:
       float _IR_calibrated[5];
       float _si[3]; //Currently calibration only works for the front three
       float _bi[3];
       int n = 50; //Number of datapoints for calibration

      IRSense() {
      }

      void initiate(byte sensor_select, byte EMIT_STATE, int time_out) {

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
      

      void calibrateIR() {
        int floor_IR_1[n];
        int floor_IR_2[n];
        int floor_IR_3[n];
        for (int i = 0; i < n; i++) {
          _IRread();
          floor_IR_1[i] = _IR_output[1];
          floor_IR_2[i] = _IR_output[2];
          floor_IR_3[i] = _IR_output[3];
        }
        Serial.println("Floor_done - move to line");
        buzz.beep_once(1000);
        delay(5000);

        //2 Move to line surface (the line) and repeat
        int line_IR_1[n];
        int line_IR_2[n];
        int line_IR_3[n];
        for (int i = 0; i < n; i++) {
          _IRread();
          line_IR_1[i] = _IR_output[1];
          line_IR_2[i] = _IR_output[2];
          line_IR_3[i] = _IR_output[3];
        }
        Serial.println("Line done");
        buzz.beep_once(500);

        //3 Derive the offset bias from the minimum white light reading
        _bi[0] = minarray(floor_IR_1);
        _bi[1] = minarray(floor_IR_2);
        _bi[2] = minarray(floor_IR_3);

        //4 Derive the scaling factor from the reading range (difference between min and max readings)
        _si[0] = 1.0000/(average(line_IR_1)-average(floor_IR_1));
        _si[1] = 1.0000/(average(line_IR_2)-average(floor_IR_2));
        _si[2] = 1.0000/(average(line_IR_3)-average(floor_IR_3));
      }


      //This function needs an input - a globally defined array of length 5. This will then be populated and
      //passed back.
      int* IRread(int ARRAY[5]) { 
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

           //Check for calibration of forward sensors
           if (_bi[0] == 0) {
            Serial.println("Warning - calibration has not been completed. Add calibrationIR() command.");
            ARRAY[1] = (end_time[1]-start_time);
            ARRAY[2] = (end_time[2]-start_time);
            ARRAY[3] = (end_time[3]-start_time);
           }
           else {
            ARRAY[1] = (end_time[1]-start_time - _bi[0])*_si[0];
            ARRAY[2] = (end_time[2]-start_time - _bi[1])*_si[1];
            ARRAY[3] = (end_time[3]-start_time - _bi[2])*_si[2];
           }

           //Populate other IR sensor values
           ARRAY[0] = (end_time[0]-start_time);
           ARRAY[4] = (end_time[4]-start_time);
           
           return ARRAY;           
        }
 

      //This is an internal IRread function, it is used within this class only, hence the underscore
      //The function is the same as the one above apart from it populates a class specific array and not a global one
      //This is then calibrated as per the calibration sequence
      int _IRread() { 
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

           _IR_output[0] = (end_time[0]-start_time);
           _IR_output[1] = (end_time[1]-start_time);
           _IR_output[2] = (end_time[2]-start_time);
           _IR_output[3] = (end_time[3]-start_time);
           _IR_output[4] = (end_time[4]-start_time);

           if (_bi[0] == 0) {
            Serial.println("Error - calibration has not been completed. Add calibrationIR() command.");
           }
           else {
            _IR_calibrated[1] = (_IR_output[1] - _bi[0])*_si[0];
            _IR_calibrated[2] = (_IR_output[2] - _bi[1])*_si[1];
            _IR_calibrated[3] = (_IR_output[3] - _bi[2])*_si[2];
           }
        }
        

        //eline() returns a weighted line value for the front three sensors, this is useful for line following
        float eline() {
          if (_bi[0] == 0) {
            Serial.println("Error - calibration has not been completed. Add calibrationIR() command.");
           }
           else {
            _IRread();
            int total_activation = _IR_calibrated[1] + _IR_calibrated[2] + _IR_calibrated[3]; //Total of three forward sensor readings
            float w_L = (_IR_calibrated[1] + (_IR_calibrated[2]*0.5)) / total_activation; //Weight left side sensors
            float w_R = (_IR_calibrated[3] + (_IR_calibrated[2]*0.5))/total_activation; //Weight right side sensors
            float e_l = w_R - w_L;
            return e_l;
           }
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
 };

 #endif
