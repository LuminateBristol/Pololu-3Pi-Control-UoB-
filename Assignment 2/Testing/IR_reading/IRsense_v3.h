/*Class file for Pololu IR Sensor Control - v3
Author: Henry Hickson
Date: 11/11/21
References: Robotic Systems Lab Sheet 3 / 4  Supplementary Lab Sheet 2
Notes:
- Updated to include the two front IR sensors 11/11/21
- Updated to include calibration of linesensors 14/10/21
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
       
       #define IRD_PIN_1 A11
       #define IRD_PIN_2 A0
       #define IRD_PIN_3 A2
       #define IRD_PIN_4 A3
       #define IRD_PIN_5 A4
       #define IRF_PIN_1 4
       #define IRF_PIN_2 5
       #define EMIT_PIN 11

       //Setup IR reading parameters
       byte _NB_IRD_PINS = 3;
       byte _NB_IRF_PINS = 2;
       int _IRD_output[3];		//Private internal integer array for the three downward sensors
       int _time_out;
       int ls_pin[3]; 			//Array of downward facing line sensor pins
       int fs_pin[2];			//Array of forward facing IR sensor ping
       unsigned long end_time[5];
       byte which;

       //Setup Calibration parameters:
       //Currently calibration only works for the front three downward facing sensors
       float _IRD_calibrated[3];	//Private internal array for the calibrated three downward sensors
       float _si[3]; 			
       float _bi[3];			
       int n = 50; 			//Number of datapoints for calibration


      IRSense() {
      /*Constructor function
      */
      }


      void initiate(byte sensor_select, byte EMIT_STATE, int time_out) {
      /*Function to initiate the IR sensors by setting up pins and turning IR LEDs on if needed
      */
          if(sensor_select == 1) {
             ls_pin[0] = IRD_PIN_2;
             ls_pin[1] = IRD_PIN_3;
             ls_pin[2] = IRD_PIN_4;
             Serial.println("Middle three bottom sensors only ready to go");
          } else if (sensor_select == 2) {
	           fs_pin[0] = IRF_PIN_1;
	           fs_pin[1] = IRF_PIN_2;
             Serial.println("All forward sensors ready to go.");
          } else {
             Serial.println("Error: Sensor select must be 1 (outer bottom sensors only), 2 (forward sensors only)");
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
      

      float* IRFread(float ARRAY[2]) {
      /* A function to read from the forward facing IR sensors.
         This works by setting each IR sensor to HIGH, waiting until it decays to LOW,
         then recording the time taken.
         High IR readings will decay much faster.
         There is no calibration so we use integers.
         The input and output is an array of integers, length 2
      */
           bool done = false;
           unsigned long start_time = micros();
           
           for (which = 0; which < _NB_IRF_PINS; which++) {
            if (fs_pin[which] != 0) {
                pinMode(fs_pin[which], OUTPUT);
                digitalWrite(fs_pin[which], HIGH);
                pinMode(fs_pin[which], INPUT);
                end_time[which] = start_time;
            }
           }
           delayMicroseconds(10);
           
           while (done == false) {
            for (which = 0; which < _NB_IRF_PINS; which++) {
                if (digitalRead(fs_pin[which]) == 0 && end_time[which] == start_time) {
                    end_time[which] = micros();
                }
            }
            if (end_time[0] != start_time && end_time[1] != start_time) {
             done=true;
            }
            if ((micros()-start_time) >= _time_out) {
             Serial.println("ERROR: Timeout....");
             done=true;
            }
           }    
                
           ARRAY[0] = (end_time[0]-start_time);
           ARRAY[1] = (end_time[1]-start_time);
           
           return ARRAY;
      }
      

      void calibrateIRD() {
      /*Function to calibrate the front three IR downward facing IR sensors only.
      
      	This currently relies on a private _IR_read function which updates an array that is accessible 
      	by the entire class. This should be replaced by using the existing IR_read and just inputting
      	that same accessible array.
      */
        int floor_IR_1[n];
        int floor_IR_2[n];
        int floor_IR_3[n];
        for (int i = 0; i < n; i++) {
          _IRDread();
          floor_IR_1[i] = _IRD_output[0];
          floor_IR_2[i] = _IRD_output[1];
          floor_IR_3[i] = _IRD_output[2];
        }
        Serial.println("Floor_done - move to line");
        buzz.beep_once(1000, 100);
        delay(5000);

        //2 Move to line surface (the line) and repeat
        int line_IR_1[n];
        int line_IR_2[n];
        int line_IR_3[n];
        for (int i = 0; i < n; i++) {
          _IRDread();
          line_IR_1[i] = _IRD_output[0];
          line_IR_2[i] = _IRD_output[1];
          line_IR_3[i] = _IRD_output[2];
        }
        Serial.println("Line done");
        buzz.beep_once(500, 100);

        //3 Derive the offset bias from the minimum white light reading
        _bi[0] = minarray(floor_IR_1);
        _bi[1] = minarray(floor_IR_2);
        _bi[2] = minarray(floor_IR_3);

        //4 Derive the scaling factor from the reading range (difference between min and max readings)
        _si[0] = 1.0000/(average(line_IR_1)-average(floor_IR_1));
        _si[1] = 1.0000/(average(line_IR_2)-average(floor_IR_2));
        _si[2] = 1.0000/(average(line_IR_3)-average(floor_IR_3));
      }   
      
      
      float* IRDread(float ARRAY[3]) { 
      /* A public function to read from the middle three downward facing IR sensors.
         This works by setting each sensor to HIGH, then waiting until that high decays to LOW
         When it has decayed to low, the time measurement is taken.
         The time measuement is then calibrated (if IRCalibrate was ran) and returned.
      */
           bool done = false;
           unsigned long start_time = micros();

           for (which = 0; which < _NB_IRD_PINS; which++) {
            if (ls_pin[which] != 0) {
                pinMode(ls_pin[which], OUTPUT);
                digitalWrite(ls_pin[which], HIGH);
                pinMode(ls_pin[which], INPUT);
                end_time[which] = start_time;
            }
           }
           delayMicroseconds(10);

           while (done == false) {
            for (which = 0; which < _NB_IRD_PINS; which++) {
                if (digitalRead(ls_pin[which]) == 0 && end_time[which] == start_time) {
                    end_time[which] = micros();
                }
            }
            if (end_time[0] != start_time && end_time[1] != start_time && end_time[2] != start_time) {
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
            ARRAY[0] = (end_time[0]-start_time);
            ARRAY[1] = (end_time[1]-start_time);
            ARRAY[2] = (end_time[2]-start_time);
           }
           else {
            ARRAY[0] = (end_time[0]-start_time - _bi[0])*_si[0];
            ARRAY[1] = (end_time[1]-start_time - _bi[1])*_si[1];
            ARRAY[2] = (end_time[2]-start_time - _bi[2])*_si[2];
           }           
           return ARRAY;           
        }
 

      void _IRDread() { 
      /*This function is a private version of the function above (technically it sits in public still atm tho).
      It creates class specific readings for the e_line function below.
      
      This should not be needed, should be able to just use the one above....so need to check this and try and update.
      */
           bool done = false;
           unsigned long start_time = micros();

           for (which = 0; which < _NB_IRD_PINS; which++) {
            if (ls_pin[which] != 0) {
                pinMode(ls_pin[which], OUTPUT);
                digitalWrite(ls_pin[which], HIGH);
                pinMode(ls_pin[which], INPUT);
                end_time[which] = start_time;
            }
           }

           delayMicroseconds(10);
           
           while (done == false) {
            for (which = 0; which < _NB_IRD_PINS; which++) {
                if (digitalRead(ls_pin[which]) == 0 && end_time[which] == start_time) {
                    end_time[which] = micros();
                }
            }
           if (end_time[0] != start_time && end_time[1] != start_time && end_time[2] != start_time) {
             done=true;
           }
           if ((micros()-start_time) >= _time_out) {
             Serial.println("ERROR: Timeout....");
             done=true;
           }
           }

           _IRD_output[0] = (end_time[0]-start_time);
           _IRD_output[1] = (end_time[1]-start_time);
           _IRD_output[2] = (end_time[2]-start_time);

           if (_bi[0] == 0) {
            Serial.println("Error - calibration has not been completed. Add calibrationIR() command.");
           }
           else {
            _IRD_calibrated[0] = (_IRD_output[0] - _bi[0])*_si[0];
            _IRD_calibrated[1] = (_IRD_output[1] - _bi[1])*_si[1];
            _IRD_calibrated[2] = (_IRD_output[2] - _bi[2])*_si[2];
           }
        }
        

        float eline() {
        /* This is a function to creaste a weighted line output for the front three IR sensors
           It does this by weighting the IR reading from the front three as a function or the total
           amount of IR reading time. 
           A value between -1 and 1 is returned to show whether the line is to the right or left
           of centre.
           */
          if (_bi[0] == 0) {
            Serial.println("Error - calibration has not been completed. Add calibrationIR() command.");
           }
           else {
            _IRDread();
            float total_activation = _IRD_calibrated[0] + _IRD_calibrated[1] + _IRD_calibrated[2]; 
            float w_L = (_IRD_calibrated[0] + (_IRD_calibrated[1]*0.5)) / total_activation; 
            float w_R = (_IRD_calibrated[2] + (_IRD_calibrated[1]*0.5)) / total_activation; 
            float e_l = w_R - w_L;
            if (total_activation == 0) {
              return 0;
            } else {
              return e_l;
            }
           }
        }
        

        float average(int myarray[50]) {
        /* Function to take the average of an array of length 50
           50 is chosen as this is the number of data points over which we are calibrating
        */
          int total = 0;
            for (int i = 0; i < (sizeof(myarray) / sizeof(myarray[0])); i++) {
              total += myarray[i];
           }
          return total/(sizeof(myarray) / sizeof(myarray[0]));
        }
        

      int minarray(int myarray[50]) {
      /* Function to find the minimum of an array of length 50
         50 is chosen as this is the number of data points over which we are calibrating
      */
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
