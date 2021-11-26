/*PID Controller Class Header File
 * 28.10.21
 * Henry Hickson
 * 
 * 
 */



#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
  float P;
  float I;
  float K_P = 0.3;
  float K_I = 0.001;

  unsigned long oldtime = 0;
  unsigned long newtime;
  unsigned long dt;
  float etdt;

    PID_c() {
    } 

    void initiate() {
      P = 0;      
    }

    float update_p(float demand, float measurement) {
      float error = demand - measurement;
      P = K_P * error;

      newtime = millis();
      dt = newtime - oldtime;
      etdt = etdt + error*dt;
      I = K_I * etdt;
      oldtime = newtime;

      return (P+I);          
    }

   
};



#endif
