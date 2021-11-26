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
  
  float p;

    PID_c() {
    } 

    void initiate() {
      p = 0;      
    }

    float update_p(float demand, float measurement, float K_p) {
      float error = demand - measurement;
      p = error*K_p;
      return p;
    }

   
};



#endif
