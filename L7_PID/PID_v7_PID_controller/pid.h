/*PID Controller Class Header File
 * 28.10.21
 * Henry Hickson
 * 
 * This controller is a velocity sensitve PID controller for the 3PI+
 * Notes:
 * - P: the Proportional part of the controller simply sends an error signal back to
 * the controller. This is multiplied by a gain value and added to the motor power
 * input to increase or decrease power as needed to meet our speed requirements.
 * - I: the Integral part of the contoller integrates the above mentioned error and 
 * adds the result. This helps to remove steady state error - the differnece betwee
 * the desired speed, and the speed set by the P controller.
 * - D: the derivative term is used to remove overshoot errors when switching from
 * one demand to another. For example, a sudden change from fwd motion to bwd motion
 * would normally result in a large error which can cause issues for the PI controller
 * which will try to instantly correct this with a large, usually excessive response.
 * 
 */

#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
  float P;
  float I;
  float D;

  unsigned long oldtime = 0;
  unsigned long newtime;
  unsigned long dt;
  float etdt;
  float error_old = 0;

    PID_c() {
    } 

    void initiate() {
      P = 0;      
    }

    float update_pid(float demand, float measurement, float K_P, float K_I, float K_D) {
      float error = demand - measurement;
      
      newtime = millis();            
      dt = newtime - oldtime;
      oldtime = newtime;
      
      etdt = etdt + error*dt;
      float ediff = error - error_old;
      error_old = error;
      
      P = K_P * error;
      I = K_I * etdt;
      if (dt != 0) {
        D = K_D*(ediff/dt);
      }
      else {
        Serial.println("Error: PID controller dt is zero!");
        D = 0;
      }

      return (P+I+D);          
    }

    void resetvariables() { 
      //This is needed if we have a status change that causes a large dt
      //and or large value of I
      etdt = 0;
      oldtime = millis();
    }      
};



#endif
