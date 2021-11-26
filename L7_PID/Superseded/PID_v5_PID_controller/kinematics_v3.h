/*Class for Kinemati;cs updating of the Romi Robot
 * 
 *Date: 21/10/21
 *Henry Hickson
 *
 *Kinematics are used to locate the robot in the global coordinate system
 *Location include x, y and rotation angle
 *
 *Update position function
 *- Each time it is ran, the function first checks the rotation angle of each wheel since the last update
 *- It then calculates translation in x and rotation about itself
 *- It then calculates the global movement and adds this to the previous update to give a total global
 *  position
 */


#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:
    float theta_global = 0;
    float x_global = 0;
    float y_global = 0;
    float x_local;
    float y_local;
    float arc_radius;
    float theta_local;
    float travel_r;             //mm moved of left wheel per timestep
    float travel_l;             //mm moved of right wheel per timestep
    float phir;                //radians right wheel turned per timestep
    float phil;                //radians left wheel turned per timestep
    long oldcounte0 = 0;
    long oldcounte1 = 0;
    int wheel_r = 32/2;        //mm wheel radius - needs measuring to confirm
    int l = 43;                //mm distance between wheel centrelines - needs measuring to confirm
    float cpr = 358.3;          //counts per rev from pololu documentation
    float radpc = (2*PI)/cpr;  //radians per count 

    Kinematics_c() {
    } 

    void updateposition() {      
      phir = (count_e0-oldcounte0)*radpc;                      //note that the encoders.h must be included for counts
      phil = (count_e1-oldcounte1)*radpc;
      oldcounte0 = count_e0;
      oldcounte1 = count_e1; 
         
      travel_r = phir*wheel_r;
      travel_l = phil*wheel_r;

      if (travel_r == 0 || travel_l == 0) {
        arc_radius = 2*l;
        theta_local = -travel_r/(arc_radius) + travel_l/(arc_radius);
        x_local = (arc_radius)*sin(abs(theta_local));
        Serial.println("one wheel");
                
      } else if (abs(travel_r) > abs(travel_l)) {
        arc_radius = (2*l*travel_l) / abs(travel_r-travel_l);
        theta_local = -travel_r / (arc_radius+2*l);
        x_local = (arc_radius+l)*tan(abs(theta_local)); 
        Serial.println("r>l");
        
      } else if (abs(travel_r) < abs(travel_l)) {
        arc_radius = (2*l*travel_r) / abs(travel_l-travel_r);
        theta_local = travel_l / (arc_radius+2*l); 
        x_local = (arc_radius+l)*tan(abs(theta_local)); 
        Serial.println("l>r");
               
      } else {
        theta_local = 0;
        x_local = (travel_r+travel_l)/2;    
        Serial.println("equal");    
      }
      
      theta_global = theta_global+theta_local;
      x_global = x_global + x_local*cos(theta_global);
      y_global = y_global + x_local*sin(theta_global);      
    }

};



#endif
