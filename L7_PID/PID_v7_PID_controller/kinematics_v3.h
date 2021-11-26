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
 *  
 *  Reference:
 *  http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/odometry.html
 */


#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:
    float theta = 0;
    float x_global = 0;
    float y_global = 0;
    float x_local;
    float arc_radius = 1;
    float phi;                  //radians moved per timestep in local coordinate system
    float travel_r;             //mm moved of left wheel per timestep
    float travel_l;             //mm moved of right wheel per timestep
    float phir;                //radians right wheel turned per timestep
    float phil;                //radians left wheel turned per timestep
    long oldcounte0 = 0;
    long oldcounte1 = 0;
    int spin_sensitivity = 1;  //the error bound for spinning on the spot in mm
    int wheel_r = 32/2;        //mm wheel radius - needs measuring to confirm
    int l = 43;                //mm distance between wheel centrelines - needs measuring to confirm
    float cpr = 358.3;         //counts per rev from pololu documentation
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

      if (travel_r > -travel_l - spin_sensitivity || travel_r < -travel_l + spin_sensitivity) {
        //Spinning on the spot (with a sesnitivty of +- spin_sensitivity mm
        //i.e. it is said to be spinning if travel_l is the same as -travel_r +- the spin sensitivity
        //This is to avoid the program assuming it is moving in an arc when it is actually spinning on the spot
        basic_move(); 
        
      } else if (abs(travel_r) > abs(travel_l)) {
        //Turning in an arc to the right
        arc_radius = ((2*l*travel_l) / abs(travel_r-travel_l)) + l;
        phi = (travel_r - travel_l)/(2*l); 
        arc_move();
                
      } else if (abs(travel_r) < abs(travel_l)) {
        //Turning in an arc to the left
        arc_radius = ((2*l*travel_r) / abs(travel_l-travel_r)) + l;
        phi = (travel_l - travel_r)/(2*l); 
        arc_move();
               
      } else if (travel_r == travel_l) {
        //Straight forward or straight back
        basic_move();
      }
    }

    void arc_move() {
      x_global = x_global + arc_radius*(-sin(theta) + sin(phi)*cos(theta) + sin(theta)*cos(phi));
      y_global = y_global + arc_radius*(cos(theta) - cos(phi)*cos(theta) + sin(theta)*sin(phi)); 
      theta = theta + phi;
    }

    void basic_move() {
      x_local = (travel_r + travel_l)/2;                  
      phi = (travel_l - travel_r)/(2*l);        
      x_global = x_global + x_local*cos(theta);
      y_global = y_global + x_local*sin(theta); 
      theta = theta + phi;
    }
    

};



#endif
