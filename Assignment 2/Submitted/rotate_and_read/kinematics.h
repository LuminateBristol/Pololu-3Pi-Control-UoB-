// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:
  
	float x = 0;
	float y = 0;
	float theta = 0;
	float wheel_r = 16*1.08;
	float wheel_l = 42.5*1.02;
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    // Use this function to update
    // your kinematics
    void update(long delta_e_right, long delta_e_left) {
	
		float delta_left = delta_e_left * wheel_r * 3.14159 / 180;
		float delta_right = delta_e_right * wheel_r * 3.14159 / 180;

		float delta_dist = (delta_left + delta_right) / 2;
		float delta_theta = (delta_right - delta_left) / (2*wheel_l);
		
		x += delta_dist * cos(theta);
		y += delta_dist * sin(theta);
		theta += delta_theta;

    }

};



#endif
