// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
	float feedback_signal;
	float error_signal;
	float integrated_error_signal;
	float error_logs[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float delta_error;
	
	float k_p;
	float k_i;
	float k_d;
	
	float p_term;
	float i_term;
	float d_term;
  
    // Constructor, must exist.
    PID_c() {

    } 
	
	void initialise(float p, float i, float d) {
		k_p = p;
		k_i = i;
		k_d = d;
		
		p_term = 0;
		i_term = 0;
		d_term = 0;
	}
	
	float update(float demand, float measurement, float delta_t) {
		error_signal = demand - measurement;
		
		delta_error = error_signal - integrated_error_signal / 5;

		error_logs[0] = error_logs[1];
		error_logs[1] = error_logs[2];
		error_logs[2] = error_logs[3];
		error_logs[3] = error_logs[4];
		error_logs[4] = error_signal;
		
		integrated_error_signal = error_logs[0] + error_logs[1] + error_logs[2] + error_logs[3] + error_logs[4];
		
		p_term = k_p * error_signal;
		i_term = k_i * integrated_error_signal;
		d_term = k_d * (delta_error / delta_t);
		
		feedback_signal = p_term + i_term + d_term;
		
		return feedback_signal;
	}
	
};



#endif
