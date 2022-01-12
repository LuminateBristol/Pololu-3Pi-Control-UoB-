#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

class Motor_c {
  public:

  void initialise() {
    digitalWrite(L_DIR_PIN, FWD);
    digitalWrite(R_DIR_PIN, FWD);
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
  }

  int setMotorPower(String motor, float pwm) {

    int target_dir_pin;
    int target_pwm_pin;
  
    if (motor == "left") {
      target_pwm_pin = L_PWM_PIN;
      target_dir_pin = L_DIR_PIN;
    }
    else if (motor == "right") {
      target_pwm_pin = R_PWM_PIN;
      target_dir_pin = R_DIR_PIN;
    }
    else {
      Serial.println("Error: Invalid motor selection. Please set first argument to \"left\" or \"right\".");
      
      return 1;
    }
    
    if (pwm >= 0) {
       digitalWrite(target_dir_pin, FWD);
    }
    else {
       digitalWrite(target_dir_pin, REV);
    }
  
    if (abs(pwm) > 200) {
      Serial.print("Warning: Illegal motor speed (");
      Serial.print(pwm);
      Serial.println(").");
      Serial.println("Motor speed setting exceeded and will be capped at 200.");
      Serial.println("Please use motor speeds in range -200 < x < 200.");
    }
    
    analogWrite(target_pwm_pin, min(abs(pwm), 200));
  
    return 0;
  }
};
