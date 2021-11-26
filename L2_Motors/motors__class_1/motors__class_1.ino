#include "motors_v2.h"

#define BUZZER_PIN 6
#define MIN_SPEED 20
#define MAX_SPEED 100

char R = 'R';
char L = 'L';

Motors_c motors;

void setup() {
  Serial.begin(9600);
  motors.init(MIN_SPEED, MAX_SPEED);
  beep_once(1000);
  delay(500);
}

void loop() {
  motors.setMotorPower(30,30);
  delay(2000);
  motors.setMotorPower(30,-30);
  delay(1000);
  motors.stopMotors();
  
  while(1) {
    Serial.println("Program Halted");
    delay(500);
  }
}

void beep_once(int pitch) {
  for (int x = 0; x <= 100; x++) {
    beep(pitch);
  }
}

void beep(int toggle_duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delayMicroseconds(toggle_duration);
  digitalWrite(BUZZER_PIN, LOW);
  delayMicroseconds(toggle_duration);
}
