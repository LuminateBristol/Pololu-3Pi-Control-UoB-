#define L_PWM_PIN 16
#define L_DIR_PIN 5
#define R_PWM_PIN 15
#define R_DIR_PIN 9

void setup() {
  beep_once();
}

void loop() {
  analogWrite)L_PWM_PIN, 20);
  analogWrite(R_PWM_PIN, 20);
  delay(5);
}


void beep_once() {
  for (int x = 0; x <= 500; x++) {
    beep(pitch);
  }
  delay(500);
}

void beep(int toggle_duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delayMicroseconds(toggle_duration);
  digitalWrite(BUZZER_PIN, LOW);
  delayMicroseconds(toggle_duration);
}
