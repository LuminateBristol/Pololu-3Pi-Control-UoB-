#define BUZZER_PIN 6
int pitch = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
 while (pitch>0) {
  beep(pitch);
  pitch=pitch-10;
  Serial.println(pitch);
 }
 while (pitch<1000) {
  beep(pitch);
  pitch=pitch+10;
 }
 delay(200);
 for (int x = 0; x <= 500; x++) {
  beep(pitch);
 }
 delay(200);
}

void beep(int toggle_duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delayMicroseconds(toggle_duration);
  digitalWrite(BUZZER_PIN, LOW);
  delayMicroseconds(toggle_duration);
}
