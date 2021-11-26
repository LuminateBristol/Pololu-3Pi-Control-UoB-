/*Robotic Systems Lab 3 - Macbook version - v1
 * Other laptop is drying out...
 * So we continue from the last part of exercise 3
 */

#define IR_PIN_1 A11
#define IR_PIN_2 A0
#define IR_PIN_3 A2
#define IR_PIN_4 A3
#define IR_PIN_5 A4
#define EMIT_PIN 11
#define time_out 5000
#define NB_IR_PINS 5

int ls_pin[NB_IR_PINS] = {IR_PIN_1, IR_PIN_2, IR_PIN_3, IR_PIN_4, IR_PIN_5};
unsigned long end_time[NB_IR_PINS];
int which;

void setup() {
  Serial.begin(9600);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
}

void loop() {
  bool done = false;
  unsigned long start_time = micros();
  
  for (which = 0; which < NB_IR_PINS; which++) {
    pinMode(ls_pin[which], OUTPUT);
    digitalWrite(ls_pin[which], HIGH);
    delayMicroseconds(10);
    pinMode(ls_pin[which], INPUT);
    end_time[which] = start_time;
  }

  while(done == false) {
    for (which = 0; which < NB_IR_PINS; which++) {
      if (digitalRead(ls_pin[which]) == 0 && end_time[which] == start_time) {
        end_time[which] = micros();
      }
    }

    if (end_time[0] != start_time && end_time[1] != start_time && end_time[2] != start_time && end_time[3] != start_time && end_time[4] != start_time) {
      done=true;
    }

    if (micros()-start_time >= time_out) {
      Serial.println("ERROR: Timeout");
      done=true;
    }
  }

  Serial.print("IR 1...");
  Serial.println(end_time[0]-start_time);
  Serial.print("IR 2...");
  Serial.println(end_time[1]-start_time);
  Serial.print("IR 3...");
  Serial.println(end_time[2]-start_time);
  Serial.print("IR 4...");
  Serial.println(end_time[3]-start_time);
  Serial.print("IR 5...");
  Serial.println(end_time[4]-start_time);
  Serial.println(" ");

  delay(100);
}
