/*Pololu sumo programme - HH - v1
 *Moves until either bump sensor or line sensor is activated 
 *When activated, turn the other direction and continue
 *IMPROVEMENTS NEEDED
 *1. Turning by angle rather than randomly using delay functions
 *2. Replacing any delay functions with millis and possibly adding this to forloop
 *3. Additional battle modes and switches between those modes
 */ 
 
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>
#include <Pololu3piPlus32U4LCD.h>
#include <Pololu3piPlus32U4Encoders.h>
using namespace Pololu3piPlus32U4;

LCD display; 
Buzzer buzzer;
LineSensors lineSensors;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
Encoders encoders;
BumpSensors bumpSensors;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
unsigned long timeNow = 0;
uint16_t maxSpeedL;
uint16_t maxSpeedR;
uint16_t calibrationSpeed;

int IRmin = 1500;

byte d = 32; //mm
byte l = 96; //mm
float cpr = 358.3; //counts per revolution on the encoder
float mm = (PI * d)/cpr;

String robostatus;

void setup() {
  Serial.begin(9600);
  bumpSensors.calibrate();
  display.clear();
  display.print("READY!");
  selectTurtle();
  move();
}

void loop() {
  bumpcheck();
}

void move() {
  motors.setSpeeds(maxSpeedL,maxSpeedR);
}

void moveback() {
  motors.setSpeeds(-maxSpeedL,-maxSpeedR);
  delay(250);
}

void IRcheck() {
  lineSensors.read(lineSensorValues);
  if(lineSensorValues[0] >= IRmin || lineSensorValues[2] >= IRmin || lineSensorValues[4] >= IRmin) {
    moveback();
    turn90ish();
  }
}
  
void bumpcheck() {
    bumpSensors.read();
    
    if (bumpSensors.leftChanged()) {
      ledYellow(bumpSensors.leftIsPressed());
      if (bumpSensors.leftIsPressed()) {
        turn90ish();
        display.print('L');
        move();
      }
      else {
        selectTurtle();
        display.clear();
        move();
      }
    }
  
    if (bumpSensors.rightChanged()) {
      ledRed(bumpSensors.rightIsPressed());
      if (bumpSensors.rightIsPressed()) {
        turn90ish();
        display.print('R');
        move();
      }
      else {
        selectTurtle();
        display.clear();
        move();
      }
    }
}

void selectTurtle() {
  maxSpeedL = 150;
  maxSpeedR = 143;
  calibrationSpeed = 50;
}

void spotturn(int newangle) {
  float posl = mm*encoders.getCountsLeft(); //gives the number of clicks the wheel has turned since records began
  float posr = mm*encoders.getCountsRight();
  float oldangle = (posr - posl)/cpr;
  Serial.println("yes");
  Serial.println(oldangle);

  while (newangle != oldangle) {
    if (newangle < oldangle) {
      motors.setSpeeds(maxSpeedL,-maxSpeedR);
    }
    if (newangle > oldangle) {
      motors.setSpeeds(-maxSpeedL,maxSpeedR);
    }
    posl = mm*encoders.getCountsLeft();
    posr = mm*encoders.getCountsRight();
    oldangle = (posr - posl)/cpr;
    Serial.println(oldangle);
  }
}

void turn180ish() {
  motors.setSpeeds(maxSpeedL,-maxSpeedR);
  delay(400);
  move();
}

void turn90ish() {
  motors.setSpeeds(-maxSpeedL,maxSpeedR);
  delay(200);
  move();
}
