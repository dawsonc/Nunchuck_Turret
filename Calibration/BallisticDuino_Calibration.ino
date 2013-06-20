//-----------------------------------------------
//******* BallisticDuino Calibration*************
//-----------------------------------------------
//
// By Charles Dawson
// CC-BY-SA
//-----------------------------------------------

#include <Servo.h>

//Servos
Servo trigger;
Servo aimer;

//Pins
int alertPin = 13;
int aimerPin = 9;
int triggerPin = 10;

//Parameters
int triggerClosedAngle = 152;
int triggerOpenAngle = 90;
int aimOffset = 0;

void setup(){
  //Attach the servos
  trigger.attach(triggerPin);
  aimer.attach(aimerPin);
  
  //Setup the pins
  pinMode(alertPin, OUTPUT);
}//End setup

void loop(){
  aimer.write(aimOffset);
  trigger.write(triggerClosedAngle);
}//End loop

