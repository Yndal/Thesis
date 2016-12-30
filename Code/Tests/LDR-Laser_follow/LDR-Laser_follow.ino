#include <ZumoMotors.h>

#define LDR_PIN A4
#define LIGHT_THRESHOLD 330
//#define MAX_TIME_NO_LIGHT 5000 //ms
#define LOW_TURN_VALUE 60
#define HIGH_TURN_VALUE 100
#define START_TURN_VALUE 75

ZumoMotors motors;

bool turningLeft = true;
bool isTurning = false;

void setup() {   
  Serial.begin(9600);
  motors.setLeftSpeed(-START_TURN_VALUE);
  motors.setRightSpeed(START_TURN_VALUE);
  while(analogRead(LDR_PIN) < LIGHT_THRESHOLD);
  
}

void loop() {
  int value = analogRead(LDR_PIN);
  //Serial.println(value);
  // put your main code here, to run repeatedly:
  if(LIGHT_THRESHOLD <= value){
    if(!isTurning){
      if(turningLeft)
        turnRight();
      else
        turnLeft();
      isTurning = true;
      turningLeft = !turningLeft;
    }
  } else if (isTurning){
    isTurning = false;
  } 
}

void turnLeft(){
  Serial.println("Turning left");
  motors.setLeftSpeed(LOW_TURN_VALUE);
  motors.setRightSpeed(HIGH_TURN_VALUE);
}

void turnRight(){
  Serial.println("Turning right");
  motors.setLeftSpeed(HIGH_TURN_VALUE);
  motors.setRightSpeed(LOW_TURN_VALUE);
}


