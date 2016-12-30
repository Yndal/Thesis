#include <Wire.h>
#include <FastGPIO.h>
#include <L3G.h>
#include <LSM303.h>
#include <PololuBuzzer.h>
#include <PololuHD44780.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <USBPause.h>
#include <Zumo32U4.h>
#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4IRPulses.h>
//#include <Zumo32U4LCD.h>
#include <Zumo32U4LineSensors.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4ProximitySensors.h>


#define SPEED_ACC 10
#define SPEED_MAX 300;
#define DISTANCE_FRONT_THRESHOLD_CM 8.0
#define IR_PRESENCE_THREDSHOLD 1
#define IR_SCAN_TRESHOLD 2

#define FRONT_US_TRIG_PIN 14
#define FRONT_US_ECHO_PIN 17

enum State {
  MoveForward,
  ScanIR,
  Stop
};


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;
//LSM303 compass;
Zumo32U4ButtonB buttonB;

State state;
uint16_t brightnessLevels_IROff[] = {0, 0, 0, 0, 0, 0 };
uint16_t brightnessLevels_IROn[] = {4, 15, 32, 55, 85, 120 };

uint16_t currentSpeedLeft = 0;
uint16_t currentSpeedRight = 0;
uint16_t lastIRLevel = 0;

float MAX_HEADING = 360; //TODO Is this correct?

void setup() {
  Serial.begin(9600);
  delay(1000);
  // put your setup code here, to run once:
  //startCompass();
  //TODO Calibrate compass

  proxSensors.initThreeSensors();
  encoders.init();
  pinMode(FRONT_US_TRIG_PIN, OUTPUT);
  pinMode(FRONT_US_ECHO_PIN, INPUT);


  state = MoveForward;
}

void startCompass(){
 /* Wire.begin();
  if (!compass.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the compass."));
      delay(100);
    }
  }
  compass.enableDefault();

  //These be updated to the calibrated values
  compass.m_min = (LSM303::vector<int16_t>){-1147, -35, -26107};
  compass.m_max = (LSM303::vector<int16_t>){+5299, +5007, +18328};*/
//  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
//  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}



void moveForward(int left, int right){
  motors.setSpeeds(left, right);
}

void stop(){
  motors.setSpeeds(0,0);
}

void turnLeft(){
  motors.setSpeeds(-100, 100);
}

void turnRight(){
  motors.setSpeeds(100, -100);
}

void turnLeftSlowly(){
  motors.setSpeeds(-80, 80);
}

void turnRightSlowly(){
  motors.setSpeeds(80, -80);
}

void loop() {
  if(buttonB.isPressed())
    state = MoveForward;
    
  Serial.print("State: ");
  switch(state){
    case MoveForward:{
        Serial.print("Move forward");
        if(distanceFrontCM() <= DISTANCE_FRONT_THRESHOLD_CM){
          state = Stop;
          break;
        }

        //Move forward, while the ir level is not decreasing
        uint8_t irLevel = getIRLevel(false);
        Serial.print(" (IR level: "); Serial.print(irLevel); Serial.println(")");
        if(irLevel + IR_SCAN_TRESHOLD < lastIRLevel){
            state = ScanIR;
            lastIRLevel = 0; //Reset...
        }
        lastIRLevel = irLevel;
        
        moveForward(100,100);

      break;
    }

    case ScanIR:{
      Serial.println("Scan IR");
        //Stop vehicle
      stop();
      delay(1000);

      //Reset encoders
      //TODO Include counting right encoder?
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      int16_t encodersStart = encoders.getCountsLeft(); //This should be 0...
      
      //Slowly move left while measuring IR level - when level has dropped with 2 levels or reached 0: Save direction
      uint16_t irMax = getIRFront(false);
      uint16_t currentFrontIR = irMax;
      turnLeft();
      //int16_t encoderLast = encoders.getCountsLeft();
      while(irMax <= currentFrontIR + IR_SCAN_TRESHOLD && 0 < currentFrontIR ||
            currentFrontIR <= getIRLeft(false)){
        if(irMax < currentFrontIR)
          irMax = currentFrontIR;  
      
        currentFrontIR = getIRFront(false);
      }
      //Serial.print("Overflows:"); Serial.println(encoderOverflows);
      int16_t leftBorder = encoders.getCountsLeft(); //encoderOverflows * 65535 + encodersStart;
      //uint16_t overflowsToStartinPoint = encoderOverflows;
      Serial.print("LeftBorder:"); Serial.println(leftBorder);
      
      stop();
      delay(1000);
     
      //Move back to starting point
      bool keepMoving = true;
      turnRight();
      while(keepMoving){
        if(encoders.getCountsLeft() <= 0)
          keepMoving = false;
      }

      Serial.println("Starting point reached");
      //stop();
      //delay(2000);
      //Now do it to the right side
      currentFrontIR = getIRFront(false);
      turnRight();
      while(irMax <= currentFrontIR + IR_SCAN_TRESHOLD && 0 < currentFrontIR ||
            currentFrontIR <= getIRRight(false)){
        if(irMax < currentFrontIR)
          irMax = currentFrontIR; 

        currentFrontIR = getIRFront(false);
      }
      stop();
      delay(1000);
      int16_t rightBorder = encoders.getCountsLeft();
      Serial.print("Right border: "); Serial.println(rightBorder);
      int16_t center = rightBorder + (leftBorder + (-rightBorder))/2;
      
      //encoderLast = encoders.getCountsLeft(); 
      bool passed = false;
      turnLeft();
      while(!passed){
        if(center <= encoders.getCountsLeft())
          passed = true;
        //passed = compass.heading() < dir; 
      }
     
      state = MoveForward;
      break;
    }
    
    case Stop:{
      Serial.println("Stop");
        stop();
      break;
    }
    defaults:
      Serial.println("???? Defaulted...");
      //Nothing...
      break;
  }
}



double distanceFrontCM(){
  long duration, distance;
  digitalWrite(FRONT_US_TRIG_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(FRONT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_US_TRIG_PIN, LOW);
  duration = pulseIn(FRONT_US_ECHO_PIN, HIGH);
  distance = (duration/2) / 29.1; //Convert from time-of-flight into cm

  Serial.print("Front distance: "); Serial.println(distance);
  
  return distance;
}

void disableIREmitters(){
  proxSensors.setBrightnessLevels(brightnessLevels_IROff,6); 
}

void enableIREmitters(){
  proxSensors.setBrightnessLevels(brightnessLevels_IROn,6);
}

uint8_t getIRFront(bool emitIR){
  if(emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();
  
  uint8_t front = proxSensors.countsFrontWithLeftLeds() +
                  proxSensors.countsFrontWithRightLeds();
  
  return front;
}

uint8_t getIRLeft(bool emitIR){
  if(emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();
  
  uint8_t left = proxSensors.countsLeftWithLeftLeds() +
                 proxSensors.countsLeftWithRightLeds();
  
  return left;
}

uint8_t getIRRight(bool emitIR){
  if(emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();
  
  uint8_t right = proxSensors.countsRightWithLeftLeds() +
                  proxSensors.countsRightWithRightLeds();
  
  return right;
}

uint8_t getIRLevel(bool emitIR){
  
  uint8_t frontValue = getIRFront(emitIR);
  uint8_t leftValue = getIRLeft(emitIR);
  uint8_t rightValue = getIRRight(emitIR);

  return frontValue + leftValue + rightValue;
}

