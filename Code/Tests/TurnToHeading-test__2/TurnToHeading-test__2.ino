#include <EEPROM.h>
#include <FastGPIO.h>
//#include <L3G.h>
#include <Wire.h>
//#include <LSM303.h>
//  #include <PololuHD44780.h>
//#include <Zumo32U4LCD.h>
/*  #include <Pushbutton.h>
  #include <QTRSensors.h>
  #include <USBPause.h>*/
#include <PololuBuzzer.h>
#include <Zumo32U4.h>
//#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Motors.h>
//#include <Zumo32U4LineSensors.h>
#include <Zumo32U4ProximitySensors.h>
#include "Teensy.h"

#define MAX_SPEED 150 //Max is 400
#define MIN_SPEED 70
#define DELTA_SPEED (MAX_SPEED - MIN_SPEED)
#define SCALE_SPEED 10

/*************************
   IR search related
 *************************/
#define DISTANCE_FRONT_THRESHOLD 3 //Readings

uint16_t brightnessLevels_IROff[] = {0, 0, 0, 0, 0, 0 };
uint16_t brightnessLevels_IROn[] = {4, 15, 32, 55, 85, 120 };



/***********************************
   TBD
 ***********************************/
#define EEPROM_ADDRESS 0
unsigned int robotNumber = 0;

#define usTrigPin 13
#define usEchoPin 17


unsigned long lastHeartbeat = 0;
bool _irIsOn = false;



/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!
//LSM303 compass;
Teensy teensy;


/*****************************
   States
 *****************************/
enum State {
  Search_MoveForward,
  Search_FindDirection,
  Search_Alignment,
  Search_Stop,
  Stop,
  Idle
} _state = Idle;


/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/
#define MIN_DISTANCE_FRONT 3
#define IR_SCAN_TRESHOLD 1
uint16_t lastIRLevel = 0;
uint8_t _minDist = 0;
double _otherHeading = 0;
#define ROBOT_WIDTH_MM 98
#define ROBOT_LENGTH_MM 86
/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/

void setup(void) {
  Serial.begin(9600);
  delay(3000);

  EEPROM.get(EEPROM_ADDRESS, robotNumber);

  teensy.initialize();


  delay(5000);

  Serial.println(F("Zumo stands ready!"));
  Serial.print("Zumo number: "); Serial.println(robotNumber);

}



void beep() {
  //Serial.println(F("Beep!"));
  // Start playing a tone with frequency 440 Hz at maximum
  // volume (15) for 200 milliseconds.
  buzzer.playFrequency(440, 200, 15);

  // Delay to give the tone time to finish.
  delay(1000);
}


void drive(int left, int right) {
  //Pre-conditions
  if (SCALE_SPEED < abs(left))
    left = (left / abs(left)) * SCALE_SPEED; //Fancy way to set it to the max
  if (SCALE_SPEED < abs(right))
    right = (right / abs(right)) * SCALE_SPEED; //Fancy way to set it to the max


  int l = left * (DELTA_SPEED / SCALE_SPEED);
  if (l < 0)
    l -= MIN_SPEED;
  else
    l += MIN_SPEED;

  int r = right * (DELTA_SPEED / SCALE_SPEED);
  if (r < 0)
    r -= MIN_SPEED;
  else
    r += MIN_SPEED;
  //int l = MIN_SPEED dd+ left * (DELTA_SPEED/ SCALE_SPEED); // map(left, -SCALE_SPEED, SCALE_SPEED, -MAX_SPEED, MAX_SPEED);
  //int r = right * (DELTA_SPEED/ SCALE_SPEED); //map(right, -SCALE_SPEED, SCALE_SPEED, -MAX_SPEED, MAX_SPEED);

  /*Serial.print("Before conditions: "); Serial.print(l); Serial.print(","); Serial.println(r);
    if (abs(l) < MIN_SPEED)
    l = l < 0 ? -MIN_SPEED : MIN_SPEED;
    if (abs(r) < MIN_SPEED)
    r = r < 0 ? -MIN_SPEED : MIN_SPEED;*/

  //Serial.print("Driving with: "); Serial.print(l); Serial.print(","); Serial.println(r);
  motors.setSpeeds(l, r);
}

void stop() {
  motors.setSpeeds(0, 0);
}


void turnToHeading(double targetHeading, bool turnLeft) {
  targetHeading = fmod(targetHeading + 360.0, 360.0);
  Serial.print(F("Turning to heading ")); Serial.print(targetHeading); Serial.print(F(" (")); Serial.print(turnLeft ? F("left") : F("right")); Serial.println(F(")"));
  int8_t turnerL = turnLeft ? -1 : 1;
  int8_t turnerR = turnerL * -1;

  //Allow the robot to stop completely
  stop();
  delay(1000);

  //for (int i = 0; i < 5; i++) {


  Teensy::Request req;
  req.type = Teensy::TEENSY_REQUEST_HEADING;
  teensy.sendRequest(req);
  while (!teensy.hasNewResponse())
    teensy.checkForIncomingMsg(); //Just idle until response

  double heading =  fmod(teensy.getNextResponse().valueDouble + 360.0, 360.0);
  Serial.print(F("Current heading: ")); Serial.println(heading);




  //bool isLower = heading < targetHeading;
  // bool isLower_whenDone = !isLower;
  bool lastHeadingLessThan180 = heading < 180;
  bool passedZero = false;
  bool done = false;
  bool needsToPassZero = turnLeft ? heading < targetHeading : heading > targetHeading;
  Serial.print("Needs to pass Zero: "); Serial.println(needsToPassZero ?  "true" : "false");

  int speed = 1;
  drive(turnerL * speed, turnerR * speed);

  while (!done) { //isLower_whenDone != isLower) { //while (!passed) {
    // Serial.println("turnHeading loop");
    req.type = Teensy::TEENSY_REQUEST_HEADING;
    teensy.sendRequest(req);
    while (!teensy.hasNewResponse() || teensy.getNextResponse().type != Teensy::TEENSY_RESPONSE_HEADING)
      teensy.checkForIncomingMsg(); //Check for messages until response

    heading = fmod(teensy.getNextResponse().valueDouble + 360.0, 360.0);
    Serial.print("Heading : "); Serial.println(heading);
    double tempHeading = heading;
    if (turnLeft && heading > 180 && lastHeadingLessThan180) {
      Serial.println(F("Passed 0!"));
      passedZero = true;
    }
    if (!turnLeft && heading < 180 && !lastHeadingLessThan180) {
      Serial.println(("Passed 0!"));
      passedZero = true;
    }

    if (needsToPassZero) {
      if (turnLeft)
        done = heading < targetHeading && passedZero;
      else
        done = targetHeading < heading && passedZero;
    } else {
      if (turnLeft)
        done = heading < targetHeading;
      else
        done = targetHeading < heading;
    }

    lastHeadingLessThan180 = tempHeading < 180;
    Serial.print(heading); Serial.print(F(" => ")); Serial.println(targetHeading);
  }
  stop();
  Serial.print(F("Target reached: ")); Serial.println(heading);
}


char buffer[64];
int counter = 0;
void loop(void) {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);
    if (c == ';') {
      buffer[counter] = '\0';
      double h = String(buffer).toFloat();
      counter = 0;
      turnToHeading(h, false);
    } else {
      buffer[counter++] = c;
    }
  }
}

