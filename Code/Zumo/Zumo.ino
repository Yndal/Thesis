#include <EEPROM.h>
#include <FastGPIO.h>
#include <Zumo32U4.h>
#include "Teensy.h"



/**********************************
   Helping functions
 **********************************/
#define DegToRad(x) (x*(PI/180))


//#define HEARTBEAT_CHAR        '*'
//#define HEARTBEAT_INTERVAL_MS 500

#define MAX_SPEED 150 //Max is 400
#define MIN_SPEED 75
#define DELTA_SPEED (MAX_SPEED - MIN_SPEED)
#define SCALE_SPEED 10

/*************************
   IR search related
 *************************/
#define DISTANCE_FRONT_THRESHOLD 3 //Readings

//const uint16_t brightnessLevels_IROff[] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0};
//const uint16_t brightnessLevels_IROn[]  = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 49, 55, 65, 75, 85, 120};
//uint16_t brightnessLevels_IROff[] = {0, 0, 0, 0, 0, 0 };
//uint16_t brightnessLevels_IROn[] = {4, 15, 32, 55, 85, 120 };

#define IR_SCAN_TIME 10 //ms
#define IR_EMIT_TIME 100 //ms
#define IR_TIME_BUFFER 700 //ms

#define IR_SCAN_TRESHOLD 50
uint32_t lastIRLevel = 0;

/***********************************
   TBD
 ***********************************/
#define EEPROM_ADDRESS 0
uint8_t robotNumber = 0;

#define usFrontTrigPin 17
#define usFrontEchoPin 13

#define usLeftTrigPin 12
#define usLeftEchoPin 14

#define usRightTrigPin 2
#define usRightEchoPin 3

enum DistanceDirection {
  Left, Front, Right
};


//unsigned long lastHeartbeat = 0;
bool _emitIR = false;



/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
//Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!
//LSM303 compass;
Teensy teensy;


/*****************************
   States
 *****************************/
enum State {
  Search_FindInitialDirection,
  Search_MoveForward,
  Search_FindDirection,
  Search_Alignment,
  Search_Docking,
  Search_Stop,
  Stop,
  Working,
  Idle,
} _state = Idle;


/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/
#define MIN_DISTANCE_FRONT 8

uint8_t _minDist = 0;
double _otherHeading = 0;
#define ROBOT_WIDTH_MM 98
#define ROBOT_LENGTH_MM 86

#define WORKING_DISTANCE 5//3.5 //cm

bool _dockingDoStrafe = true;
bool _dockingTurningLeft = true;
bool _dockingLaserIsDetected = false;
unsigned long _dockingLastLaserDetection = 0;

//Drive XXX mm away from the robot
#define ALIGNMENT_SAFETY_DISTANCE  80 //mm

#define DOCKING_LASER_NOT_DETECTED_TIME 2000 //Ms to wait before considered to be docked, because the laser can't be detected
unsigned long _lastLaserDetectionTime = -1 - DOCKING_LASER_NOT_DETECTED_TIME;
#define STRAFE_MM 20 //mm
/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/

void setup(void) {
  pinMode(usLeftTrigPin, OUTPUT);
  pinMode(usLeftEchoPin, INPUT);
  pinMode(usFrontTrigPin, OUTPUT);
  pinMode(usFrontEchoPin, INPUT);
  pinMode(usRightTrigPin, OUTPUT);
  pinMode(usRightEchoPin, INPUT);

  Serial.begin(9600);

  delay(3000);
  EEPROM.get(EEPROM_ADDRESS, robotNumber);

  //Serial.print(F("Zumo number: ")); Serial.println(robotNumber);

  teensy.initialize();
  proxSensors.initThreeSensors();

  //Timer3.initialize((1/38000)*1000000); //Blink at 38k Hz (when interrupt is attached)

  //_otherHeading = 0.0;
  //_state = Search_Alignment;
  /*#if DEBUG
    Serial.println(F("Zumo stands ready!"));
    #endif*/
  //beep();
}

void handleIncomingRequest_Teensy() {
  Teensy::Request req = teensy.getNextRequest();
  switch (req.type) {
    case Teensy::TEENSY_REQUEST_IR_ON: {
        _emitIR = true;
        break;
      }
    case Teensy::TEENSY_REQUEST_IR_OFF: {
        _emitIR = false;
        break;
      }
    case Teensy::TEENSY_REQUEST_BEEP: {
        beep();
        break;
      }
    case Teensy::TEENSY_REQUEST_DRIVE: {
        drive(req.value1, req.value2);
        break;
      }
    case Teensy::TEENSY_REQUEST_DRIVE_MM: {
        driveMm(req.value1, req.value2);
        break;
      }
    case Teensy::TEENSY_REQUEST_TURN_LEFT: {
        drive(-1, 1);
        break;
      }
    case Teensy::TEENSY_REQUEST_STOP: {
        stop();
        _state = Idle;
        break;
      }

    case Teensy::TEENSY_REQUEST_WORK: {
        _state = Working;
        break;
      }

    case Teensy::TEENSY_REQUEST_IR_LEFT: {
        uint32_t value = getIRLeft(req.value1);
        Teensy::Response response;
        response.type = Teensy::TEENSY_RESPONSE_IR_LEFT;
        response.value1 = value;
        teensy.sendResponse(response);
        break;
      }
    case Teensy::TEENSY_REQUEST_IR_FRONT: {
        uint32_t value = getIRFront(req.value1);
        Teensy::Response response;
        response.type = Teensy::TEENSY_RESPONSE_IR_FRONT;
        response.value1 = value;
        teensy.sendResponse(response);
        break;
      }
    case Teensy::TEENSY_REQUEST_IR_RIGHT: {
        uint32_t value = getIRRight(req.value1);
        Teensy::Response response;
        response.type = Teensy::TEENSY_RESPONSE_IR_RIGHT;
        response.value1 = value;
        teensy.sendResponse(response);
        break;
      }
    case Teensy::TEENSY_REQUEST_COARSE_LOCALIZATION: {
        _state = Search_FindInitialDirection;//Search_FindDirection;
        _minDist = req.value1;
        if (_minDist < 5)
          _minDist = 5;
        break;
      }

    case Teensy::TEENSY_REQUEST_ALIGMENT: {
        _state = Search_Alignment;
        _otherHeading = req.valueDouble;
        break;
      }

    case Teensy::TEENSY_REQUEST_DOCKING: {

        _otherHeading = req.valueDouble;//TODO Remove when not testing

        //_otherHeading is -180 <--> 180
        _otherHeading += 180;
        if (_otherHeading > 180)
          _otherHeading -= 360;

        _dockingLastLaserDetection = 0;
        _dockingDoStrafe = true;
        _dockingLaserIsDetected = false;
        _state = Search_Docking;

        break;
      }
  }
}



//uint16_t brightnesses[] = { 4, 15, 32, 55, 85, 120 };
//uint16_t brightnessCount = sizeof(brightnesses) / sizeof(uint16_t);
void emitIrSignal() {
  unsigned long endTime = millis() + IR_EMIT_TIME;
  while (millis() < endTime) {
    proxSensors.read();
  }
  /*

    uint8_t brightnessCount = sizeof(brightnessLevels_IROn) / sizeof(uint16_t);
    for (int a = 0; a < 10; a++) {
      for (int i = 0; i < brightnessCount; i++) {
        irPulses.start(Zumo32U4IRPulses::Left, brightnessLevels_IROn[i], Zumo32U4IRPulses::defaultPeriod);
        delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOnTimeUs);
        irPulses.start(Zumo32U4IRPulses::Right, brightnessLevels_IROn[i], Zumo32U4IRPulses::defaultPeriod);
        delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOnTimeUs);
      }
    }
    irPulses.stop();
    delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOffTimeUs);
  */
}


/*void turnIrOn() {
  #if DEBUG
  //Serial.println(F("Turning IR on"));
  #endif
  _irIsOn = true;
  //Timer3.attachInterrupt(blinkIR)

  }

  void turnIrOff() {
  //Serial.println(F("Turning IR off"));
  _irIsOn = false;
  irPulses.stop();
  //Timer3.detachInterrupt();
  }*/


void beep() {
  //Serial.println(F("Beep!"));
  // Start playing a tone with frequency 440 Hz at maximum
  // volume (15) for 200 milliseconds.
  buzzer.playFrequency(440, 200, 15);

  // Delay to give the tone time to finish.
  delay(300);
}


/*
   Can drive 2.2 meters before the encoders will overflow
*/
void driveMm(double mm, int speed) {
  Serial.print("Driving mm: "); Serial.println(mm);
  // double encoderCPR = 12.0;
  // double gearRatio = 150.58;
  // double wheelCircumferenceInMm = 39 * PI;
  //float tickCountForOneMm =  (encoderCPR * gearRatio) / wheelCircumferenceInMm * -1;
  float tickCountForOneMm =  (12 * 150.58) / (39 * PI) * -1;
  float totalTicks = mm * tickCountForOneMm;
  totalTicks *= 1.07; //Minor adjustments...

  //Serial.print("Driving ticks: "); Serial.println(totalTicks);
  //TODO Make sure that both wheels drive the exact same distance
  // long right = encoders.getCountsAndResetRight();
  // long left = encoders.getCountsAndResetLeft();

  //If speed and distance is going in each direction...
  if (mm * speed < 0)
    speed *= -1;
  drive(speed, speed);
  bool passed = false;
  bool stalling = false;
  int stallInterval = 1000;
  long lastStallCheckTicksLeft = encoders.getCountsAndResetLeft();
  long lastStallCheckTicksRight = encoders.getCountsAndResetRight();
  delay(50); //Drive a bit before checking...
  long nextStallingCheck = millis() + stallInterval;
  //Serial.println(F("How about using getDistanceFront() here?"));

  if (0 < mm) {
    while (!passed && !stalling) {
      passed = encoders.getCountsRight() < totalTicks;

      if (nextStallingCheck < millis()) {
        if ((encoders.getCountsRight() == lastStallCheckTicksRight) ||
            (encoders.getCountsLeft() == lastStallCheckTicksLeft)) {
          stalling = true;
        }
        lastStallCheckTicksRight = encoders.getCountsRight();
        lastStallCheckTicksLeft = encoders.getCountsLeft();
        nextStallingCheck = millis() + stallInterval;
      }
    }
  } else {
    while (!passed && !stalling) {
      passed = totalTicks < encoders.getCountsRight();
      if (nextStallingCheck < millis()) {
        if ((encoders.getCountsRight() == lastStallCheckTicksRight) ||
            (encoders.getCountsLeft() == lastStallCheckTicksLeft))
          stalling = true;
        lastStallCheckTicksRight = encoders.getCountsRight();
        lastStallCheckTicksLeft = encoders.getCountsLeft();
        nextStallingCheck = millis() + stallInterval;
      }
    }
  }
  stop();
}




void drive(int left, int right) {
  //Serial.print(F("Drive called: ")); Serial.print(left); Serial.print(F(",")); Serial.println(right);
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

  if (left == 0)
    l = 0;
  if (right == 0)
    r = 0;
  //Serial.print(F("Driving with: ")); Serial.print(l); Serial.print(F(",")); Serial.println(r);
  motors.setSpeeds(l, r);
}

void stop() {
  motors.setSpeeds(0, 0);
}

double getDistance(DistanceDirection dir, int times) {
  int usTrigPin;
  int usEchoPin;
  switch (dir) {
    case Left: {
        usTrigPin = usLeftTrigPin;
        usEchoPin = usLeftEchoPin;
        break;
      }
    case Front: {
        usTrigPin = usFrontTrigPin;
        usEchoPin = usFrontEchoPin;
        break;
      }
    case Right: {
        usTrigPin = usRightTrigPin;
        usEchoPin = usRightEchoPin;
        break;
      }
  }


  double duration = 0,
         distance = 0;

  for (int i = 0; i < times; i++) {
    delay(10);
    digitalWrite(usTrigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(usTrigPin, HIGH);
    //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(usTrigPin, LOW);
    duration = pulseIn(usEchoPin, HIGH);
    //distance += (duration / 2) / 29.1;
    distance += ((duration / 2) / 29.1) / times;
    delay(10);
  }
  //distance /= times;

  return distance;
}

double getDistance(DistanceDirection dir) {
  return getDistance(dir, 5);
}


/*void disableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROff, sizeof(brightnessLevels_IROff) / sizeof(uint16_t));
  }

  void enableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROn, sizeof(brightnessLevels_IROn) / sizeof(uint16_t));
  }*/

uint32_t getIRFront(bool emitIR) {

  unsigned long endTime = millis() + IR_SCAN_TIME;
  uint32_t total = 0;
  if (emitIR) {
    while (millis() < endTime) {
      proxSensors.read();
      total += proxSensors.countsFrontWithLeftLeds() +
               proxSensors.countsFrontWithRightLeds();
    }
  } else {
    while (millis() < endTime) {
      total += proxSensors.readBasicFront();
    }
  }
  return total;
  /*
    return   enableIREmitters();
    else
      disableIREmitters();
    proxSensors.read();

    uint8_t front = proxSensors.countsFrontWithLeftLeds() +
                    proxSensors.countsFrontWithRightLeds();

    if (irOn)
      turnIrOn();

    return front;*/
}

uint32_t getIRLeft(bool emitIR) {
  unsigned long endTime = millis() + IR_SCAN_TIME;
  uint32_t total = 0;
  if (emitIR) {
    while (millis() < endTime) {
      proxSensors.read();
      total += proxSensors.countsLeftWithLeftLeds() +
               proxSensors.countsLeftWithRightLeds();
    }
  } else {
    while (millis() < endTime) {
      total += proxSensors.readBasicLeft();
    }
  }
  return total;


  /*bool irOn = _irIsOn;
    turnIrOff();

    if (emitIR)
    enableIREmitters();
    else
    disableIREmitters();
    proxSensors.read();

    uint8_t left = proxSensors.countsLeftWithLeftLeds() +
                 proxSensors.countsLeftWithRightLeds();
    if (irOn)
    turnIrOn();

    return left;*/
}

uint32_t getIRRight(bool emitIR) {
  unsigned long endTime = millis() + IR_SCAN_TIME;
  uint32_t total = 0;
  if (emitIR) {
    while (millis() < endTime) {
      proxSensors.read();
      total += proxSensors.countsRightWithLeftLeds() +
               proxSensors.countsRightWithRightLeds();
    }
  } else {
    while (millis() < endTime) {
      total += proxSensors.readBasicRight();
    }
  }
  return total;

  /*bool irOn = _irIsOn;
    turnIrOff();

    if (emitIR)
    enableIREmitters();
    else
    disableIREmitters();
    proxSensors.read();

    uint8_t right = proxSensors.countsRightWithLeftLeds() +
                  proxSensors.countsRightWithRightLeds();
    if (irOn)
    turnIrOn();

    return right;*/
}

uint32_t getIRTotal(bool emitIR) {

  uint32_t frontValue = getIRFront(emitIR);
  uint32_t leftValue = getIRLeft(emitIR);
  uint32_t rightValue = getIRRight(emitIR);

  return frontValue + leftValue + rightValue;
}

void turnToHeading(double targetHeading, bool turnLeft) {
  targetHeading = fmod(targetHeading + 360.0, 360.0);
  if (180 < targetHeading)
    targetHeading -= 360;
  //  Serial.print(F("Turning to heading:"));
  //  Serial.print(targetHeading);
  //Serial.print(F(" ("));
  //  Serial.print(turnLeft ? F("left") : F("right")); Serial.println(F(")"));

  int8_t turnerL = turnLeft ? -1 : 1;
  int8_t turnerR = turnerL * -1;

  //Allow the robot to stop completely
  stop();
  delay(500);




  Teensy::Request req;
  req.type = Teensy::TEENSY_REQUEST_HEADING;
  teensy.sendRequest(req);
  //delay(200);
  unsigned long nextRequestTime = 0;
  do {
    teensy.checkForIncomingMsg();
    if (nextRequestTime < millis()) {
      teensy.sendRequest(req);
      nextRequestTime = millis() + 300;
    }
  } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));

  double heading =  fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
  if (180 < heading)
    heading -= 360;
  //Serial.print(F("Current heading: ")); Serial.println(heading);

  bool lastHeadingLessThanZero = heading < 0;
  bool passed180 = false;
  bool done = false;
  bool needsToPass180 = turnLeft ? heading < targetHeading : heading > targetHeading;

  Serial.print(F("Needs to pass 180: ")); Serial.println(needsToPass180 ?  "true" : "false");


  int8_t rounds = 0;

  uint8_t speed = 1;
  drive(turnerL * speed, turnerR * speed);

  while (!done) { //isLower_whenDone != isLower) { //while (!passed) {

    do {
      teensy.checkForIncomingMsg();
      if (nextRequestTime < millis()) {
        teensy.sendRequest(req);
        nextRequestTime = millis() + 300;
      }
    } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));


    /*  req.type = Teensy::TEENSY_REQUEST_HEADING;
      teensy.sendRequest(req);
      while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING))// || teensy.getNextResponse().type != Teensy::TEENSY_RESPONSE_HEADING)
        teensy.checkForIncomingMsg(); //Check for messages until response
    */

    heading = fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
    if (180 < heading)
      heading -= 360;
    //    heading += rounds * 360;
    //Serial.print("Heading : "); Serial.println(heading);
    double tempHeading = heading;
    if (turnLeft && heading > 0 && lastHeadingLessThanZero) {
      //Serial.println(F("Passed 0!"));
      if (!passed180)
        rounds--;
      passed180 = true;

    }
    if (!turnLeft && heading < 0 && !lastHeadingLessThanZero) {
      Serial.println(("Passed 0!"));
      if (!passed180)
        rounds++;
      passed180 = true;
    }

    if (needsToPass180) {
      if (turnLeft)
        done = heading <= targetHeading && passed180;
      else
        done = targetHeading <= heading && passed180;
    } else {
      if (turnLeft)
        done = heading <= targetHeading;
      else
        done = targetHeading <= heading;

      if (passed180)
        done = true;
    }

    lastHeadingLessThanZero = tempHeading < 0;

    Serial.print(heading);
    Serial.print(F(" => "));
    Serial.println(targetHeading);

  }
  stop();

  //Serial.print(F("Heading reached: "));
  //Serial.println(heading);
}

void turnToHeadingWithDistance(DistanceDirection sensor, double targetHeading, double distanceMm) {
  targetHeading = fmod(targetHeading + 360.0, 360.0);
  if (180 < targetHeading)
    targetHeading -= 360;

  // Serial.print(F("Turning to heading with distance: "));
  // Serial.print(targetHeading); Serial.print(F(" with distance "));
  // Serial.println(distanceMm);

  //int8_t turnerL = turnLeft ? -1 : 1;
  //  int8_t turnerL = sensor == Left ? -1 : 1;
  //  int8_t turnerR = turnerL * -1;

  //Allow the robot to stop completely
  stop();
  delay(500);




  Teensy::Request req;
  req.type = Teensy::TEENSY_REQUEST_HEADING;
  teensy.sendRequest(req);
  //delay(200);
  unsigned long nextRequestTime = 0;
  do {
    teensy.checkForIncomingMsg();
    if (nextRequestTime < millis()) {
      teensy.sendRequest(req);
      nextRequestTime = millis() + 300;
    }
  } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));

  double heading =  fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
  if (180 < heading)
    heading -= 360;
  // Serial.print(F("Current heading: ")); Serial.println(heading);

  bool lastHeadingLessThanZero = heading < 0;
  bool passed180 = false;
  bool done = false;
  bool needsToPass180 = sensor == Left ? heading < targetHeading : heading > targetHeading;
  //  Serial.print(F("Needs to pass 180: ")); Serial.println(needsToPass180 ?  "true" : "false");


  int8_t rounds = 0;

  uint8_t speed = 1;
  //drive(turnerL * speed, turnerR * speed);


  bool keepDriving = true;
  bool isCorrectingDecreaseDist = false;
  bool isCorrectingIncreaseDist = false;
  //long ticksToTurn = 800;
  unsigned long lastSwitch = millis();
  drive(2, 2);
  double dist;
  bool isTurningLeft = sensor == Left;
  int passed180counter = 0;
  while (!done) {
    /************************
       The distance part
     *************************/
    dist = getDistance(sensor) * 10; //Cm => mm
    /*Serial.print("Dist mm: ");
      Serial.print(dist);
      Serial.print(" - ");
      Serial.print(distanceMm);*/
    if (dist < distanceMm && !isCorrectingIncreaseDist) {
      //Serial.println(" - Increasing");
      lastSwitch = millis();
      if (sensor == Left) {
        drive(3, 0);
        isTurningLeft = false;
        //driveOneWheelTicks(ticksToTurn, true);
      } else {
        drive(0, 3);
        isTurningLeft = true;
      }
      //driveOneWheelTicks(ticksToTurn, false);
      isCorrectingIncreaseDist = true;
      isCorrectingDecreaseDist = false;
    } else if (dist > distanceMm && !isCorrectingDecreaseDist) {
      //  Serial.println(" - Decreasing");
      lastSwitch = millis();
      //int ticks = dist - distanceMm > 100 ? ticksToTurn*2 : ticksToTurn;
      if (sensor == Left) {
        drive(0, 3);
        isTurningLeft = true;
        //driveOneWheelTicks(ticks, false);
      } else {
        drive(3, 0);
        isTurningLeft = false;
      }
      //driveOneWheelTicks(ticks, true);
      isCorrectingIncreaseDist = false;
      isCorrectingDecreaseDist = true;
    }

    if (lastSwitch + 2000 < millis()) {
      driveMm(50, 2);
      isCorrectingIncreaseDist = false;
      isCorrectingDecreaseDist = false;
      encoders.getCountsAndResetLeft();
      //delay(1500);
      //continue;
    }

    //  Serial.println();


    /************************
       The heading part
     *************************/
    //Emptying...
    teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING);
    do {
      teensy.checkForIncomingMsg();
      if (nextRequestTime < millis()) {
        teensy.sendRequest(req);
        nextRequestTime = millis() + 100;
      }
    } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));

    heading = fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
    if (180 < heading)
      heading -= 360;
    //    heading += rounds * 360;
    //Serial.print("Heading : "); Serial.println(heading);
    double tempHeading = heading;
    if ((sensor == Left && isTurningLeft) && heading > 0 && lastHeadingLessThanZero) {
      //     Serial.println(F("Passed 0!"));
      if (!passed180)
        rounds--;
      passed180 = true;

    }
    if ((sensor == Right && !isTurningLeft) && heading < 0 && !lastHeadingLessThanZero) {
      //    Serial.println(("Passed 0!"));
      if (!passed180)
        rounds++;
      passed180 = true;
    }

    if (needsToPass180) {
      if (passed180 && (passed180counter % 2 == 1)) {
        if (sensor == Left && isTurningLeft)
          done = heading <= targetHeading;
        else if (sensor == Right && !isTurningLeft)
          done = targetHeading <= heading;
      }
    } else {
      if (sensor == Left && isTurningLeft)
        done = heading <= targetHeading;
      else if (sensor == Right && !isTurningLeft)
        done = targetHeading <= heading;

      if (passed180)
        done = true;
    }

    if (tempHeading > 0 && lastHeadingLessThanZero)
      passed180counter++;
    else if (tempHeading < 0 && !lastHeadingLessThanZero)
      passed180counter++;

    lastHeadingLessThanZero = tempHeading < 0;

    /*Serial.print(heading);
      Serial.print(F(" ==> "));
      Serial.print(targetHeading);
      Serial.print(". Turning ");
      Serial.print(isTurningLeft ? "left" : "right");
      Serial.print(" - passes180Counter is ");
      Serial.println(passed180counter);*/
  }
  // Serial.print(F("Reached heading: "));
  // Serial.print(heading);

  stop();
  beep();
  delay(500);

}

/*void driveOneWheelTicks(long ticks, bool leftWheel) {
  Serial.print("driveOnWheelTicks (");
  Serial.print(ticks);
  Serial.print("    , leftWheel: ");
  Serial.print(leftWheel ? "true" : "false");
  Serial.println(")");

  if (leftWheel)
    encoders.getCountsAndResetLeft();
  else
    encoders.getCountsAndResetRight();

  if (ticks > 0) {
    if (leftWheel) {
      drive(1, 0);
      while (-encoders.getCountsLeft() < ticks) {
        //     Serial.print(-encoders.getCountsLeft());
        //     Serial.print(" ===> ");
        //     Serial.println(ticks);
      }
    } else {
      drive(0, 1);
      while (-encoders.getCountsRight() < ticks)
        //     Serial.println(encoders.getCountsLeft())
        ;
    }
  } else {
    if (leftWheel) {
      drive(-1, 0);
      while (-encoders.getCountsLeft() > ticks);
    } else {
      drive(0, -1);
      while (-encoders.getCountsRight() > ticks);
    }
  }

  stop();
  }*/


/*void driveWidthDistance(DistanceDirection sensor, uint16_t distance) {
  bool keepDriving = true;
  while (keepDriving) {
    int dist = getDistance(sensor);
    if (dist < SAFETY_DISTANCE && !isCorrectingIncreaseDist) {
      if (sensor == Right)
        drive(1, 2);
      else if (sensor == Left)
        drive(2, 1);
      delay(100);
      isCorrectingIncreaseDist = true;
    } else if (dist > SAFETY_DISTANCE && !isCorrectingDecreaseDist) {
      if (sensor == Right)
        drive(2, 1);
      else if (sensor == Left)
        drive(1, 2);
      delay(100);
      isCorrectingDecreaseDist = true;
    }

    if (isCorrectingIncreaseDist && dist >= SAFETY_DISTANCE) {
      isCorrectingIncreaseDist = false;
      drive(1, 1);
    }
    if (isCorrectingDecreaseDist && dist <= SAFETY_DISTANCE) {
      isCorrectingDecreaseDist = false;
      drive(1, 1);
    }
  }
  }*/

void scanAndGoToHeadingOfClosestObject(unsigned long scanTime) {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  long encoder = 0;
  double closestDistance = getDistance(Front);
  unsigned long endTime = millis() + scanTime;

  drive(-1, 1);
  while (millis() < endTime) {
    double dist = getDistance(Front, 1);
    if (closestDistance > dist) {
      closestDistance = dist;
      encoder = encoders.getCountsLeft();
    }
  }
  stop();
  delay(500);
  //  encoder = encoders.getCountsLeft();
  //Scan to the right side
  endTime = millis() + 2 * scanTime; //Scan all the way back plus to the right

  drive(1, -1);
  while (millis() < endTime) {
    double dist = getDistance(Front, 1);

    if (closestDistance > dist) {
      closestDistance = dist;
      encoder = encoders.getCountsLeft();
    }
  }
  stop();
  delay(500);

  drive(-1, 1);

  while (encoders.getCountsLeft() < encoder) ;
  stop();
  delay(500);


  /*  drive(1, 1);
          if (center < 0)
            while (encoders.getCountsLeft() < center);
          else
            while (encoders.getCountsLeft() > center);
          stop();*/
}

void handleCurrentState() {
  switch (_state) {
    case Stop: {
        stop();
        _state = Idle;
        break;
      }

    case Working: {
        if (getDistance(Front) > WORKING_DISTANCE)
          drive(1, 1);
        else
          stop();
        break;
      }
    case Idle: {
        //Serial.println("Idle");
        //Do nothing...
        break;
      }

    case Search_FindInitialDirection: {
        //back up 10 cm
        //driveMm(-100, -2);


        //Get current heading
        Teensy::Request req;
        req.type = Teensy::TEENSY_REQUEST_HEADING;
        teensy.sendRequest(req);
        //delay(200);
        unsigned long nextRequestTime = 0;
        do {
          teensy.checkForIncomingMsg();
          if (nextRequestTime < millis()) {
            teensy.sendRequest(req);
            nextRequestTime = millis() + 500;
          }
        } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));

        double startHeading =  fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
        if (180 < startHeading)
          startHeading -= 360;

        //turn left
        drive(-1, 1);

        //Measure IR level for 360 degress and store the heading of the highest IR level
        double highestIR_direction = startHeading;
        uint32_t highestIR_level = 0;// = getIRFront(false);
        bool lastHeadingLessThanZero = startHeading < 0;
        bool passed180 = false;
        bool done = false;
        double heading;

        int8_t rounds = 0;

        while (!done) {

          do {
            teensy.checkForIncomingMsg();
            if (nextRequestTime < millis()) {
              teensy.sendRequest(req);
              nextRequestTime = millis() + 300;
            }
          } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));


          heading = fmod(teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble + 360.0, 360.0);
          if (180 < heading)
            heading -= 360;

          //Compare IR levels
          uint32_t curIR = getIRFront(false);
          if (curIR > highestIR_level) {
            highestIR_level = curIR;
            highestIR_direction = heading;
          }

          //    heading += rounds * 360;
          //Serial.print("Heading : "); Serial.println(heading);
          double tempHeading = heading;
          if (heading > 0 && lastHeadingLessThanZero) {
            //Serial.println(F("Passed 0!"));
            if (!passed180)
              rounds--;
            passed180 = true;

          }

          done = heading <= startHeading && passed180;

          lastHeadingLessThanZero = tempHeading < 0;
        }
        stop();
        beep();
        delay(500);

        //TODO Figure out which way is the fastest to turn
        turnToHeading(highestIR_direction, true);
        beep();
        delay(500);
        beep();
        delay(500);

        // break;

        _state = Search_MoveForward;
        break;
      }

    case Search_MoveForward: {
        /*#if DEBUG
                Serial.println(F("1: Move forward"));
          #endif*/
        drive(2, 2);
        /* Teensy::Request req;
          req.type = Teensy::TEENSY_REQUEST_IR_OFF;
          teensy.sendRequest(req);
          delay(200); //Allow for time to react*/

        double dist = getDistance(Front);//getIRFront(true);
        /*#if DEBUG
                Serial.print(F("Distance: "));
                Serial.println(dist);
          #endif*/
        if (dist <= _minDist || dist <= MIN_DISTANCE_FRONT) {
          /*Teensy::Response res;
            res.type = Teensy::TEENSY_RESPONSE_STOPPED_FRONT_MIN_DISTANCE;
            teensy.sendResponse(res);*/
          stop();
          //Serial.println(F("Coarse localization finished"));
          beep();
          Teensy::Response res;
          res.type = Teensy::TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED;
          teensy.sendResponse(res);
          _state = Stop;


          break;
        }
        /* req.type = Teensy::TEENSY_REQUEST_IR_ON;
          teensy.sendRequest(req);
          delay(200); //Allow for time to react
        */

        //Move forward, while the ir level is not decreasing
        uint32_t irLevel = getIRFront(false);//getIRTotal(false);
        //Serial.print(F(" (IR level: ")); Serial.print(irLevel); Serial.println(F(")"));
        if (irLevel + IR_SCAN_TRESHOLD < lastIRLevel) {
          //_state = Stop;
          _state = Search_FindDirection;
          /*Teensy::Response res;
            res.type = Teensy::TEENSY_RESPONSE_STOPPED;
            teensy.sendResponse(res);*/
          //SearchIR_FindDirection;TODO
          lastIRLevel = 0; //Reset...
        }
        lastIRLevel = irLevel;

        //drive(2, 2);

        break;
      }
    case Search_FindDirection: {
        //Stop vehicle
        stop();
        delay(2000);
        _emitIR = false;//turnIrOff();
        Teensy::Request req;
        req.type = Teensy::TEENSY_REQUEST_IR_ON;
        teensy.sendRequest(req);
        delay(1000);

        //Reset encoders
        //TODO Include counting right encoder?
        encoders.getCountsAndResetLeft();
        encoders.getCountsAndResetRight();
        int16_t encodersStart = encoders.getCountsLeft(); //This should be 0...

        //Slowly move left while measuring IR level - when level has dropped with 2 levels or reached 0: Save direction
        uint32_t irMax = getIRFront(false);
        uint32_t currentFrontIR = irMax;
        // uint32_t currentLeftIR = getIRLeft(false);
        //Serial.print("CurrentFrontIR");  Serial.println(currentFrontIR);

        drive(-2, 2);
        //int16_t encoderLast = encoders.getCountsLeft();
        unsigned long timeOfLastGoodValue = millis();
        //while (millis() < timeOfLastGoodValue  + IR_TIME_BUFFER) {
        bool goodValue = true;
        while ((millis() < timeOfLastGoodValue  + IR_TIME_BUFFER) ||
               goodValue)

          /*(irMax <= currentFrontIR + IR_SCAN_TRESHOLD &&
            (0 < currentFrontIR || currentFrontIR <= currentLeftIR)))*/ {
          //Serial.print("currentLeftIR:  ");  Serial.println(currentLeftIR);
          //Serial.print("currentFrontIR: ");  Serial.println(currentFrontIR);
          //Serial.print("Last good IR value: " ); Serial.println(timeOfLastGoodValue);

          if (irMax < currentFrontIR)
            irMax = currentFrontIR;

          //  currentLeftIR = getIRLeft(false);
          currentFrontIR = getIRFront(false);
          goodValue = irMax <= currentFrontIR + IR_SCAN_TRESHOLD &&
                      (0 < currentFrontIR);// || currentFrontIR <= currentLeftIR);
          if (goodValue)
            timeOfLastGoodValue = millis();

          if (getDistance(Front) <= _minDist) {
            _state = Search_MoveForward;
            return;
          }
        }
        //}
        //TODO Serial.println("Consider to use compass instead of encoders for direction while scanning...");
        int16_t leftBorder = encoders.getCountsLeft(); //encoderOverflows * 65535 + encodersStart;
        //uint16_t overflowsToStartinPoint = encoderOverflows;
        //Serial.print("LeftBorder:"); Serial.println(leftBorder);

        stop();
        delay(1000);

        //Move back to starting point
        bool keepMoving = true;
        drive(2, -2);
        while (keepMoving) {
          if (encoders.getCountsLeft() <= 0)
            keepMoving = false;
        }

        //Serial.println(F("Starting point reached"));
        //stop();
        //delay(2000);
        //Now do it to the right side
        //   uint32_t currentRightIR = getIRRight(false);
        currentFrontIR = getIRFront(false);

        timeOfLastGoodValue = millis();
        goodValue = true;
        drive(2, -2);
        while ((millis() < timeOfLastGoodValue  + IR_TIME_BUFFER) ||
               goodValue)

          /*
            while (irMax <= currentFrontIR + IR_SCAN_TRESHOLD && 0 < currentFrontIR ||
                 currentFrontIR <= getIRRight(false)) */
        {
          //Serial.print("Last good IR value: " ); Serial.println(timeOfLastGoodValue);
          if (irMax < currentFrontIR)
            irMax = currentFrontIR;

          //     currentRightIR = getIRRight(false);
          currentFrontIR = getIRFront(false);
          goodValue = irMax <= currentFrontIR + IR_SCAN_TRESHOLD &&
                      (0 < currentFrontIR);// || currentFrontIR <= getIRRight(false));
          if (goodValue)
            timeOfLastGoodValue = millis();
          if (getDistance(Front) <= _minDist) {
            _state = Search_MoveForward;
            return;
          }
        }
        stop();
        delay(1000);
        int16_t rightBorder = encoders.getCountsLeft();
        // Serial.print(F("Right border: ")); Serial.println(rightBorder);
        int16_t center = rightBorder + (leftBorder + (-rightBorder)) / 2;

        //encoderLast = encoders.getCountsLeft();
        bool passed = false;
        drive(-2, 2);
        while (!passed) {
          if (center <= encoders.getCountsLeft())
            passed = true;
          //passed = compass.heading() < dir;
        }

        _state = Search_MoveForward;
        break;
      }

    case Search_Alignment: {
        //        Serial.println(F("2: Alignment"));
        //        Serial.print(F("Other heading: "));
        //        Serial.println(_otherHeading);

        beep();
        delay(500);
        /*        Teensy::Request req;
                req.type = Teensy::TEENSY_REQUEST_HEADING;
                teensy.sendRequest(req);*/

        //Wait for the message be send and executed on the other robot
        //delay(500);

        //TODO
        // Serial.println(F("Alignment starter - consider to use FaceOpponent (IR) instead"));
        //Turn left, while scanning
        unsigned long SCAN_TIME = 1500;
        scanAndGoToHeadingOfClosestObject(SCAN_TIME);

        //       Serial.println("Pointing at other robot!!");
        delay(1000);
        //Should be pointing towards the nearest point of the other robot
        double fDist = getDistance(Front) * 10;
        if (fDist < ALIGNMENT_SAFETY_DISTANCE) {
          drive(-1, -1);
          while (fDist < ALIGNMENT_SAFETY_DISTANCE)
            fDist = getDistance(Front) * 10;
        } else {
          drive(1, 1);
          while (fDist > ALIGNMENT_SAFETY_DISTANCE)
            fDist = getDistance(Front) * 10;
        }
        stop();




        /**********************
           Part II
         **********************/
        //Allow complete halt to prevent noisy readings
        beep();
        delay(500);

        Teensy::Request req;
        req.type = Teensy::TEENSY_REQUEST_HEADING;
        teensy.sendRequest(req);
        while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING))
          teensy.checkForIncomingMsg(); //Just idle
        Teensy::Response res = teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING);

        double heading = res.valueDouble;
        //        Serial.print(F("Heading is ")); Serial.println(heading);
        //        Serial.print(F("_otherHeading is ")); Serial.println(_otherHeading);

        double delta = _otherHeading - heading;
        //        Serial.print(F("Delta: ")); Serial.println(delta);

        stop();
        delay(500);
        //   delay(10000);
        DistanceDirection sensor;
        double endHeading;
        if (delta < 0) {
          endHeading = _otherHeading + 90;
          sensor = Left;
          turnToHeading(heading + 90, false);
        } else {
          endHeading = _otherHeading - 90;
          sensor = Right;
          turnToHeading(heading - 90, true);
        }
        //Serial.print("Endheading: ");
        //Serial.println(endHeading);
        // delay(10000);
        //Start distance algorithm
        turnToHeadingWithDistance(sensor, endHeading, ALIGNMENT_SAFETY_DISTANCE);

        beep();
        // delay(10000);

        //point towards other robot
        //        Serial.println("Will point towards other robot...");
        turnToHeading(_otherHeading, sensor == Left ? true : false);

        //Scan for direction of other robot
        scanAndGoToHeadingOfClosestObject(SCAN_TIME);// no

        delay(500);

        //Drive very close
        fDist = getDistance(Front);
        drive(1, 1);
        while (fDist > 7)
          fDist = getDistance(Front);
        stop();
        delay(1000);

        //turn 90 degrees
        turnToHeading(_otherHeading - 90, true);
        delay(1000);

        //drive back and forth to measure sides of other robot
        double rDist = getDistance(Right);
        uint8_t extraForEdge = 5; //5 cm extra is considered to be the edge...
        double rDistMax = rDist + extraForEdge;
        drive(1, 1);
        while (rDist < rDistMax)
          rDist = getDistance(Right);
        //Reached end...
        stop();

        encoders.getCountsAndResetLeft();
        drive(-1, -1);
        delay(1000);
        rDist = getDistance(Right);
        while (rDist < rDistMax)
          rDist = getDistance(Right);
        //Reached other end...
        stop();

        long encoder = encoders.getCountsLeft();
        long center = encoder / 2;

        // Serial.print("center is: ");
        // Serial.println(center);

        //position robot in the middle
        drive(1, 1);
        if (center < 0)
          while (encoders.getCountsLeft() < center);
        else
          while (encoders.getCountsLeft() > center);
        stop();

        delay(1000);

        //turn away from other robot
        turnToHeading(_otherHeading + 180, true);

        //drive 250 mmm
        driveMm(250, 2);

        //        Serial.println(F("Zumo should be aligned now... Response has been sent to Teensy!!!"));
        Teensy::Response resDone;
        resDone.type = Teensy::TEENSY_RESPONSE_ALIGNMENT_FINISHED;
        teensy.sendResponse(resDone);
        _state = Search_Stop;

        break;
      }









    /*******************************************************
     ***************  DOCKING    ***************************
     *******************************************************/

    case Search_Docking: {



        Serial.println(F("3: Docking"));
        Serial.print(F("_otherHeading="));
        Serial.println(_otherHeading);

        /*******************
           Strafing start
         *******************/
        while (_dockingDoStrafe) {
          // Serial.println(F("Entered strafing.."));
          turnToHeading(_otherHeading, _dockingTurningLeft);
          //delay(500);

          bool foundLaserLeft = false;
          bool foundLaserRight = false;
          const uint16_t searchTime = 2000;

          drive(-1, 1);
          unsigned long end = millis() + searchTime;
          //       Serial.println(F("Scanning left..."));

          //Empty queue
          //while (teensy.checkForIncomingMsg());
          teensy.getNextResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED);

          while (millis() < end) {
            teensy.checkForIncomingMsg();
            if (teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED)) { // && (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {

              //              Serial.println(F("Laser detected during LEFT scanning!"));

              stop();
              foundLaserLeft = true;
              _dockingTurningLeft = false;
              delay(500);
              break;
            }
          }
          stop();

          //If unsuccessful, reset heading and then try to the right side
          if (!foundLaserLeft) {

            //            Serial.println(F("Laser not found - Scanning right..."));

            //turnToHeading(_otherHeading, false);

            drive(1, -1);

            //Empty queue
            //while (teensy.checkForIncomingMsg());
            teensy.getNextResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED);


            //Twice the search time - no fancy odometry or heading stuff here
            unsigned long end = millis() + 2 * searchTime;


            while (millis() < end) {
              teensy.checkForIncomingMsg();
              if (teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED)) { // && (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {

                //                Serial.println(F("Laser detected during RIGHT scanning!"));

                stop(); //Is this needed?
                foundLaserRight = true;
                _dockingTurningLeft = true;
                delay(500);
                break;
              }
            }
            stop();
          }


          if (!(foundLaserLeft || foundLaserRight)) {
            //            Serial.println(F("LASER NOT DETECTED DURING INITIAL DOCKING PROCEDURE!!"));
            beep();
            delay(200);
            beep();
            delay(200);
            beep();
            //delay(10000);

            //            Serial.println(F("Trying again..."));

            return;//break;
            //continue;
          }

          //Anticipate that the robot is pointing directly to towards the other robot


          Teensy::Request reqHeading;
          reqHeading.type = Teensy::TEENSY_REQUEST_HEADING;
          unsigned long nextRequestTime = 0;
          do {

            teensy.checkForIncomingMsg();
            if (nextRequestTime < millis()) {
              teensy.sendRequest(reqHeading);
              nextRequestTime = millis() + 250;
            }

          } while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING));
          //  while (!teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_HEADING)) { // ||  teensy.getNextResponse().type != Teensy::TEENSY_RESPONSE_HEADING) {
          //Serial.println(F("Waiting for heading..."));

          //}
          //double dist = getDistanceFront(3);
          double heading = teensy.getNextResponse(Teensy::TEENSY_RESPONSE_HEADING).valueDouble;
          //#if DEBUG
          /*      Serial.print(F("Heading is: "));
                Serial.println(heading);
                Serial.print(F("_otherHeading is: "));
                Serial.println(_otherHeading);
          */     //#endif

          //double delta = _otherHeading - heading;
          double delta = _otherHeading - heading;
          if (delta > 180)
            delta -= 360;
          if (delta < -180)
            delta += 360;

          double maxDeltaBeforeStrafing = 5.0; //Degrees
          //Serial.print(F("Delta is: "));
          //Serial.println(delta);

          if (abs(delta) < maxDeltaBeforeStrafing) {
            //   Serial.println(F("Ending strafing"));
            _dockingDoStrafe = false;
          } else {
            if (delta < 0) {
              //Strafe a bit to the right

              //              Serial.println(F("Strafing a bit to the right"));

              //turnToHeading(_otherHeading + 90, false);
              turnToHeading(_otherHeading - 90, true);
              driveMm(STRAFE_MM, 1);
              //turnToHeading(_otherHeading, true);
              turnToHeading(_otherHeading, false);
            } else {
              //Strafe a bit to the left

              //              Serial.println(F("Strafing a bit to the left"));

              //turnToHeading(_otherHeading - 90, true);
              turnToHeading(_otherHeading + 90, false);
              driveMm(STRAFE_MM, 1);
              //turnToHeading(_otherHeading, false);
              turnToHeading(_otherHeading, true);
            }
          }
        }//Strafing end






        /**********************************************************************

           The need for strafing is over. Proceed to the final part (the snake thingy)

         **********************************************************************/

        //Final steps: Turn around to be back on back with the other robot
        //double fDist = getDistance(Front, 1);
        //#if DEBUG
        //Serial.print(F("Front distance: ")); Serial.println(fDist);
        //#endif
        //        Serial.print("_lastLaserDetectionTime: ");
        //        Serial.println(_lastLaserDetectionTime);
        if (_lastLaserDetectionTime + DOCKING_LASER_NOT_DETECTED_TIME < millis()  ) { //fDist <= 4.0) {

          //Make sure by scanning to left and right
          stop();

          //Clear
          teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED);

          unsigned long scanBuffer = 1500;
          unsigned long endTime = millis() + scanBuffer;
          bool foundLaser = false;
          bool foundLaserLeft = false;
          drive(-1, 1);
          while (millis() < endTime && !foundLaser) {
            teensy.checkForIncomingMsg();
            if (teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {
              foundLaser = true;
              foundLaserLeft = true;
            }
          }
          stop();

          if (!foundLaser) {
            endTime = millis() + scanBuffer * 2;
            drive(1, -1);

            while (millis() < endTime && !foundLaser) {
              teensy.checkForIncomingMsg();
              if (teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {
                foundLaser = true;
                foundLaserLeft = false;
              }
            }
            stop();
          }

          if (!foundLaser) {
            //Make sure that this isn't a bad measuring
            //fDist = getDistance(Front, 5);
            //if (fDist > 4.0)
            //break;
            //#if DEBUG
            //Serial.println(F("In front of robot - we are done!"));
            //#endif

            //Make it "wiggle" to all the way in...
            for (int i = 0; i < 5; i++) {
              motors.setSpeeds(-250, 0); //drive(-10, 0);
              delay(500);
              motors.setSpeeds(0, -250); //drive(0, -10);
              delay(500);
            }

            stop();
            beep();
            delay(100);
            beep();

            //turnToHeading(_otherHeading + 180, true);
            //driveMm(-90, -1);
            //        driveMm(-80, -1);

            Teensy::Response res;
            res.type = Teensy::TEENSY_RESPONSE_DOCKING_FINISHED;
            teensy.sendResponse(res);

            /***************
               Resetting
             ***************/
            _dockingLaserIsDetected = false;
            _state = Idle;
            return;
          }



          //Make sure that the laser gets on the right side of the receiver
          if (foundLaserLeft) {
            drive(1, 0);
            _dockingTurningLeft = false;
          } else {
            drive(0, 1);
            _dockingTurningLeft = true;
          }
        }



        teensy.checkForIncomingMsg();
        //If detecting laser - do nothing
        //        Serial.println(F("What if Robot is landing on laser?!"));
        if (teensy.hasNewResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED)) { // && (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {
          teensy.getNextResponse(Teensy::TEENSY_RESPONSE_LASER_DETECTED);
          Serial.println(F("Laser is detected!!"));
          _lastLaserDetectionTime = millis();

          if (!_dockingDoStrafe && !_dockingLaserIsDetected && ((millis() - _dockingLastLaserDetection) > 100)) {
            _dockingTurningLeft = !_dockingTurningLeft;
            _dockingLaserIsDetected = true;
          }
          _dockingLastLaserDetection = millis();
          break;;
        }
        _dockingLaserIsDetected = false;




        // Serial.print(F("Laser not detected => turning"));
        //Not detecting laser - act on this!


        //Commented this out!!!
        if (_dockingTurningLeft) { // || foundLaserLeft) {
          // Serial.println(F(" right"));
          //drive(7, 1);
          drive(0, -1);
          delay(250);
        } else {//if (!_dockingTurningLeft || foundLaserRight) {
          //Serial.println(F(" left"));
          //drive(1, 7);
          drive(-1, 0);
          delay(250);
        }

        break;
      }

    case Search_Stop: {
        beep();
        //Teensy::Response res;
        //res.type = Teensy::TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED;
        //teensy.sendResponse(res);
        _state = Stop;
        break;
      }
  }
}

//unsigned long start = millis();
void loop(void) {
  //if (start + 60000 < millis())
  //  turnToHeading(0.0, false);

  //usDistanceFront();
  //  heartbeat();
  teensy.checkForIncomingMsg();
  if (teensy.hasNewRequest())
    handleIncomingRequest_Teensy();

  if (_emitIR)
    emitIrSignal();

  //Serial.println(getIRFront(false));

  handleCurrentState();

  //  Serial.println(getDistance(Front));
}

