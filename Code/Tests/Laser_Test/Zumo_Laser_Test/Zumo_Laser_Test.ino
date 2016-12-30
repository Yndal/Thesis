#include <EEPROM.h>

//#include <TimerThree.h>

#include <FastGPIO.h>
//#include <L3G.h>
#include <Wire.h>
//#include <LSM303.h>
//  #include <PololuHD44780.h>
//#include <USBPause.h>
#include <PololuBuzzer.h>
#include <Zumo32U4.h>
//#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4ProximitySensors.h>
#include "Teensy.h"

/**********************************
   Helping functions
 **********************************/
#define DegToRad(x) (x*(PI/180))


//#define HEARTBEAT_CHAR        '*'
//#define HEARTBEAT_INTERVAL_MS 500

#define MAX_SPEED 150 //Max is 400
#define MIN_SPEED 70
#define DELTA_SPEED (MAX_SPEED - MIN_SPEED)
#define SCALE_SPEED 10

/*************************
   IR search related
 *************************/
#define DISTANCE_FRONT_THRESHOLD 3 //Readings

/***********************************
   TBD
 ***********************************/
#define EEPROM_ADDRESS 0
unsigned int robotNumber = 0;

#define usTrigPin 13
#define usEchoPin 17



/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Teensy teensy;


/*****************************
   States
 *****************************/
enum State {
  Search_MoveForward,
  Search_FindDirection,
  Search_Alignment,
  Search_Docking,
  Search_Stop,
  Stop,
  Idle
} _state = Idle;


/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/
#define MIN_DISTANCE_FRONT 8
#define IR_SCAN_TRESHOLD 1
uint16_t lastIRLevel = 0;
uint8_t _minDist = 0;
double _otherHeading = 0;
#define ROBOT_WIDTH_MM 98
#define ROBOT_LENGTH_MM 86

bool _dockingDoStrafe = true;
bool _dockingTurningLeft = true;
bool _dockingLaserIsDetected = false;
unsigned long _dockingLastLaserDetection = 0;
/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/

void setup(void) {
  Serial.begin(9600);
  teensy.initialize();
  delay(2000);

  pinMode(usTrigPin, OUTPUT);
  pinMode(usEchoPin, INPUT);

  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  Serial.print(F("Zumo number: ")); Serial.println(robotNumber);


  Serial.println(F("Zumo stands ready for laser testing!"));
}

void handleIncomingRequest_Teensy() {
  Teensy::Request req = teensy.getNextRequest();
  switch (req.type) {
    case Teensy::TEENSY_REQUEST_BEEP:
      beep();
      break;
    case Teensy::TEENSY_REQUEST_DRIVE:
      drive(req.value1, req.value2);
      break;
    case Teensy::TEENSY_REQUEST_TURN_LEFT: {
        drive(-1, 1);
        break;
      }
    case Teensy::TEENSY_REQUEST_STOP:
      stop();
      break;


    case Teensy::TEENSY_REQUEST_DOCKING: {
        _state = Search_Docking;
        _dockingLastLaserDetection = 0;
        _dockingDoStrafe = true;
        _dockingLaserIsDetected = false;
        break;
      }
  }
}




void beep() {
  //Serial.println(F("Beep!"));
  // Start playing a tone with frequency 440 Hz at maximum
  // volume (15) for 200 milliseconds.
  buzzer.playFrequency(440, 200, 15);

  // Delay to give the tone time to finish.
  delay(300);
}


/*
   Can drive 2.2m before the encoders will overflow
*/
void driveMm(double mm, int speed) {
  //Serial.print("Driving mm: "); Serial.println(mm);
  int encoderCPR = 12;
  double gearRatio = 150.58;
  double wheelCircumferenceInMm = 39 * PI;
  float tickCountForOneMm =  (encoderCPR * gearRatio) / wheelCircumferenceInMm * -1;
  float totalTicks = mm * tickCountForOneMm;

  //Serial.print("Driving ticks: "); Serial.println(totalTicks);
  //TODO Make sure that both wheels drive the exact same distance
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();

  //If speed and distance is going in each direction...
  if (mm * speed < 0)
    speed *= -1;
  drive(speed, speed);
  bool passed = false;
  Serial.println(F("How about using getDistanceFront() here?"));
  if (0 < mm) {
    while (!passed) {
      passed = encoders.getCountsRight() < totalTicks;
    }
  } else {
    while (!passed) {
      passed = totalTicks < encoders.getCountsRight();
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

double getDistanceFront(int times) {
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
    distance += (duration / 2) / 29.1;
    delay(10);
  }
  distance /= times;

  return distance;
}

double getDistanceFront() {
  return getDistanceFront(5);
}


void handleCurrentState() {
  switch (_state) {
    case Stop: {
        Serial.println(F("Stop"));
        stop();
        _state = Idle;
        break;
      }
    case Idle: {
        //Serial.println("Idle");
        //Do nothing...
        break;
      }

    case Search_Docking: {
        //Serial.println(F("3: Docking"));
        //beep();
        //Teensy has already told other robot to notify upon laser detection

        /*******************
           Strafing start
         *******************/
        /* while (_dockingDoStrafe) {
           turnToHeading(_otherHeading, true);

           const uint16_t searchTime = 2000;
           bool foundLaserLeft = false;
           bool foundLaserRight = false;
           drive(-1, 1);
           unsigned long end = millis() + searchTime;
           Serial.println(F("Scanning left..."));
           while (millis() < end) {
             teensy.checkForIncomingMsg();
             if (teensy.hasNewResponse()) {
               if (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED) {
                 Serial.println("Laser detected during LEFT scanning!");
                 //NB Teensy is set up to not spam the Zumo with these messages
                 stop(); //Is this needed?
                 foundLaserLeft = true;
                 delay(10000);
                 break;
               }
             }
           }
           stop();

           //If unsuccessful, reset heading and then try to the right side
           if (!foundLaserLeft) {
             Serial.println(F("Laser not found - Scanning right..."));
             //turnToHeading(_otherHeading, false);

             //foundLaser = false;
             drive(1, -1);
             unsigned long end = millis() + 2 * searchTime;
             while (millis() < end) {
               teensy.checkForIncomingMsg();
               if (teensy.hasNewResponse()) {
                 if (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED) {
                   //NB Teensy is set up to not spam the Zumo with these messages
                   Serial.println(F("Laser detected during RIGHT scanning!"));
                   stop(); //Is this needed?
                   foundLaserRight = true;
                   delay(10000);
                   break;
                 }
               }
             }
             stop();
           }


           if (!foundLaserLeft && !foundLaserRight) {
             Serial.println("LASER NOT DETECTED DURING INITIAL DOCKING PROCEDURE!!");
             beep();
             delay(200);
             beep();
             delay(200);
             beep();
             delay(10000);
             break;
           }

           //Anticipate that the robot is pointing directly to towards the other robot
           Teensy::Request req;
           req.type = Teensy::TEENSY_REQUEST_HEADING;
           teensy.sendRequest(req);
           while (!teensy.hasNewResponse())
             teensy.checkForIncomingMsg();
           //double dist = getDistanceFront(3);
           double heading = teensy.getNextResponse().valueDouble; //TODO This might be an old value of a different type...

           double delta = _otherHeading - heading;
           double maxDeltaBeforeStrafing = 5.0; //Degrees

           if (abs(delta) < maxDeltaBeforeStrafing) {
             Serial.println(F("Ending strafing"));
             _dockingDoStrafe = false;
           } else {
             uint8_t strafeMm = 20; //Move 2 cm

             if (delta < 0) {
               //Strafe a bit to the right
               Serial.println(F("Strafing a bit to the right"));
               turnToHeading(_otherHeading + 90, false);
               driveMm(strafeMm, 1);
               turnToHeading(_otherHeading, true);
             } else {
               //Strafe a bit to the left
               Serial.println(F("Strafing a bit to the left"));
               turnToHeading(_otherHeading - 90, true);
               driveMm(strafeMm, 1);
               turnToHeading(_otherHeading, false);
             }
           }
          }//Strafing end
        */
        /**********************************************************************

           The need for strafing is over. Proceed to the final part (the snake thingy)

         **********************************************************************/

        //Final steps: Turn around to be back on back with the other robot
        double fDist = getDistanceFront(1);
        Serial.print(F("Front distance: ")); Serial.println(fDist);
        if (fDist <= 6.0) {
          //Make sure that this isn't a bad measuring
          fDist = getDistanceFront(5);
          if (fDist > 6.0)
            break;
          Serial.println(F("In front of robot - we are done!"));
          stop();
          beep();
          delay(100);
          beep();

          Teensy::Response res;
          res.type = Teensy::TEENSY_RESPONSE_DOCKING_FINISHED;
          teensy.sendResponse(res);
          Serial.println(F("Docking Finished has been sent to Teensy"));



          /***************
             Resetting
           ***************/
          _dockingLaserIsDetected = false;
          _state = Idle;
          return;
        }

        teensy.checkForIncomingMsg();
        //If detecting laser - do nothing
        if (teensy.hasNewResponse() && (teensy.getNextResponse().type == Teensy::TEENSY_RESPONSE_LASER_DETECTED)) {
          Serial.println(F("Laser is detected!!"));
          // _dockingLastLaserDetection = millis();

          if (!_dockingLaserIsDetected) {
            _dockingTurningLeft = !_dockingTurningLeft;
            _dockingLaserIsDetected = true;
          }

          return;
        }
        _dockingLaserIsDetected = false;


        //Allow for a minor "buffer" before turning
        if (millis() - _dockingLastLaserDetection < 100) {
          Serial.println(F("The laser time buffer..."));
          break;
        }

        Serial.print(F("Laser not detected => turning"));
        //Not detecting laser - act on this!
        if (_dockingTurningLeft) {
          Serial.println(F(" right"));
          //drive(7, 1);
          drive(1, 0);
        } else {
          Serial.println(F(" left"));
          //drive(1, 7);
          drive(0, 1);
        }

        break;
      }

    case Search_Stop: {
        Serial.println(F("Search stopped finished"));
        beep();
        //Teensy::Response res;
        //res.type = Teensy::TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED;
        //teensy.sendResponse(res);
        _state = Stop;
        break;
      }
    default:
      Serial.println("_state defaulted! This is BAD!!");
      //Nothing...
      break;
  }
}

void loop(void) {
  teensy.checkForIncomingMsg();
  if (teensy.hasNewRequest())
    handleIncomingRequest_Teensy();



  handleCurrentState();
}

