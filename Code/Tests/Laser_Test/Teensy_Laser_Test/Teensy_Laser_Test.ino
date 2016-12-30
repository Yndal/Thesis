//#include <TimerOne.h>
#include <EEPROM.h>
//#include "Common.h"
#include "BT.h"
#include "Zumo.h"


bool setAsController = false;


/*
   Stupid Windows hack...
*/
//#include <Encoder.h>
//#include <TimerThree.h> //TODO Should be used for Zumo heart beat!!


BT bt;
Zumo zumo;

/*************************************
   Serial related
 *************************************/
//#define ZUMO_SERIAL Serial1
//#define ZUMO_BAUT 9600
#define MAX_INCOMING_ZUMO_BUFFER 32
#define INCOMING_ZUMO_END_CHAR ';'


enum State {
  Idle,
  CoarseLocalizataion,
  Alignment,
  Docking,
  Laser_Detection,
};
State _state = Idle;
State _lastState = _state;

/*char incomingZumo[MAX_INCOMING_ZUMO_BUFFER];
  int incomingZumoCounter = 0;
  bool incoming_msg_zumo = false;*/

/*************************************
   Heartbeat related
 *************************************/
//#define ZUMO_HEARTBEAT_CHAR '*'
//#define ZUMO_HEARTBEAT_INTERVAL_MS 500
//#define HEARTBEAT_EXTRA_TIME_MS 200

//IntervalTimer heartbeatTimer;
//volatile bool zumoHeartbeat = true;
//unsigned long lastZumoHeartbeatTime = 0;

/*************************************
   Aligment related
 *************************************/
double _otherHeading = 0.0;





/*************************************
   Docking related
 *************************************/
bool _notifyOnLaserDetected = false;
uint16_t laserDetectionThreshold = 0;
#define LASER_THRESHOLD     100 //100 more than the measured value
#define LASER_DETECTION_PIN  A8
#define LASER_PIN            23

/*************************************
   Misc
 *************************************/
//#define START_DELAY_MS 2000 //Give external device to start and transmit heartbeats before lunching the Teensy

#define EEPROM_ADDRESS 0
uint8_t robotNumber = 0;

unsigned long lastLaserDetection = 0;

const uint8_t led = 13;  // the pin with a LED
bool ledState = LOW;




void setup()
{
  pinMode(led, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(led, HIGH);
  digitalWrite(LASER_PIN, LOW);

  //heartbeatTimer.begin(checkForHeartbeat, (ZUMO_HEARTBEAT_INTERVAL_MS + HEARTBEAT_EXTRA_TIME_MS)*1000); //Time in micro (not milli) seconds
  Serial.begin(9600);
  delay(2000);


  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  Serial.print(F("Robot number: ")); Serial.println(robotNumber);

  if (robotNumber == 1)
    setAsController = false;
  else if (robotNumber == 2)
    setAsController = true;

  bt.initialize(robotNumber);
  zumo.initialize();

  bt.setAsSlave();//Master("884AEA3B9618");
  Zumo::Request req;

  //Stop moving
  req.type = Zumo::REQUEST_STOP;
  zumo.sendRequest(req);
  delay(200);

  if (setAsController)
    initLaserController();
  else
    initLaserDetection();

  //Notify user, that Zumo is ready
  req.type = Zumo::REQUEST_BEEP;
  zumo.sendRequest(req);
}



void getAndHandleNextBTRequest() {
  BT::Request btReq = bt.getNextRequest();
  //Serial.print("handling msg type: "); Serial.println(req.type);
  switch (btReq.type) {
    case BT::BT_REQUEST_IR_ON: {
        Zumo::Request req;
        req.type = Zumo::REQUEST_IR_ON;
        zumo.sendRequest(req);
        break;
      }
    case BT::BT_REQUEST_IR_OFF: {
        Zumo::Request req;
        req.type = Zumo::REQUEST_IR_OFF;
        zumo.sendRequest(req);
        break;
      }
    case BT::BT_REQUEST_NOTIFY_ON_LASER_DETECTION: {
        Serial.println(F("Supposing that state is idle and ready for laser detection..."));
        _state = Laser_Detection;
        break;
      }
    case BT::BT_REQUEST_CANCEL_LASER_DETECTION: {
        Serial.println(F("Stopping laser detection"));
        if (_state == Laser_Detection)
          _state = Idle;
        break;
      }
    case BT::BT_REQUEST_LASER_DETECTED: {
        Serial.println("BT request: Laser detected");
        if (_notifyOnLaserDetected) {
          Zumo::Response res;
          res.type = Zumo::RESPONSE_LASER_DETECTED;
          zumo.sendResponse(res);
          ledState = !ledState;
          digitalWrite(led, ledState);
        }
        break;
      }
    case BT::BT_REQUEST_DRIVE: {
        //Serial.println(F("Asking Zumo to drive"));
        Zumo::Request req;
        req.type = Zumo::REQUEST_DRIVE;
        req.value1 = btReq.value1;
        req.value2 = btReq.value2;
        zumo.sendRequest(req);
        break;
      }
    case BT::BT_REQUEST_STOP: {
        //Serial.println(F("Asking Zumo to stop!"));
        Zumo::Request req;
        req.type = Zumo::REQUEST_STOP;
        zumo.sendRequest(req);
        break;
      }
    case BT::BT_REQUEST_BEEP: {
        //Serial.println(F("Requesting Beep from Zumo"));
        Zumo::Request req;
        req.type = Zumo::REQUEST_BEEP;
        zumo.sendRequest(req);
        break;
      }

    case BT::BT_REQUEST_MODULE_EXCHANGE: {
        Serial.println("BT has requested module exchange...");
        Serial.println("This is currently not implemented!");
        break;
      }

    case BT::BT_REQUEST_COARSE_LOCALIZATION: {
        Zumo::Request req;
        req.type = Zumo::REQUEST_COARSE_LOCALIZATION;
        req.value1 = 3; //Min distance
        bt.setAsMaster("884AEA3B9618");
        zumo.sendRequest(req);
        _state = CoarseLocalizataion;
        break;
      }

    case BT::BT_REQUEST_ALIGNMENT: {
        /*double heading = 0;//TODO btReq.valueDouble;
          Zumo::Request req;
          req.type = Zumo::REQUEST_ALIGNMENT;
          req.valueDouble = heading;
          zumo.sendRequest(req);*/

        _otherHeading = 0.0; //TODO
        _state = Alignment;
        break;
      }

    case BT::BT_REQUEST_DOCKING: {

        _state = Docking;
        break;
      }

    case BT::BT_REQUEST_HEADING: {
        Zumo::Request req;
        req.type = Zumo::REQUEST_HEADING;
        zumo.sendRequest(req);

        //Wait for response
        while (!zumo.hasResponse());

        Zumo::Response zumoRes = zumo.getNextResponse();
        if (zumoRes.type == Zumo::RESPONSE_HEADING) {
          double heading = zumoRes.valueDouble;

          BT::Response btRes;
          btRes.type = BT::BT_RESPONSE_HEADING;
          btRes.valueDouble = heading;
          bt.sendResponse(btRes);

        } else {
          Serial.print(F("Got unknown response: "));
          Serial.println(zumoRes.type);
        }

        break;
      }

    case BT::BT_REQUEST_REPAIR: {
        _state = CoarseLocalizataion;
        break;
      }
  }
}

void getAndHandleNextZumoMsg() {
  Zumo::Request request = zumo.getNextRequest();
  Serial.println(F("Got request from Zumo: "));
  switch (request.type) {
    default: {
        Serial.print(F("Unhandled request from Zumo: "));
        Serial.println(request.type);
        break;
      }
  }
}


void initLaserController() {
  Serial.println(F("Reached Docking state!! :-)"));

  Serial.println(F("Remove the next line before full implementation!"));
  bt.setAsMaster("884AEA6080DA");  //TODO Robot 1's mac
  delay(1000);

  Serial.println("Connected via BT");
  lastLaserDetection = 0;


  //Request laser detection from other robot
  BT::Request reqBt;
  reqBt.type = BT::BT_REQUEST_NOTIFY_ON_LASER_DETECTION;
  bt.sendRequest(reqBt);
  _notifyOnLaserDetected = true;
  delay(1000);
  Serial.println("Requested laser notification");

  Zumo::Request reqZumo;
  reqZumo.type = Zumo::REQUEST_DOCKING;
  zumo.sendRequest(reqZumo);
  Serial.println("Requested Zumo to dock");

  digitalWrite(LASER_PIN, HIGH);

}


void initLaserDetection() {
  //Measure current light
  unsigned long end = millis() + 1000; //Measure for 1 second
  unsigned long value = 0;
  unsigned long counter = 0;
  while (millis() < end) {
    value += analogRead(LASER_DETECTION_PIN);
    counter++;
  }
  laserDetectionThreshold = (value / counter) + LASER_THRESHOLD;

}

void loopLaserDetection() {
  int LASER_DETECTION_THRESHOLD_MS = 100;
  int value = analogRead(LASER_DETECTION_PIN);
  Serial.print(F("Laser value: "));
  Serial.print(value);
  Serial.print("("); Serial.print(laserDetectionThreshold); Serial.println(")");
  if (laserDetectionThreshold <= value) {
    ledState = !ledState;
    digitalWrite(led, ledState);
    //Avoid spamming the Zumo: Only send if the dectection is "new"
    // if (LASER_DETECTION_THRESHOLD_MS <= millis() - lastLaserDetection) {
    BT::Request req;
    req.type = BT::BT_REQUEST_LASER_DETECTED;
    bt.sendRequest(req);

    /* Zumo::Request zReq;
      zReq.type = Zumo::REQUEST_BEEP;
      zumo.sendRequest(zReq);*/
    // }

    //lastLaserDetection = millis();
  }

}


void loopLaserController() {
  //Serial.println("State: Docking");
  //This is done on the Zumo (among other reasons, to save space on the Teensy)
  if (zumo.hasResponse()) {
    Zumo::Response res = zumo.getNextResponse();
    switch (res.type) {
      case Zumo::RESPONSE_DOCKING_FINISHED: {
          Serial.println("Docking finished!!");
          digitalWrite(LASER_PIN, LOW);
          bt.setAsSlave();
          _state = Idle;
          break;
        }
      default: {
          Serial.print(F("Docking - Lost response msg: "));
          Serial.println(res.type);
          break;
        }
    }
  }
}


// The main program will print the blink count
// to the Arduino Serial Monitor
void loop()
{
  //Repond to bluetooth communication
  bt.checkForIncomingMsg();
  if (bt.hasRequest()) {
    //ledState = !ledState;
    //digitalWrite(led, ledState);
    getAndHandleNextBTRequest();
  }

  //Respond to comminucation from the Zumo
  zumo.checkForIncoming();
  if (zumo.hasRequest())
    getAndHandleNextZumoMsg();

  if (setAsController)
    loopLaserController();
  else
    loopLaserDetection();
}



