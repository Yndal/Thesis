#include <EEPROM.h>
#include "BT.h"
#include "Zumo.h"
#include "Modules.h"
#include "MinIMU9_v5.h"


BT bt;
Zumo zumo;
Modules modules = Modules(); //Calling constructor
MinIMU9_v5 imu;

/*************************************
   Serial related
 *************************************/
//#define ZUMO_SERIAL Serial1
//#define ZUMO_BAUT 9600
#define MAX_INCOMING_ZUMO_BUFFER 32
#define INCOMING_ZUMO_END_CHAR ';'

bool initialized = false;
enum State {
  Initialize,
  Idle,
  CoarseLocalization,
  Alignment,
  Docking,
  Laser_Detection,
  Working,
  EjectingModule,
  InsertingModule,
};
State _state = Idle;// Initialize;//Idle;
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
double _otherHeading = 0.00;





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
#define HOST_MAC_ID "20C38FF67FEB"

#define EEPROM_ADDRESS 0
uint8_t robotNumber = 0;

unsigned long lastLaserDetection = 0;

const uint8_t led = 13;  // the pin with a LED
bool ledState = HIGH;

//#define FREE_ANALOG_PIN_FOR_RANDOM A3
#define WORKING_RANDOM_TIME_CHANGE 1500 //time between working routing will randomly change
unsigned long _working_timeToChangeAction_1 = 0;
unsigned long _working_timeToChangeAction_2 = 0;

bool _working_moduleIsActive_1 = false;
bool _working_moduleIsActive_2 = false;

#define MODULE_EXCHANGE_COUNTER_BUFFER 20
uint8_t _moduleToExchange = 0;
Modules::ModuleType _moduleToExchange_Type;
int exchangeTypeCounter = 0;
String _otherMac;

bool _insertionAfterEjection = false;
bool _otherRobot_returnOtherModule = false;
bool _returnOtherModule = false;

void setup()
{
  pinMode(led, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(led, ledState);
  digitalWrite(LASER_PIN, LOW);

  Serial.begin(9600);
  delay(2000);
  Serial.println(F("Please hold on..."));

  imu.init();

  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  Serial.print(F("Robot number: ")); Serial.println(robotNumber);

  bt.initialize(robotNumber);
  zumo.initialize();

  bt.setAsSlave();//Master("884AEA3B9618");
  Zumo::Request req;

  //Calibrate Acc and Gyro
  req.type = Zumo::REQUEST_STOP;
  zumo.sendRequest(req);
  delay(500);
  imu.calibrateAccGyro();

  //Calibrate Compass
   req.type = Zumo::REQUEST_TURN_LEFT;
    zumo.sendRequest(req);
    imu.calibrateMag(30000); //Calibrate for xxx seconds
    delay(200);
  
  //Stop moving
  req.type = Zumo::REQUEST_STOP;
  zumo.sendRequest(req);
  delay(200);

  double h = imu.getHeading();
  Serial.print(F("Heading: ")); Serial.println(h);


  /*
      //Notify user, that Zumo is ready
      req.type = Zumo::REQUEST_BEEP;
      zumo.sendRequest(req);
      Serial.println("Beep");*/

  /*req.type = Zumo::REQUEST_ALIGNMENT;
    req.valueDouble = 0.0;
    zumo.sendRequest(req);*/
  //    random(analogRead(FREE_ANALOG_PIN_FOR_RANDOM));

    for (int i = 0; i < 10; i++) {
      req.type = Zumo::REQUEST_BEEP;
      zumo.sendRequest(req);
      delay(700);
    }
  
  req.type = Zumo::REQUEST_BEEP;
  zumo.sendRequest(req);
  zumo.sendRequest(req);
  zumo.sendRequest(req);


  _state = Working;

  _otherMac.reserve(32);//Reserve

  ledState = !ledState;
  digitalWrite(led, ledState);
}


/*void checkForHeartbeat() {
  bool zumoHB; //Local copy of the Zumo's heartbeat

  noInterrupts();
  zumoHB = zumoHeartbeat;
  interrupts();

  //if(lastZumoHeartbeatTime + ZUMO_HEARTBEAT_INTERVAL <= millis()){
  if (zumoHB) {
    lastZumoHeartbeatTime = millis();

    noInterrupts();
    zumoHeartbeat = false;
    interrupts();

    digitalWrite(led, ledState);
    ledState = !ledState;
  } else {
    zumoHeartbeatStopped();
  }

  }

  void zumoHeartbeatStopped() {
  //TODO
  }*/




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
        //Serial.println(F("BT request: Laser detected"));
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

    case BT::BT_REQUEST_RECONFIGURATION: {
        int modulePos = btReq.value1;
        String mac = btReq.valueStr;


        Serial.println("Got reconf request:");
        Serial.print("MAC: "); Serial.println(mac);
        Serial.print("Module pos: "); Serial.println(modulePos);
        //delay(10000);

        _otherMac = btReq.valueStr;
        _moduleToExchange = btReq.value1;
        _state = CoarseLocalization;
        _otherRobot_returnOtherModule = btReq.valueBool;

        delay(1000);



        /*     if (robotNumber == 1)
               bt.setAsMaster("884AEA3B9618");  //TODO Robot 2's mac
             else if (robotNumber == 2)
               bt.setAsMaster("884AEA6080DA");  //TODO Robot 1's mac


             //Request other robot to start insertion for module !_moduleToExchange
             delay(1000);
             BT::Request req;
             req.type = BT::BT_REQUEST_MODULE_INSERTION;
             req.value1 = _moduleToExchange == 1 ? 2 : 1; //Set to other module
             bt.sendRequest(req);

             _state = EjectingModule;*/

        break;
      }

    case BT::BT_REQUEST_MODULE_EXCHANGE_TEST: {






        if (robotNumber == 1)
          _otherMac = "884AEA3B9618";  //TODO Robot 2's mac
        else if (robotNumber == 2)
          _otherMac = "884AEA6080DA";  //TODO Robot 1's mac

        _moduleToExchange = 1;
        _otherRobot_returnOtherModule = true;

        bt.setAsMaster(_otherMac);
        delay(200);

        _state = EjectingModule;

        break;


        /*        //Collect all the data
                _moduleToExchange = btReq.value1;

                if (robotNumber == 1)
                  bt.setAsMaster("884AEA3B9618");  //TODO Robot 2's mac
                else if (robotNumber == 2)
                  bt.setAsMaster("884AEA6080DA");  //TODO Robot 1's mac


                //Request other robot to start insertion for module !_moduleToExchange
                delay(1000);
                BT::Request req;
                req.type = BT::BT_REQUEST_MODULE_INSERTION;
                req.value1 = _moduleToExchange == 1 ? 2 : 1; //Set to other module
                bt.sendRequest(req);

                _state = EjectingModule;
                break;
        */
      }

    case BT::BT_REQUEST_MODULE_EXCHANGE: {
        Serial.println("BT has requested module exchange...");

        //Collect all the data
        _moduleToExchange = btReq.value1;
        String mac = btReq.valueStr;
        bt.setAsMaster(mac);

        _state = CoarseLocalization;
        break;
      }

    case BT::BT_REQUEST_MODULE_INSERTION: {
        _moduleToExchange = btReq.value1;
        _returnOtherModule = btReq.valueBool;

        _state = InsertingModule;
        break;
      }

    case BT::BT_REQUEST_COARSE_LOCALIZATION: {
        _state = CoarseLocalization;
        break;
      }

    case BT::BT_REQUEST_ALIGNMENT: {
        _state = Alignment;
        break;
      }

    case BT::BT_REQUEST_DOCKING: {
        _state = Docking;
        break;
      }

    case BT::BT_REQUEST_HEADING: {
        double heading = imu.getHeading();
        BT::Response btRes;
        btRes.type = BT::BT_RESPONSE_HEADING;
        btRes.valueDouble = heading;
        bt.sendResponse(btRes);
        // delay(200);

        break;
      }
  }
}

void getAndHandleNextZumoMsg() {
  Zumo::Request request = zumo.getNextRequest();
  Serial.println(F("Got request from Zumo: "));
  switch (request.type) {
    case Zumo::REQUEST_IR_ON: {
        BT::Request res;
        res.type = BT::BT_REQUEST_IR_ON;
        Serial.println("IR on");
        bt.sendRequest(res);
        break;
      }
    case Zumo::REQUEST_IR_OFF: {
        BT::Request res;
        res.type = BT::BT_REQUEST_IR_OFF;
        Serial.println("IR off");
        bt.sendRequest(res);
        break;
      }
    /*  case Zumo::RESPONSE_COARSE_LOCALIZATION_FINISHED: {
          Serial.println("Coarse location finished!");
          Serial.println("Starting alignment...");
          Zumo::Request req;
          req.type = Zumo::REQUEST_ALIGNMENT;
          zumo.sendRequest(req);
          break;

        }*/
    case Zumo::REQUEST_HEADING: {
        double heading = imu.getHeading();
        Zumo::Response res;
        res.type = Zumo::RESPONSE_HEADING;
        res.valueDouble = heading;
        zumo.sendResponse(res);
        break;
      }
    default: {
        Serial.print(F("Unhandled request from Zumo: "));
        Serial.println(request.type);
        break;
      }
  }
}


void handleCurrentState() {
  /**************************
    If first round with this new state
   **************************/
  if (_lastState != _state) {
    //Get ready to do something else than working...
    if (_lastState == Working)
      stopWorking();

    switch (_state) {
      case Initialize: {
          modules.stopAll();
        }

      case CoarseLocalization: {
          modules.stopAll();

          //Disconnect from host if not already done
          bt.setAsSlave();

          //Make sure that we are connected
          if (!bt.isConnected())
            bt.setAsMaster(_otherMac);

          //Request Zumo to stop
          Zumo::Request zumoReq;
          zumoReq.type = Zumo::REQUEST_STOP;
          zumo.sendRequest(zumoReq);
          delay(200);

          //Back away from current work
          zumoReq.type = Zumo::REQUEST_DRIVE_MM;
          zumoReq.value1 = -300;
          zumoReq.value2 = -2;
          zumo.sendRequest(zumoReq);

          //Establish connection to the broken robot and request to turn IR on
          //bt.setAsMaster("884AEA6080DA");  //TODO Robot 1's mac
          //bt.setAsMaster("884AEA3B9618");  //TODO Robot 2's mac
          delay(1000);
          BT::Request btReq;
          btReq.type = BT::BT_REQUEST_IR_ON;
          bt.sendRequest(btReq);

          //Request Zumo to perform Coarse Localization
          // Zumo::Request zumoReq;
          zumoReq.type = Zumo::REQUEST_COARSE_LOCALIZATION;
          zumoReq.value1 = 10; //Min distance
          zumo.sendRequest(zumoReq);
          break;
        }

      case Alignment: {
          //Make sure that we are connected
          if (!bt.isConnected())
            bt.setAsMaster(_otherMac);

          //Stop emitting IR and request the heading
          //Serial.println(F("Alignment - new state"));
          //bt.setAsMaster("884AEA6080DA"); //TODO Remove, when not testing
          delay(1000);

          BT::Request btReq;
          btReq.type = BT::BT_REQUEST_IR_OFF;
          bt.sendRequest(btReq);
          delay(500);

          //Clear
          bt.getNextResponse();

          //Get heading
          btReq.type = BT::BT_REQUEST_HEADING;
          bt.sendRequest(btReq);


          //Check for msg until a heading-response is received
          if (!bt.hasResponse() ||
              bt.getNextResponse().type != BT::BT_RESPONSE_HEADING)
            bt.checkForIncomingMsg();

          //Get the heading
          BT::Response res = bt.getNextResponse();
          _otherHeading = res.valueDouble;

          Serial.print("Alignment - _otherHeading is: ");
          Serial.println(_otherHeading);


          //Send the heading and a Alignemt requeest to the Zumo
          Zumo::Request zumoReq;
          zumoReq.type = Zumo::REQUEST_ALIGNMENT;
          zumoReq.valueDouble = _otherHeading;
          zumo.sendRequest(zumoReq);
          delay(100);
          //req.type = BT::BT_REQUEST_HEADING;
          //bt.sendRequest(req);
          break;
        }

      case Docking: {
          Serial.println(F("Reached Docking state!! :-)"));

          //Make sure that we are connected
          if (!bt.isConnected())
            bt.setAsMaster(_otherMac);

          //Serial.println(F("Remove the next line before full implementation!"));
          //bt.setAsMaster("884AEA6080DA");  //TODO Robot 1's mac
          delay(1000);

          Serial.println("Connected via BT");
          lastLaserDetection = 0;


          //Request laser detection from other robot
          BT::Request reqBt;
          reqBt.type = BT::BT_REQUEST_IR_OFF;
          bt.sendRequest(reqBt);
          delay(200);

          //bt.getNextResponse();
          reqBt.type = BT::BT_REQUEST_HEADING;
          bt.sendRequest(reqBt);
          delay(200);

          if (!(bt.hasResponse() && bt.getNextResponse().type == BT::BT_RESPONSE_HEADING))
            bt.checkForIncomingMsg();

          BT::Response response = bt.getNextResponse();
          _otherHeading = response.valueDouble;

          reqBt.type = BT::BT_REQUEST_NOTIFY_ON_LASER_DETECTION;
          bt.sendRequest(reqBt);
          _notifyOnLaserDetected = true;
          delay(1000);
          Serial.println("Requested laser notification");

          Zumo::Request reqZumo;
          reqZumo.type = Zumo::REQUEST_DOCKING;
          reqZumo.valueDouble = _otherHeading;

          zumo.sendRequest(reqZumo);
          Serial.print("Requested Zumo to dock with heading:");
          Serial.println(_otherHeading);
          //   delay(5000);
          digitalWrite(LASER_PIN, HIGH);

          break;
        }

      case Laser_Detection: {
          //Measure current light
          unsigned long end = millis() + 1000; //Measure for 1 second
          unsigned long value = 0;
          unsigned long counter = 0;
          while (millis() < end) {
            value += analogRead(LASER_DETECTION_PIN);
            counter++;
          }
          laserDetectionThreshold = (value / counter) + LASER_THRESHOLD;
          Serial.print("Laser threshold: ");
          Serial.println(laserDetectionThreshold);


          break;
        }

      case Idle: {
          /*if (modules.moduleType1 != Modules::Empty &&
              modules.moduleType2 != Modules::Empty) {
            Serial.println("State is Idle, but modules are loaded, so switching to Working...");
            _state = Working;
            }*/
          break;
        }

      case Working: {
          Zumo::Request req;
          req.type = Zumo::REQUEST_WORK;
          zumo.sendRequest(req);
          //Serial.println("New state is Working, but nothing is done here...");

          //start modules - nothing will happen if they are not present
          modules.performModuleAction(1, Modules::Start);
          modules.performModuleAction(2, Modules::Start);

          break;
        }

      case EjectingModule: {
          //Make sure that we are connected
          if (!bt.isConnected())
            bt.setAsMaster(_otherMac);
          delay(1000);

          BT::Request req;
          req.type = BT::BT_REQUEST_MODULE_INSERTION;
          req.value1 = _moduleToExchange == 1 ? 2 : 1; //Set to other module
          req.valueBool = _otherRobot_returnOtherModule;
          bt.sendRequest(req);

          //_otherRobot_returnOtherModule = false;

          //Start ejecting...
          modules.startEjection(_moduleToExchange);//.requestChangeAction(_moduleToExchange, Modules::Eject);

          break;
        }

      case InsertingModule: {
          Serial.println("Inserting module...");
          //Start the insertion on this robot
          // modules.insertModule(_moduleToExchange);//.requestChangeAction(_moduleToExchange, Modules::Insert);

          break;
        }
      default: {
          Serial.print("New state defaulted.... ");
          Serial.println(_state);
          break;
        }
    }

    _lastState = _state;
    return;

  }



  /***********************************
     This state isn't new to us...
   ***********************************/
  switch (_state) {
    case Initialize: {
        if (!initialized) {
          Serial.println("State: Initializing");
          initialized = true;
          bt.reportInToServiceStation(HOST_MAC_ID, robotNumber, bt.getThisMacID(), modules.getModuleType(0), modules.getModuleType(1));
        }
        _state = Idle;
      }
    case CoarseLocalization: {
        if (zumo.hasResponse()) {
          Zumo::Response response = zumo.getNextResponse();

          switch (response.type) {
            case Zumo::RESPONSE_COARSE_LOCALIZATION_FINISHED: {
                BT::Request req;
                req.type = BT::BT_REQUEST_IR_OFF;
                bt.sendRequest(req);
                delay(1000);
                //bt.setAsSlave();

                _state = Alignment;
                break;
              }
            default: {
                Serial.println("Response defaulted during CoarseLocalization.... This shouldn't have happened!");
                break;
              }
          }
        }
        break;
      }

    case Alignment: {
        /*if (bt.hasResponse()) {
          BT::Response res = bt.getNextResponse();
          switch (res.type) {
            case BT::BT_RESPONSE_HEADING: {
                _otherHeading = res.valueDouble;

                //Send the heading and a Alignemt requeest to the Zumo
                Zumo::Request zumoReq;
                zumoReq.type = Zumo::REQUEST_ALIGNMENT;
                zumoReq.valueDouble = _otherHeading;
                zumo.sendRequest(zumoReq);
                delay(100);

                break;
              }

            default:
              //Ignore
              break;
          }
          }*/

        if (zumo.hasResponse()) {
          Zumo::Response res = zumo.getNextResponse();
          switch (res.type) {
            case Zumo::RESPONSE_ALIGNMENT_FINISHED: {
                Serial.println(F("ZUMO HAS ALIGNED!!"));
                //_state = Docking;
                // bt.setAsSlave(); //TODO Remove when not testing
                _state = Docking;
                break;
              }

            default: {
                Serial.print(F("State: Aligment - We missed a Zumo response: "));
                Serial.println(res.type);
                break;
              }
          }
        }
        break;
      }

    case Laser_Detection: {
        //int LASER_DETECTION_THRESHOLD_MS = 100;
        int value = analogRead(LASER_DETECTION_PIN);
        // Serial.print(F("Laser value: "));
        // Serial.print(value);
        // Serial.print("("); Serial.print(laserDetectionThreshold); Serial.println(")");
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

        if (zumo.hasResponse()) {
          Zumo::Response res = zumo.getNextResponse();
          switch (res.type) {
            case Zumo::RESPONSE_DOCKING_FINISHED: {
                Serial.println(F("ZUMO HAS DOCKED!!"));
                //_state = Docking;
                //bt.setAsSlave(); //TODO Remove when not testing
                _state = Idle; //TODO Think this is okay
                break;
              }

            default: {
                Serial.print(F("State: Docking - We missed a Zumo response: "));
                Serial.println(res.type);
                break;
              }
          }
        }
        break;
      }

    case Docking: {
        Serial.println("State: Docking");
        //This is done on the Zumo (among other reasons, to save space on the Teensy)
        if (zumo.hasResponse()) {
          Zumo::Response res = zumo.getNextResponse();
          switch (res.type) {
            case Zumo::RESPONSE_DOCKING_FINISHED: {
                Serial.println("Docking finished!!");
                digitalWrite(LASER_PIN, LOW);
                //bt.setAsSlave();


                _insertionAfterEjection = true;
                _state = EjectingModule;
                break;
              }
            default: {
                Serial.print(F("Docking - Lost response msg: "));
                Serial.println(res.type);
                break;
              }
          }
        }
        break;
      }

    case EjectingModule: {
        //   Serial.println("State: EjectingModule... TODO");
        if (bt.hasResponse()) {
          BT::Response res = bt.getNextResponse();
          switch (res.type) {
            case BT::BT_RESPONSE_MODULE_EXCHANGE_STOPPED: {
                modules.stopEjection(_moduleToExchange);

                if (_otherRobot_returnOtherModule) {

                  _moduleToExchange = _moduleToExchange == 1 ? 2 : 1; //Set to other module
                  _state = InsertingModule;
                  _otherRobot_returnOtherModule = false;
                  break;
                }


                // Serial.println("Module ejected! Changing state to Idle...");
                bt.setAsSlave();


                //Set Zumo to Idle... But then move away
                Zumo::Request req;
                req.type = Zumo::REQUEST_STOP;
                zumo.sendRequest(req);
                req.type = Zumo::REQUEST_DRIVE_MM;
                req.value1 = 500;
                req.value2 = 2;
                zumo.sendRequest(req);

                /*if(_insertionAfterEjection){
                  _insertionAfterEjection = false;
                  _moduleToExchange = _moduleToExchange == 1 ? 2 : 1;
                  _state = InsertingModule;

                  break;
                  }*/
                _state = Idle;
                break;
              }

            default:
              //Ignore...
              break;
          }
        }

        break;
      }

    case InsertingModule: {
        Serial.println("State: InsertingModule... TODO");

        Serial.print("Inserting module: ");
        Serial.println(_moduleToExchange);
        //Blocking call...
        modules.insertModule(_moduleToExchange);

        //Send response to other robot
        BT::Response response;
        response.type = BT::BT_RESPONSE_MODULE_EXCHANGE_STOPPED;
        bt.sendResponse(response);

        if (_returnOtherModule) {
          _moduleToExchange = _moduleToExchange == 1 ? 2 : 1; //Set to other module
          _state = EjectingModule;
          _returnOtherModule = false;
          break;
        }

        bt.setAsSlave();
        //Return to Workign state
        _state = Working;
        Serial.println("Module inserted! Changing state to Working...");

        /*boolean inserted = false;
          if (modules.checkForChanges()) {
          //Check for the type X types to make sure that it's not just a coincidence...
          uint8_t counter = 0;
          bool isSameType = true;
          Modules::ModuleType type = modules.getModuleType(_moduleToExchange);
          for (uint8_t i = 0; i < 10; i++) {
            if (modules.getModuleType(_moduleToExchange) != type)
              break;
            delay(50);
          }
          inserted = true;
        */
        /*Modules::ModuleType type = modules.getModuleType(_moduleToExchange);
          if (type == _moduleToExchange_Type)
          if (exchangeTypeCounter++ > MODULE_EXCHANGE_COUNTER_BUFFER)
            inserted = true;*/

        //      }

        //If insertion completed
        /* if (inserted) {
           modules.stopInsertion(_moduleToExchange);

           //Send response to other robot
           BT::Response response;
           response.type = BT::BT_RESPONSE_MODULE_EXCHANGE_STOPPED;
           bt.sendResponse(response);

           //Return to Workign state
           _state = Working;
           Serial.println("Module inserted! Changing state to Working...");
          }*/
        break;
      }

    case Working: {
        modules.checkForChanges();

        if (!modules.checkModuleStatus()) {
          Serial.println("Modules status is bad...");
          //if(!modules.isModuleWorking()){
          if (modules.getModuleStatus(1) < 0) {
            Serial.println("Module 1 is bad!");
            //Modules::ModuleType type = modules.getModuleType(1);
            requestReconfiguration(1);
          }
          if (modules.getModuleStatus(2) < 0) {
            Serial.println("Module 2 is bad!");
            requestReconfiguration(2);
          }
        }

        //  if (modules.checkForChanges())
        //    Serial.println("Working state: Teensy - A module has changed");

        delay(100);

        //If not time to change yet...
        bool change1 = millis() > _working_timeToChangeAction_1;
        bool change2 = millis() > _working_timeToChangeAction_2;


        // Serial.println("Working state: Changing task....");
        if (change1) {
          if (_working_moduleIsActive_1)
            modules.performModuleAction(1, Modules::Stop);
          else
            modules.performModuleAction(1, Modules::Start);
          _working_moduleIsActive_1 = !_working_moduleIsActive_1;

          _working_timeToChangeAction_1 = millis() + random(WORKING_RANDOM_TIME_CHANGE);
        }

        if (change2) {
          if (_working_moduleIsActive_2)
            modules.performModuleAction(2, Modules::Stop);
          else
            modules.performModuleAction(2, Modules::Start);
          _working_moduleIsActive_2 = !_working_moduleIsActive_2;

          _working_timeToChangeAction_2 = millis() + random(WORKING_RANDOM_TIME_CHANGE);
        }

        //    delay(2000);
        //Note: It is not needed with the entire switch-case, but it encourages for extending this later...
        /*        switch (modules.moduleType1) {
                  case Modules::Battery: {
                      //Can't change this...
                      break;
                    }

                  case Modules::Laser: {
                      if (random(1))
                        modules.performModuleAction(1, Modules::Start);
                      else
                        modules.performModuleAction(1, Modules::Stop);
                      break;
                    }

                  case Modules::Motor: {
                      if (random(1))
                        modules.performModuleAction(1, Modules::Start);
                      else
                        modules.performModuleAction(1, Modules::Stop);
                      break;
                    }

                  case Modules::Empty: {
                      //Can't change this...
                      break;
                    }
                }

                switch (modules.moduleType2) {
                  case Modules::Battery: {
                      //Can't change this...
                      break;
                    }

                  case Modules::Laser: {
                      if (random(1))
                        modules.performModuleAction(2, Modules::Start);
                      else
                        modules.performModuleAction(2, Modules::Stop);
                      break;
                    }

                  case Modules::Motor: {
                      if (random(1))
                        modules.performModuleAction(2, Modules::Start);
                      else
                        modules.performModuleAction(2, Modules::Stop);
                      break;
                    }

                  case Modules::Empty: {
                      //Can't change this...
                      break;
                    }
                }*/



        break;
      }
    case Idle: {
        if (modules.checkForChanges())
          Serial.println("Idle state: Teensy - A module has changed");
        //Do nothing
        break;
      }
    default: {
        if (modules.checkForChanges())
          Serial.println("Default state: Teensy - A module has changed");
        break;
      }
  }
}

void stopWorking() {
  modules.stopAll();
}


void requestReconfiguration(int8_t module) {
  Serial.println("requestReconfiguration()");

  //Verify that something is wrong:
  //Check every 0.5 sec for 5 sec, that something is wrong... if not, then just ignore the request
  for (int i = 0; i < 10; i++) {
    modules.checkForChanges();
    if (modules.checkModuleStatus() ||
        (modules.getModuleStatus(module) > -1))
      return;
    delay(500);
  }


  //Stop and back away from the wall
  Zumo::Request req;
  req.type = Zumo::REQUEST_STOP;
  zumo.sendRequest(req);

  req.type = Zumo::REQUEST_DRIVE_MM;
  req.value1 = -200;
  req.value2 = -2;
  zumo.sendRequest(req);

  //Request reconfiguration
  //Serial.println("Requesting reconfiguration from service station!");
  Modules::ModuleType type = modules.getModuleType(module);
  bt.requestReconfiguration(HOST_MAC_ID, robotNumber, type, module);

  _state = Idle;
}

// The main program will print the blink count
// to the Arduino Serial Monitor
unsigned long imuTimer = 0;
void loop()
{
  /*if (!modules.checkModuleStatus()) {
    Serial.println("Modules status is bad...");
    //if(!modules.isModuleWorking()){
    if (modules.getModuleStatus(1) < 0) {
      Serial.println("Module 1 is bad!");
      //Modules::ModuleType type = modules.getModuleType(1);
      requestReconfiguration(1);
    }
    if (modules.getModuleStatus(2) < 0) {
      Serial.println("Module 2 is bad!");
      requestReconfiguration(2);
    }
    }*/


  //Repond to bluetooth communication
  bt.checkForIncomingMsg();
  if (bt.hasRequest())
    getAndHandleNextBTRequest();


  //Respond to comminucation from the Zumo
  zumo.checkForIncoming();
  if (zumo.hasRequest())
    getAndHandleNextZumoMsg();

  handleCurrentState();



  //Allow the algorithm to update the heading constantly...?
  //if (imuTimer < millis()) {
  if (_state != Laser_Detection)
    imu.getHeading();
  // Serial.println(imu.getHeading());
  //  imuTimer = millis() + 300;
  //}
}



