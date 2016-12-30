#ifndef BT_h
#define BT_h

#include "Arduino.h"

#define BT_BAUD     9600
#define _bt         Serial3
#define BT_END_CHAR ';'
#define BT_SEP_CHAR ','
#define BT_BUFFER   32

#define BT_REQUEST_IR_ON_VALUE                       'a'
#define BT_REQUEST_IR_OFF_VALUE                      'b'
#define BT_REQUEST_BEEP_VALUE                        'c'
#define BT_REQUEST_DRIVE_VALUE                       'd'
#define BT_REQUEST_STOP_VALUE                        'e'
#define BT_REQUEST_MODULE_EXCHANGE_VALUE             'f'
#define BT_REQUEST_COARSE_LOCALIZATION_VALUE         'i'
#define BT_REQUEST_HEADING_VALUE                     'h'
#define BT_REQUEST_ALIGNMENT_VALUE                   'm' //Remove?
#define BT_REQUEST_NOTIFY_ON_LASER_DETECTION_VALUE   'l'
#define BT_REQUEST_CANCEL_LASER_DETECTION_VALUE      'n'
#define BT_REQUEST_LASER_DETECTED_VALUE              'o'

#define BT_REQUEST_DOCKING_VALUE                     'p' //Remove?

#define BT_REQUEST_REPAIR_VALUE                      'r'

#define BT_RESPONSE_HEADING_VALUE                    'z'




//#define COMMANDS_ARRAY_SIZE 7

class BT {
  public:
     enum RequestType {
      BT_REQUEST_IR_ON,
      BT_REQUEST_IR_OFF,
      BT_REQUEST_NOTIFY_ON_LASER_DETECTION,
      BT_REQUEST_CANCEL_LASER_DETECTION,
      BT_REQUEST_LASER_DETECTED,
      BT_REQUEST_BEEP,
      BT_REQUEST_DRIVE,
      BT_REQUEST_STOP,
      BT_REQUEST_MODULE_EXCHANGE,
      BT_REQUEST_COARSE_LOCALIZATION,
      BT_REQUEST_ALIGNMENT,
      BT_REQUEST_DOCKING,
      BT_REQUEST_HEADING,
      
      BT_REQUEST_REPAIR,
    };
    
    class Request {
      public:
        RequestType type;
        int value1 = 0;
        int value2 = 0;
        int value3 = 0;
        int valueDouble = 0.0;
        String valueStr = "";
    };

    enum ResponseType{
      BT_RESPONSE_HEADING,
    };
    class Response{
      public:
        ResponseType type;
        int valueInt;
        double valueDouble;
        String valueStr;
    };
    
    void initialize(unsigned int robotNumber);
    
    bool checkForIncomingMsg();
    bool hasResponse();
    bool hasRequest();
    Request getNextRequest();
    Response getNextResponse();
  
    void setAsSlave();
    void setAsMaster(String macID);
    void sendRequest(Request req);
    void sendResponse(Response res);
    bool isConnected();
    bool isMaster();



  private:
    bool _initialized = false;

    bool _isConnecting = false;
    bool _isConnected;
    bool _isMaster;
    String _slaveMacID;

    int _robotNumber;
  	
  	char _msgBuffer[BT_BUFFER];
    int8_t _msgBufferCounter;
    void setLastMsg(String msg);
    bool _hasResponse = false;
    bool _hasRequest = false;
    Request _lastRequest;
    Response _lastResponse;
    
    void onConnected();
    void onDisconnected();
    String cmdsMaster[8] = {
        F("AT"),
        F("AT+RESET"),
        F("AT+NAMERobot_"),
        F("AT+IMME1"),
        F("AT+NOTI1"),
        F("AT+ROLE1"),
        F("AT+SHOW1"),
        F("AT+CON"), //AT+CON884AEA3B9618
        };
    String cmdsSlave[8] = {
        F("AT"),
        F("AT+RESET"),
        F("AT+NAMERobot_"),
        F("AT+IMME1"),
        F("AT+NOTI1"),
        F("AT+ROLE0"),
        F("AT+FILT"),
        F("AT+SHOW1"),
        };  
  };

#endif
