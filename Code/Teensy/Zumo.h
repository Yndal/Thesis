#ifndef Zumo_h
#define Zumo_h

#include "Arduino.h"
//#include <TimerThree.h> //TODO Should be used for Zumo heart beat!!

#define ZUMO_BAUD     9600
#define _zumo         Serial2
#define ZUMO_END_CHAR ';'
#define ZUMO_SEP_CHAR ','
#define ZUMO_BUFFER   32

/*************************************************
 * Requests                                      *
 *************************************************/
#define REQUEST_IR_ON_VALUE                            'a'
#define REQUEST_IR_OFF_VALUE                           'b'
#define REQUEST_BEEP_VALUE                             'c'
#define REQUEST_DRIVE_VALUE                            'd'
#define REQUEST_STOP_VALUE                             'e'
#define REQUEST_IR_LEFT_VALUE                          'f'
#define REQUEST_IR_FRONT_VALUE                         'g'
#define REQUEST_IR_RIGHT_VALUE                         'h'
#define REQUEST_COARSE_LOCALIZATION_VALUE              'i'
#define REQUEST_ALIGNMENT_VALUE                        'j'
#define REQUEST_HEADING_VALUE                          'k'
#define REQUEST_TURN_LEFT_VALUE                        'l'
#define REQUEST_DOCKING_VALUE                          'm'
#define REQUEST_WORK_VALUE                             'n'
#define REQUEST_DRIVE_MM_VALUE                         'o'

/*************************************************
 * Responses                                     *
 *************************************************/
#define RESPONSE_DOCKING_FINISHED_VALUE                'r'
#define RESPONSE_LASER_DETECTED_VALUE                  's'
#define RESPONSE_ALIGNMENT_FINISHED_VALUE              't'
#define RESPONSE_HEADING_VALUE                         'u'
#define RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE    'v' 
#define RESPONSE_STOPPED_FRONT_MIN_DISTANCE_VALUE      'w'
#define RESPONSE_IR_LEFT_VALUE                         'x'
#define RESPONSE_IR_FRONT_VALUE                        'y'
#define RESPONSE_IR_RIGHT_VALUE                        'z'



class Zumo {
  public:
     enum RequestType {
      REQUEST_IR_ON,
      REQUEST_IR_OFF,
      REQUEST_BEEP,
      REQUEST_DRIVE,
      REQUEST_DRIVE_MM,
      REQUEST_STOP,
      REQUEST_WORK,
      REQUEST_TURN_LEFT,

      REQUEST_IR_LEFT,
      REQUEST_IR_FRONT,
      REQUEST_IR_RIGHT,
      REQUEST_HEADING,
      
      REQUEST_COARSE_LOCALIZATION,
      REQUEST_ALIGNMENT,
      REQUEST_DOCKING,      
    };
    class Request{
      public:
        RequestType type;
        int value1 = 0;
        int value2 = 0;
        int value3 = 0;
        double valueDouble = 0.0;
        String valueStr = "";
    };

    enum ResponseType {
      RESPONSE_IR_LEFT,
      RESPONSE_IR_FRONT,
      RESPONSE_IR_RIGHT,
      RESPONSE_HEADING,
      RESPONSE_COARSE_LOCALIZATION_FINISHED,
      RESPONSE_ALIGNMENT_FINISHED,
      RESPONSE_DOCKING_FINISHED,
      RESPONSE_LASER_DETECTED,
      RESPONSE_STOPPED_FRONT_MIN_DISTANCE,
    };
    class Response {
      public:
        ResponseType type;
        int valueInt = 0;
        double valueDouble = 0.0;
    };
   
    void initialize();
    void checkForIncoming();
    bool hasResponse();
    bool hasRequest();
    Request getNextRequest();
    Response getNextResponse();
    void sendRequest(Request request);
    void sendResponse(Response response);
    

  private:
    bool _initialized = false;
    bool _hasResponse = false;
    bool _hasRequest = false;

    bool waitForOK(unsigned long ms);
    void setLastMsg(String msg);
    char _msgBuffer[ZUMO_BUFFER];
    uint8_t _msgBufferCounter = 0;
    
    Request _lastRequest;
    Response _lastResponse;
    
  };

#endif
