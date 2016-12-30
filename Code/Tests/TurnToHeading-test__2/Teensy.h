#ifndef Teensy_h
#define Teensy_h

#include "Arduino.h"

#define TEENSY_BAUD     9600
#define _teensy         Serial1
#define TEENSY_END_CHAR ';'
#define TEENSY_SEP_CHAR ','
#define TEENSY_BUFFER   32

#define TEENSY_REQUEST_IR_ON_VALUE                          'a'
#define TEENSY_REQUEST_IR_OFF_VALUE                         'b'
#define TEENSY_REQUEST_BEEP_VALUE                           'c'
#define TEENSY_REQUEST_DRIVE_VALUE                          'd'
#define TEENSY_REQUEST_STOP_VALUE                           'e'
#define TEENSY_REQUEST_IR_LEFT_VALUE                        'f'
#define TEENSY_REQUEST_IR_FRONT_VALUE                       'g'
#define TEENSY_REQUEST_IR_RIGHT_VALUE                       'h'
#define TEENSY_REQUEST_COARSE_LOCALIZATION_VALUE            'i'
#define TEENSY_REQUEST_ALIGMENT_VALUE                       'j'
#define TEENSY_REQUEST_HEADING_VALUE                        'k'
#define TEENSY_REQUEST_TURN_LEFT_VALUE                      'l'
#define TEENSY_REQUEST_DOCKING_VALUE                        'm'

#define TEENSY_RESPONSE_DOCKING_FINISHED_VALUE              'r'
#define TEENSY_RESPONSE_LASER_DETECTED_VALUE                's'
#define TEENSY_RESPONSE_ALIGNMENT_FINISHED_VALUE            't'
#define TEENSY_RESPONSE_HEADING_VALUE                       'u'
#define TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE  'v'
#define TEENSY_RESPONSE_STOPPED_FRONT_MIN_DISTANCE_VALUE    'w'
#define TEENSY_RESPONSE_IR_LEFT_VALUE                       'x'
#define TEENSY_RESPONSE_IR_FRONT_VALUE                      'y'
#define TEENSY_RESPONSE_IR_RIGHT_VALUE                      'z'



//#define TEENSY_OK_RESPONSE          "OK"

//#define COMMANDS_ARRAY_SIZE 7

class Teensy {
  public:
    enum RequestType {
      TEENSY_REQUEST_IR_ON,
      TEENSY_REQUEST_IR_OFF,
      TEENSY_REQUEST_BEEP,
      TEENSY_REQUEST_DRIVE,
      TEENSY_REQUEST_TURN_LEFT,
      TEENSY_REQUEST_STOP,
      TEENSY_REQUEST_HEADING,

      TEENSY_REQUEST_IR_LEFT,
      TEENSY_REQUEST_IR_FRONT,
      TEENSY_REQUEST_IR_RIGHT,
      TEENSY_REQUEST_COARSE_LOCALIZATION,
      TEENSY_REQUEST_ALIGMENT,
      TEENSY_REQUEST_DOCKING,
      };
    class Request {
      public:
        RequestType type;
        int value1 = 0;
        int value2 = 0;
        double valueDouble1 = 0.0;};

    enum ResponseType {
      TEENSY_RESPONSE_IR_LEFT,
      TEENSY_RESPONSE_IR_FRONT,
      TEENSY_RESPONSE_IR_RIGHT,
      TEENSY_RESPONSE_HEADING,
      TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED,
      TEENSY_RESPONSE_ALIGNMENT_FINISHED,
      TEENSY_RESPONSE_DOCKING_FINISHED,
      TEENSY_RESPONSE_STOPPED_FRONT_MIN_DISTANCE,
      TEENSY_RESPONSE_LASER_DETECTED,
      };
    class Response {
      public:
        ResponseType type;
        int value1;
        double valueDouble;};

    void initialize();
    bool checkForIncomingMsg();
    Request getNextRequest();
    Response getNextResponse();
    void sendRequest(Request request);
    void sendResponse(Response response);
    bool hasNewRequest();
    bool hasNewResponse();

  private:
    bool _initialized = false;

    bool waitForOK(unsigned long ms);

    char _msgBuffer[TEENSY_BUFFER];
    int8_t _msgBufferCounter;
    void setLastMsg(String msg);
    Request _lastRequest;
    Response _lastResponse;
    bool _hasNewRequest = false;
    bool _hasNewResponse = false;


};

#endif
