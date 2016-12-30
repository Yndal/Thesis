#ifndef BT_h
#define BT_h

#include "Arduino.h"

#define BT_BAUD 9600
#define bt Serial2
#define BT_END_CHAR ';'
#define BT_BUFFER 32

#define REQUEST_IR_ON  'a'
#define REQUEST_IR_OFF 'b'

class BT {
	enum Request{
		IR_ON,
		IR_OFF,
	}


	bool checkForIncomingMsg();
	Request getMessage();

  

  bool tryConnectTo(char mac[12]);
  void test(int *);




  private:
  	char msgBuffer[BT_BUFFER];
  	int8_t msgBufferCounter;
    bool initialiazed = false;
    bool hasNewRequest;
    Request lastRequest;
    void initialize();
  
  
};

