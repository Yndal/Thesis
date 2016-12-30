#include "BT.h"

	void initialize(){
		bt.begin(BT_BAUD);
		delay(1000); //Let it settle

	}

	bool BT::checkForIncomingMsg(){
		while(bt.available()){
			char c = bt.read();
			if(c == BT_END_CHAR){
				//Translate into Request
				
				msgBufferCounter = 0;
			} else {
				msgBuffer[msgBufferCounter++] = c;
			}
		}

		return true;
	}

	Request getMessage(){

	}



bool BT::tryConnectTo(char mac[12]){
  if(!initialized)
    initialize();


      return true;
}

void BT::test(int *){

  
}

