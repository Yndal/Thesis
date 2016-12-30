#include "BT.h"


void BT::initialize(unsigned int robotNumber) {
  if (_initialized)
    return;
  _initialized = true;
  tryReconnect = true;

  if (_robotNumber < 1)
    _robotNumber = robotNumber;
  _bt.begin(BT_BAUD);
  Serial.begin(9600);
  delay(1000); //Let it settle

  //Empty previous buffer
  _bt.clear();
  while (_bt.available())
    _bt.read();

  //Get MacID
  _bt.write("AT");
  delay(200);
  _bt.write("AT+ADDR?");
  //_bt.write(F("AT+ADDR?"));
  delay(500);

  //while (_bt.read() != ':');
  char buff[32];
  int counter = 0;
  while (_bt.available()) {
    char c = _bt.read();
    //   Serial.print(c);
    buff[counter++] = c;
  }
  buff[counter] = '\0';

  String buffStr = String(buff);
  uint8_t index = buffStr.indexOf(':');
  if (-1 < index)
    _thisMacID = buffStr.substring(index + 1);
  Serial.print("Has macID: ");
  Serial.println(_thisMacID);


}

void BT::reportInToServiceStation(String hostMacID, uint8_t robotNumber, String macID, Modules::ModuleType leftType, Modules::ModuleType rightType) {
  setAsMaster(hostMacID);
  _bt.print(BT_REPORT_IN_VALUE);
  _bt.print(robotNumber);
  _bt.print(BT_SEP_CHAR);
  _bt.print(macID);
  _bt.print(BT_SEP_CHAR);

  switch (leftType) {
    case Modules::Battery: _bt.print('b'); break;
    case Modules::Laser: _bt.print('l'); break;
    case Modules::Motor: _bt.print('m'); break;
    case Modules::LDR: _bt.print('d'); break;
    case Modules::Empty: _bt.print('e'); break;
  }
  _bt.print(BT_SEP_CHAR);

  switch (rightType) {
    case Modules::Battery: _bt.print('b'); break;
    case Modules::Laser: _bt.print('l'); break;
    case Modules::Motor: _bt.print('m'); break;
    case Modules::LDR: _bt.print('d'); break;
    case Modules::Empty: _bt.print('e'); break;
  }
  _bt.print(BT_END_CHAR);
  setAsSlave();

}


void BT::requestReconfiguration(String hostMac, uint8_t robotNumber, Modules::ModuleType type, int8_t module) {
  setAsMaster(hostMac);
  delay(500);
  _bt.print(BT_REQUEST_RENCONFIGURATION_TO_HOST_VALUE);
  _bt.print(robotNumber);
  _bt.print(BT_SEP_CHAR);

  //type
  switch (type) {
    case Modules::Battery: _bt.print('b'); break;
    case Modules::Laser: _bt.print('l'); break;
    case Modules::Motor: _bt.print('m'); break;
    case Modules::LDR: _bt.print('d'); break;
    case Modules::Empty: _bt.print('e'); break;
  }
  _bt.print(BT_SEP_CHAR);

  //position
  char p = module == 1 ? 'l' : 'r';
  _bt.print(p);
  _bt.print(BT_END_CHAR);
  delay(200);

  setAsSlave();
}


String BT::getThisMacID() {
  return _thisMacID;
}

bool BT::checkForIncomingMsg() {
  while (_bt.available()) {
    char c = _bt.read();
    Serial.print(c);
    if (c == BT_END_CHAR) {
      //TODO Translate into BT_REQUESt

      _msgBuffer[_msgBufferCounter] = '\0'; //Tell String where it should stop
      setLastMsg(String(_msgBuffer));
      _msgBufferCounter = 0;
      return true;
    }

    //Add the char to the buffer
    _msgBuffer[_msgBufferCounter++] = c;

    //Create temp String with the current buffer
    _msgBuffer[_msgBufferCounter] = '\0'; //Tell String where it should stop
    String tempStr = String(_msgBuffer); //Create the string
    if (-1 < tempStr.indexOf(F("OK+CONN"))) {
      //Device is connected
      onConnected();
      _msgBufferCounter = 0;
    }
    if (-1 < tempStr.indexOf(F("OK+LOST"))) {
      //Lost connection
      onDisconnected();
      _msgBufferCounter = 0;
      //  Serial.print("TempStr: "); Serial.println(tempStr);
    }

    if (-1 < tempStr.indexOf(F("OK+CONNF"))) {
      //Failed to connect. Accept this and set as slave
      setAsSlave();
      _msgBufferCounter = 0;
    }
  }
  return false;
}

String _clientRobotMacID;
void BT::setLastMsg(String msg) {
  Serial.print("BT::SetLastMsg: ");
  Serial.println(msg);

  bool validRequest = false;
  bool validResponse = false;
  Request request;
  Response response;
  switch (msg[0]) {
    case BT_REQUEST_IR_ON_VALUE: {
        request.type = BT_REQUEST_IR_ON;
        validRequest = true;
        break;
      }

    case BT_REQUEST_IR_OFF_VALUE: {
        request.type = BT_REQUEST_IR_OFF;
        validRequest = true;
        break;
      }

    case BT_REQUEST_NOTIFY_ON_LASER_DETECTION_VALUE: {
        request.type = BT_REQUEST_NOTIFY_ON_LASER_DETECTION;
        validRequest = true;
        break;
      }

    case BT_REQUEST_CANCEL_LASER_DETECTION_VALUE: {
        request.type = BT_REQUEST_CANCEL_LASER_DETECTION;
        validRequest = true;
        break;
      }

    case BT_REQUEST_LASER_DETECTED_VALUE: {
        request.type = BT_REQUEST_LASER_DETECTED;
        validRequest = true;
        break;
      }

    case BT_REQUEST_DRIVE_VALUE: {
        request.type = BT_REQUEST_DRIVE;
        int i = msg.indexOf(BT_SEP_CHAR);
        int val1 = msg.substring(1, i).toInt();
        int val2 = msg.substring(i + 1).toInt();
        // Serial.print("Val1: "); Serial.print(val1); Serial.print(", val2: "); Serial.println(val2);
        request.value1 = val1;
        request.value2 = val2;
        validRequest = true;
        break;
      }

    case BT_REQUEST_STOP_VALUE: {
        request.type = BT_REQUEST_STOP;
        validRequest = true;
        break;
      }

    case BT_REQUEST_BEEP_VALUE: {
        request.type = BT_REQUEST_BEEP;
        validRequest = true;
        break;
      }

    case BT_REQUEST_RECONFIGURATION_VALUE: {
        Serial.println("GOT RECONF REQUEST!!");
        request.type = BT_REQUEST_RECONFIGURATION;
        int8_t i = msg.indexOf(BT_SEP_CHAR);
        request.valueStr = msg.substring(1, i);
        msg = msg.substring(i + 1);

        i = msg.indexOf(BT_SEP_CHAR);
        request.value1 = msg.substring(0, 1).toInt();
        msg = msg.substring(i + 1);

        request.valueBool = false;
        char cBool = msg.charAt(0);
        if (cBool == 'y')
          request.valueBool = true;

       /* Serial.print("Got bool from reconf: ");
        Serial.println(request.valueBool ? "true" : "false");*/

        validRequest = true;
        break;
      }

    case BT_REQUEST_COARSE_LOCALIZATION_VALUE: {
        request.type = BT_REQUEST_COARSE_LOCALIZATION;
        validRequest = true;
        break;
      }

    case BT_REQUEST_ALIGNMENT_VALUE: {
        request.type = BT_REQUEST_ALIGNMENT;
        request.valueDouble = msg.substring(1).toFloat();
        validRequest = true;
        break;
      }

    case BT_REQUEST_DOCKING_VALUE: {
        request.type = BT_REQUEST_DOCKING;
        validRequest = true;
        break;
      }

    case BT_REQUEST_MODULE_EXCHANGE_VALUE: {
        request.type = BT_REQUEST_MODULE_EXCHANGE;
        int i = msg.indexOf(BT_SEP_CHAR);
        int ii = msg.lastIndexOf(BT_SEP_CHAR);
        int macID = msg.substring(1, i).toInt();
        int side = msg.substring(i + 1, ii).toInt();
        int type = msg.substring(ii + 1).toInt();
        request.value1 = macID;
        request.value2 = side;
        request.value3 = type;
        validRequest = true;
        break;
      }

    /* case BT_REQUEST_MODULE_EXCHANGE_START_VALUE: {
         request.type = BT_REQUEST_MODULE_EXCHANGE_START;
         validRequest = true;
         break;

         break;
       }*/

    case BT_REQUEST_HEADING_VALUE: {
        request.type = BT_REQUEST_HEADING;
        validRequest = true;
        break;
      }

    /* case BT_REQUEST_REPAIR_VALUE: {
         request.type = BT_REQUEST_REPAIR;
         int i = msg.indexOf(BT_SEP_CHAR);
         // int ii = msg.lastIndexOf(BT_SEP_CHAR);
         String macID = msg.substring(1, i);
         //int side = msg.substring(i+1,ii).toInt();
         //int type = msg.substring(ii+1).toInt();
         request.valueStr = macID;
         //request.value2 = side;
         //request.value3 = type;
         validRequest = true;
         break;
       }*/

    case BT_REQUEST_MODULE_INSERTION_VALUE: {
        request.type = BT_REQUEST_MODULE_INSERTION;
        int val1 = msg.substring(1,2).toInt();
        request.value1 = val1;
        request.valueBool = msg.substring(3,4).toInt();
        Serial.print("BT::Got insertion of module ");
        Serial.println(val1);

        validRequest = true;
        break;
      }

    /*********************
      Tests
    *********************/
    case BT_REQUEST_MODULE_EXCHANGE_TEST_VALUE: {
        request.type = BT_REQUEST_MODULE_EXCHANGE_TEST;
        int i = msg.indexOf(BT_SEP_CHAR);
        int val1 = msg.substring(1, i).toInt();
        String macID = msg.substring(i + 1);
        request.value1 = val1;
        request.valueStr = macID;

        validRequest = true;
        break;
      }




    /*********************
       Responses
     *********************/
    case BT_RESPONSE_HEADING_VALUE: {
        response.type = BT_RESPONSE_HEADING;
        response.valueDouble = msg.substring(1).toFloat();

        validResponse = true;
        break;
      }

    case BT_RESPONSE_MODULE_EXCHANGE_STOPPED_VALUE: {
        response.type = BT_RESPONSE_MODULE_EXCHANGE_STOPPED;

        validResponse = true;
        break;
      }
  }

  if (validRequest) {
    _lastRequest = request;
    _hasRequest = true;
  } else if (validResponse) {
    _lastResponse = response;
    _hasResponse = true;
  } else {
    Serial.println("Invalid msg...");
  }
}


void BT::sendRequest(Request req) {
  switch (req.type) {
    case BT_REQUEST_IR_ON: {
        _bt.write(BT_REQUEST_IR_ON_VALUE);
        break;
      }
    case BT_REQUEST_IR_OFF: {
        _bt.write(BT_REQUEST_IR_OFF_VALUE);
        break;
      }
    case BT_REQUEST_NOTIFY_ON_LASER_DETECTION: {
        _bt.write(BT_REQUEST_NOTIFY_ON_LASER_DETECTION_VALUE);
        break;
      }
    case BT_REQUEST_CANCEL_LASER_DETECTION: {
        _bt.write(BT_REQUEST_CANCEL_LASER_DETECTION_VALUE);
        break;
      }
    case BT_REQUEST_LASER_DETECTED: {
        _bt.write(BT_REQUEST_LASER_DETECTED_VALUE);
        break;
      }
    case BT_REQUEST_BEEP: {
        _bt.write(BT_REQUEST_BEEP_VALUE);
        break;
      }

    case BT_REQUEST_HEADING: {
        _bt.write(BT_REQUEST_HEADING_VALUE);
        break;
      }

    case BT_REQUEST_MODULE_INSERTION: {
        _bt.write(BT_REQUEST_MODULE_INSERTION_VALUE);
        _bt.print(req.value1);
        _bt.write(BT_SEP_CHAR);
        _bt.print(req.valueBool);
        break;
      }

    case BT_REQUEST_RECONFIGURATION:
    case BT_REQUEST_DRIVE:
    case BT_REQUEST_STOP:
    case BT_REQUEST_COARSE_LOCALIZATION:
    case BT_REQUEST_ALIGNMENT:
    case BT_REQUEST_DOCKING:
    case BT_REQUEST_MODULE_EXCHANGE:
    case BT_REQUEST_MODULE_EXCHANGE_TEST:
      //case BT_REQUEST_REPAIR:
      {
        Serial.print(F("BT is not allowed to request that..."));
        Serial.println(req.type);
        break;
      }

  }
  _bt.write(BT_END_CHAR);
  _bt.flush();
}


void BT::sendResponse(Response res) {
  switch (res.type) {
    case BT_RESPONSE_HEADING: {
        _bt.write(BT_RESPONSE_HEADING_VALUE);
        _bt.print(res.valueDouble, 2);
        Serial.print("BT::SendResponse() - Heading with value ");
        Serial.println(res.valueDouble, 2);
        break;
      }

    case BT_RESPONSE_MODULE_EXCHANGE_STOPPED: {
        _bt.write(BT_RESPONSE_MODULE_EXCHANGE_STOPPED_VALUE);
        break;
      }
  }
  _bt.write(BT_END_CHAR);
  _bt.flush();
}

void BT::onConnected() {
  _isConnected = true;
  // Serial.println("We are connected to someone!!");



}

void BT::onDisconnected() {
  _isConnected = false;
  //Serial.print("We lost the connection");

  if (!_isConnecting) {
    if (tryReconnect && _isMaster) {
      tryReconnect = false;
      //Try to re-establish connection
      Serial.print(F("Trying to re-establish connection to "));
      Serial.print(_slaveMacID);
      Serial.println(F("..."));
      delay(2000);
      setAsMaster(_slaveMacID);
    } else {
      //Reset the BT device
      setAsSlave();
      tryReconnect = true;
    }
  }
}

bool BT::hasRequest() {
  return _hasRequest;
}


bool BT::hasResponse() {
  return _hasResponse;
}
BT::Request BT::getNextRequest() {
  _hasRequest = false;
  return _lastRequest;
}

BT::Response BT::getNextResponse() {
  _hasResponse = false;
  return _lastResponse;
}



void BT::setAsSlave() {
  _isConnecting = true;
  initialize(0);
  Serial.print("Setting device as BT slave");

  char buf[64];
  uint8_t length = 8;//sizeof(cmdsMaster);
  //Serial.print("Length: "); Serial.println(length);
  for (uint8_t i = 0; i < length; i++) {
    cmdsSlave[i].toCharArray(buf, sizeof(buf));

    if (cmdsSlave[i].startsWith("AT+NAME")) {
      String str = String(buf);
      str += String(_robotNumber);
      _bt.print(str);

      Serial.print(str); Serial.print(": ");
      delay(1000);
      checkForIncomingMsg();
      Serial.println();
      continue;
      // buf[cmdsSlave[i].length() + 1] = buf[cmdsSlave[i].length()];
      //buf[cmdsSlave[i].length()] = String(_robotNumber)[0];
    }


    _bt.write(buf);
    Serial.print(buf); Serial.print(": ");
    delay(1000);
    checkForIncomingMsg();
    Serial.println();
  }
  _isMaster = false;
  _slaveMacID = "";
  _isConnecting = false;
}

/*
   Set device as master and connect to the slave with the passed on macID
*/
void BT::setAsMaster(String macID) {
  _isConnecting = true;
  initialize(0);

  Serial.print("Setting as BT master and connecting to: ");
  Serial.println(macID);

  char buf[64];
  uint8_t length = 8;//sizeof(cmdsMaster);
  // Serial.print("Length: "); Serial.println(length);
  for (uint8_t i = 0; i < length - 1; i++) {
    cmdsMaster[i].toCharArray(buf, sizeof(buf));

    if (cmdsMaster[i].startsWith("AT+NAME")) {
      String str = String(buf);
      str += String(_robotNumber);
      _bt.print(str);

      Serial.print(str); Serial.print(": ");
      delay(1000);
      checkForIncomingMsg();
      Serial.println();
      continue;
      //buf[cmdsMaster[i].length() + 1] = buf[cmdsMaster[i].length()];
      //buf[cmdsMaster[i].length()] = String(_robotNumber)[0];
    }

    _bt.write(buf);
    Serial.print(buf); Serial.print(": ");
    delay(1000);
    checkForIncomingMsg();
    Serial.println();
  }

  cmdsMaster[length - 1].toCharArray(buf, sizeof(buf)); //TODO Compiler is giving a warning here...
  String(buf + macID).toCharArray(buf, sizeof(buf));
  _bt.write(buf);
  Serial.print(buf); Serial.print(": ");
  _isMaster = true;
  _slaveMacID = macID;
  delay(500);
  checkForIncomingMsg();

  _isConnecting = false;
}

bool BT::isConnected() {
  return _isConnected;
}

bool BT::isMaster() {
  return _isMaster;
}

