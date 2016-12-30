#include "BT.h"


void BT::initialize(unsigned int robotNumber) {
  if (_initialized)
    return;
  _initialized = true;

  if (_robotNumber < 1)
    _robotNumber = robotNumber;
  _bt.begin(BT_BAUD);
  Serial.begin(9600);
  delay(1000); //Let it settle

  //Empty previous buffer
  _bt.clear();
  while (_bt.available())
    _bt.read();
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

    if(-1 < tempStr.indexOf(F("OK+CONNF"))){
      //Failed to connect. Accept this and set as slave
      setAsSlave();
      _msgBufferCounter = 0;
    }
  }
  return false;
}

String _clientRobotMacID;
void BT::setLastMsg(String msg) {
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

    case BT_REQUEST_HEADING_VALUE: {
        request.type = BT_REQUEST_HEADING;
        validRequest = true;
      }

    case BT_REQUEST_REPAIR_VALUE: {
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
  }

  if (validRequest) {
    _lastRequest = request;
    _hasRequest = true;
  } else if (validResponse) {
    _lastResponse = response;
    _hasResponse = true;
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
      case BT_REQUEST_CANCEL_LASER_DETECTION:{
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

    case BT_REQUEST_DRIVE:
    case BT_REQUEST_STOP:
    case BT_REQUEST_COARSE_LOCALIZATION:
    case BT_REQUEST_ALIGNMENT:
    case BT_REQUEST_DOCKING:
    case BT_REQUEST_MODULE_EXCHANGE:
    case BT_REQUEST_REPAIR: {
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
        _bt.print(res.valueDouble, DEC);
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
    if (_isMaster) {
      //Try to re-establish connection
      Serial.print(F("Trying to re-establish connection to "));
      Serial.print(_slaveMacID);
      Serial.println(F("..."));
      delay(2000);
      setAsMaster(_slaveMacID);
    } else {
      //Reset the BT device
      setAsSlave();
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
  //Serial.print("Setting device as slave");

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

  // Serial.print("Setting as master and connecting to: ");
  // Serial.println(macID);

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

