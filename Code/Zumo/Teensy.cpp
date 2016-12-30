#include "Teensy.h"


void Teensy::initialize() {
  if (_initialized)
    return;

  _teensy.begin(TEENSY_BAUD);
  Serial.begin(9600);
  delay(1000); //Let it settle


  for (uint8_t i = 0; i < TOTAL_REQUESTS; i++)
    _hasNewRequest[i] = false;

  for (uint8_t i = 0; i < TOTAL_RESPONSES; i++)
    _hasNewResponse[i] = false;


  //Empty previous buffer
  //  _teensy.clear();
  while (_teensy.available())
    _teensy.read();

}


//TODO Is this ever used??
void Teensy::sendRequest(Request request) {
  switch (request.type) {
    case TEENSY_REQUEST_IR_ON : _teensy.write(TEENSY_REQUEST_IR_ON_VALUE);  break;
    case TEENSY_REQUEST_IR_OFF: _teensy.write(TEENSY_REQUEST_IR_OFF_VALUE); break;
    case TEENSY_REQUEST_DRIVE : _teensy.write(TEENSY_REQUEST_DRIVE_VALUE);  break;
    case TEENSY_REQUEST_HEADING: _teensy.write(TEENSY_REQUEST_HEADING_VALUE); break;
  }
  _teensy.write(TEENSY_END_CHAR);
  _teensy.flush();
  /* if (!waitForOK(200)) {
     Serial.print("Unable to get OK from request: "); Serial.println(request.type);
    }*/
}

void Teensy::sendResponse(Response response) {
  //Serial.print("Sending response: ");
  switch (response.type) {
    case TEENSY_RESPONSE_IR_LEFT : {
        _teensy.write(TEENSY_RESPONSE_IR_LEFT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }

    case TEENSY_RESPONSE_IR_FRONT : {
        _teensy.write(TEENSY_RESPONSE_IR_FRONT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }


    case TEENSY_RESPONSE_IR_RIGHT : {
        _teensy.write(TEENSY_RESPONSE_IR_RIGHT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }

    case TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED: {
        _teensy.write(TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE);
        break;
      }

    case TEENSY_RESPONSE_ALIGNMENT_FINISHED: {
        _teensy.write(TEENSY_RESPONSE_ALIGNMENT_FINISHED_VALUE);
        break;
      }

    case TEENSY_RESPONSE_DOCKING_FINISHED: {
        _teensy.write(TEENSY_RESPONSE_DOCKING_FINISHED_VALUE);
        break;
      }
  }

  _teensy.write(TEENSY_END_CHAR);
  _teensy.flush();
  /*if (!waitForOK(200)) {
    Serial.print(F("Unable to get OK from response: ")); Serial.println(response.type);
    }*/

}

/*bool Teensy::waitForOK(unsigned long ms) {
  unsigned long endTime = millis() + ms;
  char buf[32];
  uint8_t counter = 0;
  while (millis() <= endTime) {
    if (_teensy.available()) {
      char c = _teensy.read();
      if (c == TEENSY_END_CHAR) {
        buf[counter] = '\0';
        if (String(buf).equals(TEENSY_OK_RESPONSE))
          return true;
        counter = 0;
      }
      buf[counter++] = c;
      //if(_teensy.indexOf(TEENSY_OK_RESPONSE)
      return true;
    }
  }

  return false;
  }
*/
bool Teensy::checkForIncomingMsg() {
  while (_teensy.available()) {
    char c = _teensy.read();
    // Serial.print(c);
    if (c == TEENSY_END_CHAR) {
      //Serial.println();
      //Serial.println(F("Teensy - checkForIncomingMsg() - got END_CHAR"));
      _msgBuffer[_msgBufferCounter] = '\0'; //Tell String where it should stop
      _msgBufferCounter = 0;
      String msg = String(_msgBuffer);
      //uint8_t l = msg.length();
      //Serial.print("String length: "); Serial.println(l);
      if (!msg.equals(F(""))) {
        setLastMsg(msg);
        return true;
      }
    }
    //else {
    //Add the char to the buffer
    _msgBuffer[_msgBufferCounter++] = c;
    //}
  }
  return false;
}

void Teensy::setLastMsg(String msg) {
  Serial.print(F("setLastMsg: ")); Serial.println(msg);
  bool isValidRequest = false;
  bool isValidResponse = false;
  Request request;
  Response response;
  switch (msg[0]) {
    case TEENSY_REQUEST_IR_ON_VALUE: {
        request.type = TEENSY_REQUEST_IR_ON;
        isValidRequest = true;
        break;
      }
    case TEENSY_REQUEST_IR_OFF_VALUE: {
        request.type = TEENSY_REQUEST_IR_OFF;
        isValidRequest = true;
        break;
      }
    case TEENSY_REQUEST_BEEP_VALUE: {
        request.type = TEENSY_REQUEST_BEEP;
        isValidRequest = true;
        break;
      }
      
    case TEENSY_REQUEST_DRIVE_VALUE: {
        request.type = TEENSY_REQUEST_DRIVE;

        uint8_t i = msg.indexOf(TEENSY_SEP_CHAR);
        int8_t c1 = String(msg.substring(1, i)).toInt();
        int8_t c2 = String(msg.substring(i + 1)).toInt();
        // Serial.print(F("Drive - val1: ")); Serial.print(c1); Serial.print(F(", val2: ")); Serial.println(c2);

        request.value1 = c1;
        request.value2 = c2;
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_DRIVE_MM_VALUE: {
        request.type = TEENSY_REQUEST_DRIVE_MM;

        uint8_t i = msg.indexOf(TEENSY_SEP_CHAR);
        int v1 = String(msg.substring(1, i)).toInt();
        int v2 = String(msg.substring(i + 1)).toInt();

        request.value1 = v1;
        request.value2 = v2;
        isValidRequest = true;
        break;
      }
      
    case TEENSY_REQUEST_TURN_LEFT_VALUE: {
        request.type = TEENSY_REQUEST_TURN_LEFT;

        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_STOP_VALUE: {
        request.type = TEENSY_REQUEST_STOP;
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_WORK_VALUE: {
        request.type = TEENSY_REQUEST_WORK;
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_IR_LEFT_VALUE: {
        request.type = TEENSY_REQUEST_IR_LEFT;
        request.value1 = String(msg[1]).toInt();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_IR_FRONT_VALUE: {
        request.type = TEENSY_REQUEST_IR_FRONT;
        request.value1 = String(msg[1]).toInt();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_IR_RIGHT_VALUE: {
        request.type = TEENSY_REQUEST_IR_RIGHT;
        request.value1 = String(msg[1]).toInt();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_COARSE_LOCALIZATION_VALUE: {
        request.type = TEENSY_REQUEST_COARSE_LOCALIZATION;
        request.value1 = msg.substring(1).toInt();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_ALIGMENT_VALUE: {
        request.type = TEENSY_REQUEST_ALIGMENT;
        request.valueDouble = msg.substring(1).toFloat();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_DOCKING_VALUE: {
        request.type = TEENSY_REQUEST_DOCKING;
        request.valueDouble = msg.substring(1).toFloat();
        isValidRequest = true;
        break;
      }

    /*  case TEENSY_REQUEST_MOVE_FORWARD_VALUE: {
        Serial.println("Move forward Request set");
          request.type = TEENSY_REQUEST_MOVE_FORWARD;
          request.value1 = String(msg.substring(1)).toInt();
          isValidValue = true;
          break;
        }*/




    /*******************************
       Responses
     *******************************/
    case TEENSY_RESPONSE_HEADING_VALUE: {
        // Serial.print(F("Teensy::Got Heading response! "));
        response.type = TEENSY_RESPONSE_HEADING;
        response.valueDouble = msg.substring(1).toFloat();
        // Serial.println(response.valueDouble);
        isValidResponse = true;
        break;
      }

    case TEENSY_RESPONSE_LASER_DETECTED_VALUE: {
        //Serial.println(F("GOT LASER DETECTED RESPONSE"));
        response.type = TEENSY_RESPONSE_LASER_DETECTED;
        isValidResponse = true;

        break;
      }

    default: {
        Serial.print(F("Got unknown msg: "));
        Serial.println(msg);
      }


  }

  if (isValidRequest) {
    _latestRequest = request;
    _lastRequest[request.type] = request;
    _hasLatestRequest = true;
    _hasNewRequest[request.type] = true;
  }

  if (isValidResponse) {
    /*Serial.print("Valid response: ");
      Serial.println(response.type);*/
    _latestResponse = response;
    _lastResponse[response.type] = response;
    _hasLatestResponse = true;
    _hasNewResponse[response.type] = true;
  }
}


boolean Teensy::hasNewRequest() {
  return _hasLatestRequest;
}

boolean Teensy::hasNewRequest(RequestType type) {
  return _hasNewRequest[type];
}


boolean Teensy::hasNewResponse(ResponseType type) {
  return _hasNewResponse[type];
}


Teensy::Request Teensy::getNextRequest() {
  _hasLatestRequest = false;
  return _latestRequest;
}

Teensy::Request Teensy::getNextRequest(RequestType type) {
  _hasNewRequest[type] = false;
  return _latestRequest;
}


/*Teensy::Response Teensy::getNextResponse() {
  _hasLatestResponse = false;
  return _latestResponse;
  }*/

Teensy::Response Teensy::getNextResponse(ResponseType type) {
  _hasNewResponse[type] = false;
  return _lastResponse[type];
}





