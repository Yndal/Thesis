
#include "Zumo.h"



void Zumo::initialize() {
  if (_initialized)
    return;

  _zumo.begin(ZUMO_BAUD);
  Serial.begin(9600);
  delay(1000); //Let it settle

  //Empty previous buffer
  _zumo.clear();
  while (_zumo.available())
    _zumo.read();

  Request req;
  req.type = REQUEST_STOP;
  sendRequest(req);

}

void Zumo::sendRequest(Request request) {
  switch (request.type) {
    case REQUEST_IR_ON : {
        _zumo.write(REQUEST_IR_ON_VALUE);
        break;
      }

    case REQUEST_IR_OFF: {
        _zumo.write(REQUEST_IR_OFF_VALUE);
        break;
      }

    case REQUEST_BEEP  : {
        _zumo.write(REQUEST_BEEP_VALUE);
        break;
      }

    case REQUEST_DRIVE : {
        _zumo.write(REQUEST_DRIVE_VALUE);
        _zumo.print(request.value1, DEC);
        _zumo.write(ZUMO_SEP_CHAR);
        _zumo.print(request.value2, DEC);
        break;
      }

    case REQUEST_DRIVE_MM : {
        _zumo.write(REQUEST_DRIVE_MM_VALUE);
        _zumo.print(request.value1, DEC);
        _zumo.write(ZUMO_SEP_CHAR);
        _zumo.print(request.value2, DEC);
        break;
      }

    case REQUEST_TURN_LEFT: {
        _zumo.write(REQUEST_TURN_LEFT_VALUE);
        break;
      }

    case REQUEST_STOP  : {
        _zumo.write(REQUEST_STOP_VALUE);
        break;
      }

    case REQUEST_WORK: {
        _zumo.write(REQUEST_WORK_VALUE);
        break;
      }

    case REQUEST_IR_LEFT: {
        _zumo.write(REQUEST_IR_LEFT_VALUE);
        _zumo.print(request.value1, DEC);
        break;
      }

    case REQUEST_IR_FRONT: {
        _zumo.write(REQUEST_IR_FRONT_VALUE);
        _zumo.print(request.value1, DEC);
        break;
      }

    case REQUEST_IR_RIGHT: {
        _zumo.write(REQUEST_IR_RIGHT_VALUE);
        _zumo.print(request.value1, DEC);
        break;
      }

    case REQUEST_COARSE_LOCALIZATION: {
        _zumo.write(REQUEST_COARSE_LOCALIZATION_VALUE);
        _zumo.print(request.value1, DEC);
        break;
      }

    case REQUEST_ALIGNMENT: {
        _zumo.write(REQUEST_ALIGNMENT_VALUE);
        _zumo.print(request.valueDouble, 2);
        break;
      }

    case REQUEST_DOCKING: {
        Serial.print("Sending Docking request to Zumo with heading: ");
        _zumo.write(REQUEST_DOCKING_VALUE);
        _zumo.print(request.valueDouble, 2);
        Serial.println(request.valueDouble, 2);

        break;
      }

    case REQUEST_HEADING: {
        _zumo.write(REQUEST_HEADING_VALUE);
        break;
      }
    default: {
        Serial.print(F("Unable to send unknown request: "));
        Serial.println(request.type);
        break;
      }
  }
  _zumo.write(ZUMO_END_CHAR);
  _zumo.flush();
  /*if (!waitForOK(500)) {
    Serial.print("Unable to get OK from request: "); Serial.println(request);
    }*/
}

void Zumo::sendResponse(Response response) {
  switch (response.type) {
    case RESPONSE_HEADING : {
        _zumo.write(RESPONSE_HEADING_VALUE);
        _zumo.print(response.valueDouble, 2);
        //Serial.print(F("Sending heading response with value response.valueDouble: "));
        //Serial.println(response.valueDouble);
        // delay(2000);
        break;
      }
    case RESPONSE_LASER_DETECTED: {
        _zumo.write(RESPONSE_LASER_DETECTED_VALUE);
        break;
      }
    default: {
        Serial.print(F("Unable to send unknown response: "));
        Serial.println(response.type);
        break;
      }
  }
  _zumo.write(ZUMO_END_CHAR);
  Serial.println("Flush is commented out in sendResponse()");
  // _zumo.flush();
  /*if (!waitForOK(500)) {
    Serial.print("Unable to get OK from request: "); Serial.println(request);
    }*/
}

//TODO Is this method working?
/*bool Zumo::waitForOK(unsigned long ms) {
  unsigned long endTime = millis() + ms;
  char buf[32];
  uint8_t counter = 0;
  while (millis() <= endTime) {
    if (_zumo.available()) {
      char c = _zumo.read();
      if (c == ZUMO_END_CHAR) {
        buf[counter] = '\0';
        if (String(buf).equals(ZUMO_OK_RESPONSE))
          return true;
        counter = 0;
      }
      buf[counter++] = c;
      //if(_zumo.indexOf(ZUMO_OK_RESPONSE)
      return true;
    }
  }

  return false;
  }*/


void Zumo::checkForIncoming() {
  while (_zumo.available()) {
    char c = _zumo.read();
    Serial.print(c);
    if (c == ZUMO_END_CHAR) {
      _msgBuffer[_msgBufferCounter] = '\0'; //Tell String where it should stop
      setLastMsg(String(_msgBuffer));
      _msgBufferCounter = 0;
      return;
    }

    //Add the char to the buffer
    _msgBuffer[_msgBufferCounter++] = c;
  }
}

void Zumo::setLastMsg(String msg) {
  Serial.print("Msg: "); Serial.println(String(msg));
  bool isValidRequest = false;
  bool isValidResponse = false;
  Request request;
  Response response;
  switch (msg[0]) {
    /*********************
       Requests
     *********************/
    case REQUEST_IR_ON_VALUE: {
        request.type = REQUEST_IR_ON;
        isValidRequest = true;
        break;
      }
    case REQUEST_IR_OFF_VALUE: {
        request.type = REQUEST_IR_OFF;
        isValidRequest = true;
        break;
      }
    case REQUEST_DRIVE_VALUE: {
        request.type = REQUEST_DRIVE;
        isValidRequest = true;
        break;
      }
    case REQUEST_BEEP_VALUE: {
        request.type = REQUEST_BEEP;
        isValidRequest = true;
        break;
      }
    case REQUEST_HEADING_VALUE: {
        request.type = REQUEST_HEADING;
        isValidRequest = true;
      }


    /*********************
       Responses
     *********************/
    case RESPONSE_IR_LEFT_VALUE: {
        response.type = RESPONSE_IR_LEFT;
        response.valueInt = msg.substring(1).toInt();
        isValidResponse = true;
        break;
      }

    case RESPONSE_IR_FRONT_VALUE: {
        response.type = RESPONSE_IR_FRONT;
        response.valueInt = msg.substring(1).toInt();
        isValidResponse = true;
        break;
      }
    case RESPONSE_IR_RIGHT_VALUE: {
        response.type = RESPONSE_IR_RIGHT;
        response.valueInt = msg.substring(1).toInt();
        isValidResponse = true;
        break;
      }

    case RESPONSE_HEADING_VALUE: {
        response.type = RESPONSE_HEADING;
        response.valueDouble = msg.substring(1).toFloat();
        isValidResponse = true;
        break;
      }


    case RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE: {
        response.type = RESPONSE_COARSE_LOCALIZATION_FINISHED;
        isValidResponse = true;
        break;
      }

    case RESPONSE_ALIGNMENT_FINISHED_VALUE: {
        response.type = RESPONSE_ALIGNMENT_FINISHED;
        isValidResponse = true;
        break;
      }

    case RESPONSE_DOCKING_FINISHED_VALUE: {
        response.type = RESPONSE_DOCKING_FINISHED;
        isValidResponse = true;
        break;
      }
    /*case RESPONSE_IR_FOUND_DIRECTION_VALUE: {
        Response response;
        response.type = RESPONSE_IR_FOUND_DIRECTION;
        _lastResponse = response;
        _hasResponse = true;
        isValidResponse = true;
        delay(1000);
        break;
      }*/
    default: {
        Serial.print(F("Unable to set msg: "));
        Serial.println(msg);
      }

  }

  if (isValidRequest) {
    _lastRequest = request;
    _hasRequest = true;
  } else if (isValidResponse) {
    _lastResponse = response;
    _hasResponse = true;
  }
  /*_zumo.write(ZUMO_OK_RESPONSE);
    _zumo.write(ZUMO_END_CHAR);*/
}

bool Zumo::hasResponse() {
  return _hasResponse;
}
bool Zumo::hasRequest() {
  return _hasRequest;
}

Zumo::Request Zumo::getNextRequest() {
  _hasRequest = false;
  return _lastRequest;
}

Zumo::Response Zumo::getNextResponse() {
  _hasResponse = false;
  return _lastResponse;
}


