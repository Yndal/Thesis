#include "Teensy.h"


void Teensy::initialize() {
  if (_initialized)
    return;

  _teensy.begin(TEENSY_BAUD);
  Serial.begin(9600);
  delay(1000); //Let it settle

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
        /*Serial.print("IR left ");
          Serial.println(response.value1);*/

        _teensy.write(TEENSY_RESPONSE_IR_LEFT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }

    case TEENSY_RESPONSE_IR_FRONT : {
        /*Serial.print("IR front ");
          Serial.println(response.value1);*/
        _teensy.write(TEENSY_RESPONSE_IR_FRONT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }


    case TEENSY_RESPONSE_IR_RIGHT : {
        /*Serial.print("IR right ");
          Serial.println(response.value1);*/
        _teensy.write(TEENSY_RESPONSE_IR_RIGHT_VALUE);
        _teensy.print(response.value1, DEC);
        break;
      }

    case TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED: {
        /*Serial.println("Coarse loc finished");
          Serial.print("With value: ");
          Serial.println(TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE);*/
        _teensy.write(TEENSY_RESPONSE_COARSE_LOCALIZATION_FINISHED_VALUE);
        break;
      }

    case TEENSY_RESPONSE_ALIGNMENT_FINISHED: {
        _teensy.write(TEENSY_RESPONSE_ALIGNMENT_FINISHED_VALUE);
        break;
      }

    case TEENSY_RESPONSE_DOCKING_FINISHED:{
      _teensy.write(TEENSY_RESPONSE_DOCKING_FINISHED_VALUE);
      break;
    }
      /*  case TEENSY_RESPONSE_IR_FOUND_DIRECTION: {
          Serial.println("Teenst: Found diretion is written");
          delay(1000);
            _teensy.write(TEENSY_RESPONSE_IR_FOUND_DIRECTION_VALUE);
            break;
          }*/
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
    //Serial.print(c);
    if (c == TEENSY_END_CHAR) {
      //Serial.println();
      //Serial.println(F("Teensy - checkForIncomingMsg() - got END_CHAR"));
      _msgBuffer[_msgBufferCounter] = '\0'; //Tell String where it should stop
      _msgBufferCounter = 0;
      String msg = String(_msgBuffer);
      int l = msg.length();
      //Serial.print("String length: "); Serial.println(l);
      if (!msg.equals("")) {
        setLastMsg(msg);
        return true;
      }
    } else {
      //Add the char to the buffer
      _msgBuffer[_msgBufferCounter++] = c;
    }
  }
  return false;
}

void Teensy::setLastMsg(String msg) {
 // Serial.print(F("setLastMsg: ")); Serial.println(msg);
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

        int i = msg.indexOf(TEENSY_SEP_CHAR);
        int c1 = String(msg.substring(1, i)).toInt();
        int c2 = String(msg.substring(i + 1)).toInt();
        // Serial.print(F("Drive - val1: ")); Serial.print(c1); Serial.print(F(", val2: ")); Serial.println(c2);

        request.value1 = c1;
        request.value2 = c2;
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
        request.valueDouble1 = msg.substring(1).toFloat();
        isValidRequest = true;
        break;
      }

    case TEENSY_REQUEST_DOCKING_VALUE: {
        request.type = TEENSY_REQUEST_DOCKING;
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

   case TEENSY_RESPONSE_LASER_DETECTED_VALUE:{
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
    _lastRequest = request;
    _hasNewRequest = true;
  }

  if (isValidResponse) {
    _lastResponse = response;
    _hasNewResponse = true;
  }
}


boolean Teensy::hasNewRequest() {
  return _hasNewRequest;
}


boolean Teensy::hasNewResponse() {
  return _hasNewResponse;
}

Teensy::Request Teensy::getNextRequest() {
  // Serial.println(F("Teensy - getNextMsg()"));
  _hasNewRequest = false;
  return _lastRequest;
}


Teensy::Response Teensy::getNextResponse() {
  // Serial.println(F("Teensy - getNextMsg()"));
  _hasNewResponse = false;
  return _lastResponse;
}





