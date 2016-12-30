#include "Modules.h"


Modules::Modules()/* :
  _encoder1(MODULE_ENC1_LEFT_PIN,  MODULE_ENC2_LEFT_PIN),
  _encoder2(MODULE_ENC1_RIGHT_PIN, MODULE_ENC2_RIGHT_PIN) */{

}

void Modules::init() {
  if (initialized)
    return;
  initialized = true;

  pinMode(MODULE_PWM_LEFT_PIN,  OUTPUT);
  pinMode(MODULE_DIR_LEFT_PIN,  OUTPUT);
  pinMode(MODULE_PWM_RIGHT_PIN, OUTPUT);
  pinMode(MODULE_DIR_RIGHT_PIN, OUTPUT);

  /* Serial.print("Init - current action BEFORE: ");
    Serial.println(moduleCurrentAction[1]);
    Serial.print("Init - Stop value: ");
    Serial.println(Stop); */

  for (int i = 0; i < MODULE_AMOUNT; i++) {
    _motorWorking[i] = false;
    setModuleType(i + 1, Empty); //Will define the correct pins!
    moduleCurrentAction[i] = Stop;
  }

  /*Serial.print("Init - current action After: ");
    Serial.println(moduleCurrentAction[1]); */

  // Serial.print("before value: ");
  // Serial.println(moduleTypeValues[Laser]);
  //Serial.println(moduleTypeValues[moduleType[1]]);

  moduleTypeValues[Battery] = MODULE_TYPE_VALUE_BATTERY;
  moduleTypeValues[Laser] = MODULE_TYPE_VALUE_LASER;
  moduleTypeValues[Motor] = MODULE_TYPE_VALUE_MOTOR;
  moduleTypeValues[Empty] = MODULE_TYPE_VALUE_EMPTY;

  /*  Serial.print("after value1: ");
    Serial.println(moduleTypeValues[moduleType[1]]);
    Serial.print("after value2: ");
    Serial.println(moduleTypeValues[Laser]);*/

  pinMode(MODULE_FUNC_1_TYPE_PIN, INPUT);
  pinMode(MODULE_FUNC_2_TYPE_PIN, INPUT);
  pinMode(MODULE_FUNC_1_STATE_PIN, INPUT);
  pinMode(MODULE_FUNC_2_STATE_PIN, INPUT);

}



bool Modules::checkForChanges() {
  init();
  bool changed = false;

  /*uint16_t val1 = analogRead(MODULE_FUNC_1_TYPE_PIN);
    if (abs(val1 - module1LastTypeValue) > MODULE_TYPE_VALUE_MAX_DELTA) {
     val1 = 0;
     uint8_t times = 10;
     for (int i = 0; i < times; i++) {
       val1 += analogRead(MODULE_FUNC_1_TYPE_PIN);
       delay(10);
     }
     val1 /= times;
    //  Serial.print("val1: ");
    // Serial.println(val1);

     if (abs(val1 - module1LastTypeValue) > MODULE_TYPE_VALUE_MAX_DELTA) {
       //Module 1 changed
       Serial.println("Modules::Module 1 has changed");
       module1LastTypeValue = val1;
       changed = true;
       setModuleType(1, val1);
     }
    }*/

  int32_t val2 = analogRead(MODULE_FUNC_2_TYPE_PIN);
  Serial.print("value: ");
  Serial.println(val2);
  /* Serial.print("other value: ");
    Serial.println(moduleTypeValues[moduleType[1]]);*/
  if (abs(val2 - moduleTypeValues[moduleType[1]]) > MODULE_TYPE_VALUE_MAX_DELTA) {

    // if (abs(val2 - moduleLastTypeValue[1]) > MODULE_TYPE_VALUE_MAX_DELTA) {
    val2 = 0;
    uint8_t times = 10;
    for (int i = 0; i < times; i++) {
      val2 += analogRead(MODULE_FUNC_2_TYPE_PIN);
      delay(10);
    }
    val2 /= times;
    /* Serial.print("val2: ");
      Serial.println(val2);
      Serial.print("other value: ");
      Serial.println(moduleTypeValues[moduleType[1]]);
      Serial.print("Type: ");
      Serial.println(moduleType[1]);
      Serial.print("delta value: ");
       Serial.println(MODULE_TYPE_VALUE_MAX_DELTA);
       Serial.print("abs value: ");
       Serial.println(abs(val2 - moduleTypeValues[moduleType[1]]));*/

    if (abs(val2 - moduleTypeValues[moduleType[1]]) > MODULE_TYPE_VALUE_MAX_DELTA) {
      //Module 2 changed
      Serial.print("Modules::Module 2 has changed - value is ");
      Serial.println(val2);
      changed = true;
      setModuleType(2, val2);
    }
  }

  return changed;
}

void Modules::setModuleType(uint8_t module, int typeValue) {
  init();
  ModuleType type = Empty;


  if (abs(typeValue - MODULE_TYPE_VALUE_BATTERY) < MODULE_TYPE_VALUE_MAX_DELTA) {
    type = Battery;
    Serial.println("Set as Battery");
  } else if (abs(typeValue - MODULE_TYPE_VALUE_MOTOR) < MODULE_TYPE_VALUE_MAX_DELTA) {
    type = Motor;
    Serial.println("Set as Motor");
  } else if (abs(typeValue - MODULE_TYPE_VALUE_LASER) < MODULE_TYPE_VALUE_MAX_DELTA) {
    type = Laser;
    Serial.println("Set as Laser");
  } else if (abs(typeValue - MODULE_TYPE_VALUE_EMPTY) < MODULE_TYPE_VALUE_MAX_DELTA) {
    type = Empty;
    Serial.println("Set as Empty");
  } else {
    type = Empty;
    //Serial.println("Unable to determine module - will be set to Empty");
    // Serial.print("Value is "); Serial.println(typeValue);
  }

  // Serial.print("Type: ");
  // Serial.println(type);

  moduleType[module - 1] = type;
  // Serial.print("Set type: ");
  // Serial.println(moduleType[module - 1]);

  //Set the correct IOs
  switch (type) {
    case Battery: {
        switch (module) {
          case 1: {
              /*    pinMode(MODULE_FUNC_1_ANALOG_PIN, INPUT);
                  pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
                  pinMode(MODULE_FUNC_1_STATE, INPUT);*/
              break;
            }

          case 2: {
              /*pinMode(MODULE_FUNC_2_ANALOG_PIN, INPUT);
                pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
                pinMode(MODULE_FUNC_2_STATE_PIN, INPUT);*/
              break;
            }
        }
        break;
      }
    case Laser: {
        switch (module) {
          case 1: {
              pinMode(MODULE_FUNC_1_ANALOG_PIN, INPUT);
              pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
              pinMode(MODULE_FUNC_1_STATE_PIN, INPUT);
              break;
            }

          case 2: {
              pinMode(MODULE_FUNC_2_ANALOG_PIN, INPUT);
              pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
              pinMode(MODULE_FUNC_2_STATE_PIN, INPUT);
              break;
            }
        }
        break;
      }

    case Motor: {
        switch (module) {
          case 1: {
              pinMode(MODULE_FUNC_1_ANALOG_PIN, OUTPUT);
              pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
              pinMode(MODULE_FUNC_1_STATE_PIN, INPUT);
              break;
            }

          case 2: {
              pinMode(MODULE_FUNC_2_ANALOG_PIN, OUTPUT);
              pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
              pinMode(MODULE_FUNC_2_STATE_PIN, INPUT);
              break;
            }
        }
        break;
      }

    case Empty: {
        //Nothing
        break;
      }
  }

  //Stop the newly inserted module (just a safety feature)
  performModuleAction(module, Stop);
}

void Modules::requestChangeAction(uint8_t module, ModuleChangeAction action) {
  init();
  switch (action) {
    case (Insert):
      insertModule(module);
      break;

    case (Eject):
      ejectModule(module);
      break;
  }
}

void Modules::insertModule(uint8_t module) {
  init();
  switch (module) {
    case 1: {
        //Start motor

        //Check for changes

        //If new change is in module <module>
        //Done
        //else
        //Repeat

        bool done = true;//false;
        do {



          if (checkForChanges()) {
            if (getModuleType(1) != moduleType[0]) {
              //Change is done
            }
          }


          //          done = !checkForChanges() && correctModule;
        } while (!done);

        break;
      }

    case 2: {

        break;
      }

  }
  //Set direction of motor
  //Set speed of motor
  //Run for max XXX seconds OR when connected is detected
}


void Modules::ejectModule(uint8_t module) {
  init();
}


void Modules::performModuleAction(uint8_t moduleNumber, ModuleAction action) {
  init();
  //TODO Have a second look at the switch here
  /* Serial.print("Perform action - moduleNumber: ");
    Serial.println(moduleNumber);
    Serial.print("Perform action - action: ");
    Serial.println(action);

    Serial.print("Perform action - current action BEFORE: ");
    Serial.println(moduleCurrentAction[moduleNumber - 1]); */
  moduleCurrentAction[moduleNumber - 1] = action;
  /*  Serial.print("Perform action - current action AFTER: ");
    Serial.println(moduleCurrentAction[moduleNumber - 1]); */

  switch (moduleNumber) {
    case 1: {
        Serial.println("Starting action for module 1");
        switch (action) {
          case Start: {
              switch (moduleType[0]) {
                case Battery: {
                    //Measure the voltage...
                    uint16_t voltage = analogRead(MODULE_FUNC_1_ANALOG_PIN);
                    //..and if is okay...
                    if (1000 < voltage) {
                      //..then turn on the relay
                      pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
                      digitalWrite(MODULE_FUNC_1_PWM_PIN, HIGH);
                    }
                    break;
                  }

                case Laser: {
                    pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
                    digitalWrite(MODULE_FUNC_1_PWM_PIN, HIGH);
                    Serial.println("Laser started");
                    break;
                  }

                case Motor: {
                    //Direction
                    pinMode(MODULE_FUNC_1_ANALOG_PIN, OUTPUT);
                    digitalWrite(MODULE_FUNC_1_ANALOG_PIN, 1);

                    //Power
                    pinMode(MODULE_FUNC_1_PWM_PIN, OUTPUT);
                    analogWrite(MODULE_FUNC_1_PWM_PIN, 255);

                    pinMode(MODULE_FUNC_1_STATE_PIN, INPUT);
                    //                    attachInterrupt(digitalPinToInterrupt(MODULE_FUNC_1_STATE_PIN), motor1workingISR , RISING);

                    break;
                  }

                case Empty: {
                    //Do nothing
                    break;
                  }
              }
              break;
            }

          case Stop: {
              switch (moduleType[0]) {
                case Battery: {
                    //Turn off the relay
                    digitalWrite(MODULE_FUNC_1_PWM_PIN, LOW);

                    break;
                  }

                case Laser: {
                    digitalWrite(MODULE_FUNC_1_PWM_PIN, LOW);
                    Serial.println("Laser stopped");
                    break;
                  }

                case Motor: {
                    //Power
                    digitalWrite(MODULE_FUNC_1_PWM_PIN, 0);

                    //Stop listening for correct performance
                    detachInterrupt(MODULE_FUNC_1_STATE_PIN);

                    break;
                  }

                case Empty: {
                    //Do nothing
                    break;
                  }

              }
              break;
            }
        }
        break;
      }

    case 2: {
        // Serial.print("Case 2!!");
        switch (action) {
          case Start: {
              //Serial.println("Starting");
              switch (moduleType[1]) {
                case Battery: {
                    //Measure the voltage...
                    uint16_t voltage = analogRead(MODULE_FUNC_2_ANALOG_PIN);
                    //..and if is okay...
                    if (1000 < voltage) {
                      //..then turn on the relay
                      //            pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
                      digitalWrite(MODULE_FUNC_2_PWM_PIN, HIGH);
                    }
                    break;
                  }

                case Laser: {
                    //                   pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
                    digitalWrite(MODULE_FUNC_2_PWM_PIN, HIGH);
                    break;
                  }

                case Motor: {
                    //Direction
                    // pinMode(MODULE_FUNC_2_ANALOG_PIN, OUTPUT);
                    digitalWrite(MODULE_FUNC_2_ANALOG_PIN, 1);

                    //Power
                    //pinMode(MODULE_FUNC_2_PWM_PIN, OUTPUT);
                    analogWrite(MODULE_FUNC_2_PWM_PIN, 150);

                    // pinMode(MODULE_FUNC_2_IS_WORKING_PIN, INPUT);
                    //                    attachInterrupt(digitalPinToInterrupt(MODULE_FUNC_2_IS_WORKING_PIN), motor2workingISR , RISING);

                    break;
                  }

                case Empty: {
                    //Do nothing
                    break;
                  }
              }
              break;
            }

          case Stop: {
              // Serial.println("Stopping");
              switch (moduleType[1]) {
                case Battery: {
                    //Turn off the relay
                    digitalWrite(MODULE_FUNC_2_PWM_PIN, LOW);

                    break;
                  }

                case Laser: {
                    digitalWrite(MODULE_FUNC_2_PWM_PIN, LOW);
                    break;
                  }

                case Motor: {
                    //Power
                    analogWrite(MODULE_FUNC_2_PWM_PIN, 0);
                    //Stop listening for correct performance
                    //detachInterrupt(MODULE_FUNC_2_IS_WORKING_PIN);

                    break;
                  }

                case Empty: {
                    Serial.println("Unknown module type: Can't stop it!");
                    //Do nothing
                    break;
                  }

              }
              break;
            }
        }
        break;
      }
  }
}




int8_t Modules::isWorking(uint8_t module) {
  init();
  int8_t result = -1;
  if (module > MODULE_AMOUNT)
    return result;

  bool isOff = false;
  if (moduleCurrentAction[module - 1] == Stop) {
    //Serial.println("isWorking(): Module was stopped");
    isOff = true;
    performModuleAction(module, Start);
    delay(1); //Allow cap to charge
    //return 0;
  }



  switch (moduleType[module - 1]) {
    case Battery: {

        break;
      }

    case Laser: {
        switch (module) {
          case 1: {
              int value = digitalRead(MODULE_FUNC_1_STATE_PIN);
              Serial.print("is on: ");
              Serial.println(value);
              break;
            }

          case 2: {
              int value = digitalRead(MODULE_FUNC_2_STATE_PIN);
              // Serial.print("value working: ");
              // Serial.println(value);
              if (value == LOW)
                result = 1;
              if (value == HIGH)
                result = -1;
              break;
            }
        }
        break;
      }

    case Motor: {
          int value = digitalRead(MODULE_FUNC_2_STATE_PIN);// digitalRead(MODULE_FUNC_2_IS_WORKING_PIN);
          // Serial.print("value working: ");
          // Serial.println(value);
          if (value == HIGH)
          result = 1;
          if (value == LOW)
          result = -1;
        break;
      }

    case Empty: {
        return true;
        break;
      }
  }

  if (isOff)
    performModuleAction(module, Stop);

  return result;
  /*if (motor == 1)
    _motor1working = true;
    else if (motor == 2)
    _motor2working = true;*/

}


