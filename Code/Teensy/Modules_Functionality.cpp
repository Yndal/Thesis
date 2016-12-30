#include "Modules_Functionality.h"



void Modules_Functionality::begin ()
{
  /*switch (whichISR_)
    {
    case 0:*/
  attachInterrupt (0, isr0, FALLING);
  attachInterrupt (0, isr0, RISING);
  /*      instance0_ = this;
        break;

      case 1:*/
  attachInterrupt (1, isr1, FALLING);
  attachInterrupt (1, isr1, RISING);
  _instance = this;
  /*      break;
    }*/
}  // end of Foo::begin

// constructor
/*Modules_Functionality::Modules_Functionality (const byte whichISR) : whichISR_ (whichISR)
  {
  counter_ = 0;
  }*/

// ISR glue routines
void Modules_Functionality::isr0 ()
{
  _instance->handleInterrupt ();
}  // end of Foo::isr0

void Modules_Functionality::isr1 ()
{
  _instance->handleInterrupt ();
}  // end of Foo::isr1

// for use by ISR glue routines
Modules_Functionality * Modules_Functionality::_instance;
//Modules_Functionality * Modules_Functionality::instance1_;

// class instance to handle an interrupt
void Modules_Functionality::handleInterrupt ()
{
  Serial.println("handleInterrupt() called");
  counter_++;

}  // end of Foo::handleInterrupt

// instances of class
Modules_Functionality firstFoo;// (0);
//Modules_Functionality secondFoo (1);

/*void setup()
  {
  firstFoo.begin ();
  secondFoo.begin ();
  }  // end of setup
*/


void Modules_Functionality::init() {
  pinMode(MODULE_FUNC_1_IS_WORKING_PIN, INPUT);
  pinMode(MODULE_FUNC_2_IS_WORKING_PIN, INPUT);

  //  attachInterrupt(MODULE_FUNC_1_TYPE_PIN, test, RISING);
  //  attachInterrupt(MODULE_FUNC_1_TYPE_PIN, onNewModuleInserted1, RISING);
}



void Modules_Functionality::onNewModuleInserted(uint8_t moduleNumber) {
  Serial.print("Module insertion detected: ");
  Serial.println(moduleNumber);

  switch (moduleNumber) {
/*    case : {


        break;
      }

    case : {

        break;
      }

    case {

        break;
      }*/
  }
}

void Modules_Functionality::classifyModule(uint8_t module) {
  int value = -1;
  switch (module) {
    case 1: {
value = analogRead(MODULE_FUNC_1_TYPE_PIN);

        break;
      }

    case 2: {
value = analogRead(MODULE_FUNC_2_TYPE_PIN);

        break;
      }
  }

  if(value < 0){
    Serial.print(F("Classification of module requested, but wrong analogValue: "));
    Serial.println(value);
    return;
  }
}

/*void Modules_Functionality::onNewModuleInserted(int number) {
  delay(500); //Allow for module to be completely inserted

  uint16_t value1 = analogRead(MODULE_FUNC_1_ANALOG_PIN);
  uint16_t value2 = analogRead(MODULE_FUNC_2_ANALOG_PIN);

  }*/



