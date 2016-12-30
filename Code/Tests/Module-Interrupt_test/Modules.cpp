#include "Modules.h"



void Modules::init ()
{
  //Init error detection pins
  pinMode(MODULE_FUNC_1_IS_WORKING_PIN, INPUT);
  pinMode(MODULE_FUNC_2_IS_WORKING_PIN, INPUT);

  //Init interrups for exchanging modules
  attachInterrupt (0, module1Updated, CHANGE);
  //attachInterrupt (0, module1Updated, RISING);

  attachInterrupt (1, module2Updated, CHANGE);
  //attachInterrupt (1, isr1, RISING);
  instance0_ = this;

}

// constructor
/*Modules_Functionality::Modules_Functionality (const byte whichISR) : whichISR_ (whichISR)
  {
  counter_ = 0;
  }*/

// ISR glue routines
void Modules::module1Updated ()
{
  instance0_->handleInterrupt (1);
}  // end of Foo::isr0

void Modules::module2Updated ()
{
  //instance1_->handleInterrupt (2);
  instance0_->handleInterrupt (2);
}  // end of Foo::isr1

// for use by ISR glue routines
Modules * Modules::instance0_;
//Modules * Modules::instance1_;

// class instance to handle an interrupt
void Modules::handleInterrupt (int module)
{
  Serial.print("handleInterrupt() called: ");
  Serial.println(module);
  counter++;

}

// instances of class
//Modules firstFoo;// (0);
//Modules_Functionality secondFoo (1);

/*void setup()
  {
  firstFoo.begin ();
  secondFoo.begin ();
  }  // end of setup
*/





void Modules::onNewModuleInserted(uint8_t moduleNumber) {
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

void Modules::classifyModule(uint8_t module) {
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

  if (value < 0) {
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



