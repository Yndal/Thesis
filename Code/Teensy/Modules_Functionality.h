
#ifndef Modules_Functionality_h
#define Modules_Functionality_h

#include "Arduino.h"
#include <Encoder.h>

#define MODULE_FUNC_1_ANALOG_PIN A7 //TODO
#define MODULE_FUNC_1_PWM_PIN 1 //TODO
#define MODULE_FUNC_1_TYPE_PIN A6 //TODO
#define MODULE_FUNC_1_IS_WORKING_PIN A4 //TODO

#define MODULE_FUNC_2_ANALOG_PIN A7 //TODO
#define MODULE_FUNC_2_PWM_PIN 1 //TODO
#define MODULE_FUNC_2_TYPE_PIN A6 //TODO
#define MODULE_FUNC_2_IS_WORKING_PIN A4 //TODO

#define MODULE_FUN_TYPE_BATTERY_VALUE 512
#define MODULE_FUN_TYPE_LASER_VALUE   512
#define MODULE_FUN_TYPE_MOTOR_VALUE   512
#define MODULE_FUN_TYPE_MAX_DELTA      10

class Modules_Functionality {

  public:
    void init();
    void onNewModuleInserted(uint8_t moduleNumber);
    //    Modules_Functionality (const byte which);
    void begin ();


  private:
    static void isr0 ();
    static void isr1 ();

    //const byte whichISR_;
    static Modules_Functionality * _instance;
    //static Modules_Functionality * instance1_;

    void handleInterrupt ();

    volatile int counter_;


    void classifyModule(uint8_t module);

};

#endif
