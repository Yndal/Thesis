
#ifndef Modules_h
#define Modules_h

#include "Arduino.h"
#include <Encoder.h>

#define MODULE_AMOUNT 2
#define MODULE_TYPES_AMOUNT 8


#define MODULE_PWM_LEFT_PIN    4
#define MODULE_DIR_LEFT_PIN    6
#define MODULE_ENC1_LEFT_PIN  11
#define MODULE_ENC2_LEFT_PIN  12

#define MODULE_PWM_RIGHT_PIN   3
#define MODULE_DIR_RIGHT_PIN   5
#define MODULE_ENC1_RIGHT_PIN 14
#define MODULE_ENC2_RIGHT_PIN 15

#define MODULE_FUNC_1_ANALOG_PIN      A10
#define MODULE_FUNC_1_PWM_PIN          20
#define MODULE_FUNC_1_TYPE_PIN         16
#define MODULE_FUNC_1_STATE_PIN         0

#define MODULE_FUNC_2_ANALOG_PIN      A11
#define MODULE_FUNC_2_PWM_PIN          21
#define MODULE_FUNC_2_TYPE_PIN         17
#define MODULE_FUNC_2_STATE_PIN         1

#define MODULE_INSERTION_SPEED        140
#define MODULE_EJECTION_SPEED         110
//#define MODULE_CHANGE_SPEED           130


#define MODULE_TYPE_VALUE_BATTERY 690//590
#define MODULE_TYPE_VALUE_MOTOR   230 //TESTED
#define MODULE_TYPE_VALUE_LASER   1010//850 //670//520//9250 //967
#define MODULE_TYPE_VALUE_LDR     490                 //TESTED
#define MODULE_TYPE_VALUE_EMPTY   0

#define MODULE_TYPE_VALUE_MAX_DELTA 100



class Modules {
  public:
    enum ModuleType {
      Battery,
      Laser,
      Motor,
      LDR,
      Empty
    };
    /*    enum ModuleChangeAction {
          Insert,
          Eject,
        };*/
    enum ModuleAction {
      Start,
      Stop,
    };

    ModuleType moduleType[MODULE_AMOUNT];
    /*= Empty;
      ModuleType moduleType2 = Empty;*/

    Modules();
    bool checkForChanges();
    bool hasChanged(uint8_t module);
    ModuleType getModuleType(uint8_t module);

    //void requestChangeAction(uint8_t module, ModuleChangeAction action);
    void performModuleAction(uint8_t moduleNumber, ModuleAction action);
    int getAnalogValue(uint8_t module);

    void insertModule(uint8_t module);
    void startEjection(uint8_t module);
    void stopEjection(uint8_t module);

    bool checkModuleStatus();
    int8_t getModuleStatus(uint8_t module);
    int8_t isWorking(uint8_t motor);

    void stopAll();


  private:
    Encoder _encoder1;
    Encoder _encoder2;

    int8_t _moduleStatus[MODULE_AMOUNT];

    void init();
    bool initialized = false;

    //long _lastValue_Encoder1 = 0;
    //long _lastValue_Encoder2 = 0;

    uint16_t moduleTypeValues[MODULE_TYPES_AMOUNT];
    ModuleAction moduleCurrentAction[MODULE_AMOUNT];

    //   uint16_t moduleLastTypeValue[MODULE_AMOUNT];// = 0;
    // uint16_t module2LastTypeValue = 0;

    void setModuleType(uint8_t module, int typeValue);

    volatile bool _motorWorking[MODULE_AMOUNT];

    //    static Modules * _instance;

    //    static void motor1workingISR();
    //    static void motor2workingISR();
    //    ModuleType getModuleType(uint8_t module);
    void motorWorking(uint8_t motor);
    // void insertModule(uint8_t module);
    // void ejectModule(uint8_t module);


};

#endif
