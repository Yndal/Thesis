#include "Modules.h"


Modules modules = Modules();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);
  Serial.println("Module test started");
}


bool isRunning_1 = false;
bool isRunning_2 = false;

unsigned long runTime_1 = 5000; //Ms
unsigned long stopTime_1 = 2000; //Ms
unsigned nextSwitch_1 = 0;

unsigned long runTime_2 = 5000; //Ms
unsigned long stopTime_2 = 2000; //Ms
unsigned nextSwitch_2 = 0;

void loop() {
  // put your main code here, to run repeatedly:
  modules.checkForChanges();

  testAndPrint(1);
  Serial.print("\t\t");
  //delay(50);
  testAndPrint(2);
  delay(100);
  Serial.println();


  if (isRunning_1) {
    if (nextSwitch_1 <= millis()) {
      modules.performModuleAction(1, Modules::Stop);
      isRunning_1 = false;
      nextSwitch_1 = stopTime_1 + millis();
    }
  } else {
    if (nextSwitch_1 <= millis()) {
      modules.performModuleAction(1, Modules::Start);
      isRunning_1 = true;
      nextSwitch_1 = runTime_1 + millis();
    }
  }

  if (isRunning_2) {
    if (nextSwitch_2 <= millis()) {
     // Serial.println("Stopping...");
      modules.performModuleAction(2, Modules::Stop);
      isRunning_2 = false;
      nextSwitch_2 = stopTime_2 + millis();
    }
  } else {
    if (nextSwitch_2 <= millis()) {
      //Serial.println("Starting...");
      modules.performModuleAction(2, Modules::Start);
      isRunning_2 = true;
      nextSwitch_2 = runTime_2 + millis();
    }
  }
  delay(50);
}

void testAndPrint(uint8_t module) {
  Modules::ModuleType type;
  type = modules.getModuleType(module);
  int isRunning = module == 1 ? isRunning_1 : isRunning_2;
  int state = modules.isWorking(module);

  String typeStr;
  switch (type) {
    case Modules::Battery: {
        typeStr = "Battery";
        break;
      }
    case Modules::Motor: {
        typeStr = "Motor";
        break;
      }
    case Modules::LDR: {
        typeStr = "LDR";
        break;
      }
    case Modules::Laser: {
        typeStr = "Laser";
        break;
      }
    case Modules::Empty: {
        typeStr = "Empty";
        break;
      }


  }
  Serial.print(module);
  Serial.print(": ");
  Serial.print(typeStr);
  Serial.print(", isRunning: ");
  Serial.print(isRunning ? "Yes" : "No");
  Serial.print(", State: ");
  Serial.print(state == 1 ? "Good" : state == -1 ? "Bad" : "Unknown");

  //If is a sensor, then print value
  if (type == Modules::LDR) {
    int value = modules.getAnalogValue(module);
    Serial.print(", Sensor value: ");
    Serial.print(value);
  }
  Serial.print("  ");
}

