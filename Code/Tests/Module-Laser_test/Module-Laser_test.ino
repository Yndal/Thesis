#include "Modules.h"


Modules modules = Modules();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(2000);
  Serial.println("Module test started");
}

long limit = 70;
long counter = 0;
bool nextIsStart = true;
bool started = false;
void loop() {
  // put your main code here, to run repeatedly:
  modules.checkForChanges();

  //modules.performModuleAction(2, Modules::Stop);
  //modules.performModuleAction(2, Modules::Start);
 /* if (!started) {
    started = true;
    modules.performModuleAction(2, Modules::Start);
    Serial.println("It is now started");
  }*/

   if (limit < counter++) {
     counter = 0;
     if (nextIsStart) {
       modules.performModuleAction(2, Modules::Start);
       Serial.println("Starting module");
       delay(1000);
     } else {
       modules.performModuleAction(2, Modules::Stop);
       Serial.println("Stopping module");
       delay(1000);
     }
     nextIsStart = !nextIsStart;

    }

  Serial.print("is working: ");
  Serial.println(modules.isWorking(2));
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  Serial.print(counter);
    Serial.print("\t");
    Serial.println(limit);
  delay(50);
}
