#include "Modules.h"

Modules modules;



void setup() {
  modules.init();

}


void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.print("Counter: ");
  Serial.println(modules.counter);

}
