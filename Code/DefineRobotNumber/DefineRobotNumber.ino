#include <EEPROM.h>


#define EEPROM_ADDRESS 0
unsigned int robotNumber = 2;

void setup(){

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  EEPROM.put( EEPROM_ADDRESS, robotNumber );

  Serial.println("Robot number saved!");

}

void loop(){ /* Empty loop */ }
