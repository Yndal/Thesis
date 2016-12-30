//#include <TimerOne.h>
#include <EEPROM.h>

/*************************************
   Docking related
 *************************************/
#define LASER_DETECTION_PIN  A8
#define LASER_PIN            23


#define EEPROM_ADDRESS 0
uint8_t robotNumber = 0;

const uint8_t led = 13;  // the pin with a LED
bool ledState = LOW;

#define AVR_INTERVAL 10000



void setup()
{
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  //heartbeatTimer.begin(checkForHeartbeat, (ZUMO_HEARTBEAT_INTERVAL_MS + HEARTBEAT_EXTRA_TIME_MS)*1000); //Time in micro (not milli) seconds
  Serial.begin(9600);
  delay(2000);


  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  Serial.print(F("Robot number: ")); Serial.println(robotNumber);
  
  
  digitalWrite(LASER_PIN, HIGH);
  Serial.println("Laser is on!");

  digitalWrite(led, LOW);
}
void loop()
{

}



