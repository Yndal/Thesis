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

#define WINDOW_LENGTH 40
int window[WINDOW_LENGTH];



void setup()
{
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  //heartbeatTimer.begin(checkForHeartbeat, (ZUMO_HEARTBEAT_INTERVAL_MS + HEARTBEAT_EXTRA_TIME_MS)*1000); //Time in micro (not milli) seconds
  Serial.begin(9600);
  delay(2000);

  for (int i = 0; i < WINDOW_LENGTH; i++)
    window[i] = 0;

  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  Serial.print(F("Robot number: ")); Serial.println(robotNumber);


  digitalWrite(led, LOW);
}






// The main program will print the blink count
// to the Arduino Serial Monitor
unsigned long counter = 0;
long total = 0;
int windowCounter = 0;
void loop()
{

  int value = analogRead(LASER_DETECTION_PIN);
  total += value;
  if (counter++ > AVR_INTERVAL) {
    int avg = total / AVR_INTERVAL;
    window[windowCounter++] = avg;
    if (windowCounter >= WINDOW_LENGTH)
      windowCounter = 0;
    Serial.print(F("Laser avg value: "));
    Serial.print(avg);

    int windowValue = 0;
    for (int i = 0; i < WINDOW_LENGTH; i++)
      if (window[i] > windowValue)
        windowValue = window[i];


    Serial.print(F("\t("));
    Serial.print(windowValue);
    Serial.println(F(")"));

    total = 0;
    counter = 0;
  }
}



