#include <EEPROM.h>

#include <Zumo32U4.h>


/*************************
   IR search related
 *************************/
#define DISTANCE_FRONT_THRESHOLD 3 //Readings

uint16_t brightnessLevels_IROff[] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0};
uint16_t brightnessLevels_IROn[]  = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 49, 55, 65, 75, 85, 120};
//uint16_t brightnessLevels_IROff[] = {0, 0, 0, 0, 0, 0 };
//uint16_t brightnessLevels_IROn[] = {4, 15, 32, 55, 85, 120 };




#define usFrontTrigPin 17
#define usFrontEchoPin 13

#define usLeftTrigPin 12
#define usLeftEchoPin 14

#define usRightTrigPin 2
#define usRightEchoPin 3


//unsigned long lastHeartbeat = 0;
bool _irIsOn = false;



/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!




/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/
#define MIN_DISTANCE_FRONT 8
#define IR_SCAN_TRESHOLD 1
uint8_t lastIRLevel = 0;

/********************************************************************

   TO BE SORTED! TO BE SORTED! TO BE SORTED!

 ********************************************************************/

void setup(void) {
  pinMode(usLeftTrigPin, OUTPUT);
  pinMode(usLeftEchoPin, INPUT);
  pinMode(usFrontTrigPin, OUTPUT);
  pinMode(usFrontEchoPin, INPUT);
  pinMode(usRightTrigPin, OUTPUT);
  pinMode(usRightEchoPin, INPUT);
  proxSensors.initThreeSensors();

  //Timer3.initialize((1/38000)*1000000); //Blink at 38k Hz (when interrupt is attached)

  //_otherHeading = 0.0;
  //_state = Search_Alignment;
  /*#if DEBUG
    Serial.println(F("Zumo stands ready!"));
    #endif*/
  //  disableIREmitters();
}

//uint16_t brightnesses[] = { 4, 15, 32, 55, 85, 120 };
//uint16_t brightnessCount = sizeof(brightnesses) / sizeof(uint16_t);
/*void emitIrSignal() {
  uint8_t brightnessCount = sizeof(brightnessLevels_IROn) / sizeof(uint16_t);
  for (int a = 0; a < 10; a++) {
    for (int i = 0; i < brightnessCount; i++) {
      irPulses.start(Zumo32U4IRPulses::Left, brightnessLevels_IROn[i], Zumo32U4IRPulses::defaultPeriod);
      delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOnTimeUs);
      irPulses.start(Zumo32U4IRPulses::Right, brightnessLevels_IROn[i], Zumo32U4IRPulses::defaultPeriod);
      delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOnTimeUs);
    }
  }
  irPulses.stop();
  delayMicroseconds(Zumo32U4ProximitySensors::defaultPulseOffTimeUs);

  }*/


void turnIrOn() {
#if DEBUG
  //Serial.println(F("Turning IR on"));
#endif
  _irIsOn = true;
  //Timer3.attachInterrupt(blinkIR)

}

void turnIrOff() {
  //Serial.println(F("Turning IR off"));
  _irIsOn = false;
  irPulses.stop();
  //Timer3.detachInterrupt();
}


void disableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROff, sizeof(brightnessLevels_IROff) / sizeof(uint16_t));
}

void enableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROn, sizeof(brightnessLevels_IROn) / sizeof(uint16_t));
}

uint8_t getIRFront(bool emitIR) {
  bool irOn = _irIsOn;
  if (irOn)
    turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  return proxSensors.readBasicFront();

  uint8_t front = proxSensors.countsFrontWithLeftLeds() +
                  proxSensors.countsFrontWithRightLeds();

  if (irOn)
    turnIrOn();

  return front;
}

uint8_t getIRLeft(bool emitIR) {
  bool irOn = _irIsOn;
  turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  return proxSensors.readBasicLeft();

  uint8_t left = proxSensors.countsLeftWithLeftLeds() +
                 proxSensors.countsLeftWithRightLeds();
  if (irOn)
    turnIrOn();

  return left;
}

uint8_t getIRRight(bool emitIR) {
  bool irOn = _irIsOn;
  turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  return proxSensors.readBasicRight();

  uint8_t right = proxSensors.countsRightWithLeftLeds() +
                  proxSensors.countsRightWithRightLeds();
  if (irOn)
    turnIrOn();

  return right;
}

//unsigned long start = millis();
unsigned long time = 10;
void loop(void) {
  int left = 0;
  int front = 0;
  int right = 0;
  unsigned long endTime = time + millis();
  while (millis() < endTime) {
    left += proxSensors.readBasicLeft();
    front += proxSensors.readBasicFront();
    right += proxSensors.readBasicRight();
  }

  Serial.print(left);
  Serial.print('\t');
  Serial.print(front);
  Serial.print('\t');
  Serial.print(right);
  Serial.println();

}

