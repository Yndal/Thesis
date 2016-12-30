#include <Zumo32U4.h>


Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!

bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  proxSensors.initThreeSensors();
}

bool on = true;
void loop() {
  // put your main code here, to run repeatedly:
  int l = getIRLeft(on);
  int c = getIRFront(on);
  int r = getIRRight(on);

  Serial.print(l);
  Serial.print("\t");
  Serial.print(c);
  Serial.print("\t");
  Serial.print(r);
  Serial.println();

  delay(100);
}





bool _irIsOn = false;
uint16_t brightnessLevels_IROff[] = {0, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0};
uint16_t brightnessLevels_IROn[]  = {4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 49, 55, 65, 75, 85, 120};

void disableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROff, sizeof(brightnessLevels_IROff) / 2);
}

void enableIREmitters() {
  proxSensors.setBrightnessLevels(brightnessLevels_IROn, sizeof(brightnessLevels_IROn) / 2);
}

uint16_t getIRFront(bool emitIR) {
  bool irOn = _irIsOn;
  if (irOn)
    turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();

  uint16_t front = proxSensors.countsFrontWithLeftLeds() +
                   proxSensors.countsFrontWithRightLeds();

  if (irOn)
    turnIrOn();

  return front;
}

uint16_t getIRLeft(bool emitIR) {
  bool irOn = _irIsOn;
  turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();

  uint16_t left = proxSensors.countsLeftWithLeftLeds() +
                  proxSensors.countsLeftWithRightLeds();
  if (irOn)
    turnIrOn();

  return left;
}

uint16_t getIRRight(bool emitIR) {
  bool irOn = _irIsOn;
  turnIrOff();

  if (emitIR)
    enableIREmitters();
  else
    disableIREmitters();
  proxSensors.read();

  uint16_t right = proxSensors.countsRightWithLeftLeds() +
                   proxSensors.countsRightWithRightLeds();
  if (irOn)
    turnIrOn();

  return right;
}


void turnIrOn() {
  //Serial.println(F("Turning IR on"));
  _irIsOn = true;
  //Timer3.attachInterrupt(blinkIR)

}

void turnIrOff() {
  //Serial.println(F("Turning IR off"));
  _irIsOn = false;
  irPulses.stop();
  //Timer3.detachInterrupt();
}

