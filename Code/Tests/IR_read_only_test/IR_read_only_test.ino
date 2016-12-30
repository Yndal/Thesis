#include <PololuBuzzer.h>
//#include <Zumo32U4.h>
//#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4ProximitySensors.h>



#define PULSE_AMOUNT 6
#define PULSE_INTENSITY 0
#define LED_PIN 13

uint16_t defaultBrightnessLevels[PULSE_AMOUNT];

Zumo32U4ProximitySensors proxSensors;

uint8_t frontLeftValue;
uint8_t frontRightValue;

uint8_t leftLeftValue;
uint8_t leftRightValue;

uint8_t rightLeftValue;
uint8_t rightRightValue;

void setup(void) {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  proxSensors.initThreeSensors();

  for (int i = 0; i < PULSE_AMOUNT; i++)
    defaultBrightnessLevels[i] = PULSE_INTENSITY;


  //uint16_t defaultBrightnessLevels[] = {4, 15, 32, 55, 85, 120 };
  //uint16_t defaultBrightnessLevels[] = {0,0,0,0,0,0};//,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  proxSensors.setBrightnessLevels(defaultBrightnessLevels, PULSE_AMOUNT);//24);//defaultBrightnessLevels, 6);
  delay(1000);
}


void loop(void) {
  //turnStuffOff();
  readValues();
  collectValues();
  printValues();
  delay(50);
}

void turnStuffOff() {
  proxSensors.pullupsOn();
  proxSensors.lineSensorEmittersOff();
}

void readValues() {
  //proxSensors.read();

  /*prepare();

    proxSensors.readBasicLeft();
    proxSensors.readBasicFront();
    proxSensors.readBasicRight();
  */
  proxSensors.read();

}

void prepare() {
  proxSensors.pullupsOn();
  proxSensors.lineSensorEmittersOff();
}

void collectValues() {
  frontLeftValue = proxSensors.countsFrontWithLeftLeds();
  frontRightValue = proxSensors.countsFrontWithRightLeds();

  leftLeftValue = proxSensors.countsLeftWithLeftLeds();
  leftRightValue = proxSensors.countsLeftWithRightLeds();

  rightLeftValue = proxSensors.countsRightWithLeftLeds();
  rightRightValue = proxSensors.countsRightWithRightLeds();

}

int getSum() {
  int sum = frontLeftValue +
            frontRightValue +
            leftLeftValue +
            leftRightValue +
            rightLeftValue +
            rightRightValue;

  return sum;
}


void printValues() {
  Serial.print(leftLeftValue);
  Serial.print("\t");
  Serial.print(leftRightValue);
  Serial.print("\t");

  Serial.print(frontLeftValue);
  Serial.print("\t");
  Serial.print(frontRightValue);

  Serial.print("\t");
  Serial.print(rightLeftValue);
  Serial.print("\t");
  Serial.print(rightRightValue);

  int sum = getSum();
  Serial.print("\tSum:");
  Serial.println(sum);

  digitalWrite(LED_PIN, sum);
  
}

