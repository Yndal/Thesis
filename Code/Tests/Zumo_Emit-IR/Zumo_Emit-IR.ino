#include <FastGPIO.h>
/*#include <L3G.h>
#include <LSM303.h>
#include <PololuBuzzer.h>
#include <PololuHD44780.h>
#include <QTRSensors.h>*/
#include <USBPause.h>
#include <Zumo32U4.h>
#include <Zumo32U4LineSensors.h>

#include <Zumo32U4ProximitySensors.h>

#include <Zumo32U4.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Buzzer.h>

Zumo32U4Buzzer buzzer;
uint16_t hz = 200;
uint16_t dur;
uint16_t vol = 9;

Zumo32U4IRPulses irPulses;
uint16_t brightnesses[] = { 4, 15, 32, 55, 85, 120 };
uint16_t brightnessCount = 6;
  


void setup() {
  
}
int pulseOnTimeUs = Zumo32U4ProximitySensors::defaultPulseOnTimeUs;
void loop() {
  for(int i=0; i<brightnessCount; i++){
    irPulses.start(Zumo32U4IRPulses::Left, brightnesses[i], Zumo32U4IRPulses::defaultPeriod);
    //delayMicroseconds(pulseOnTimeUs);
    irPulses.start(Zumo32U4IRPulses::Right, brightnesses[i], Zumo32U4IRPulses::defaultPeriod);
    //delayMicroseconds(pulseOnTimeUs);
    
  }
}
