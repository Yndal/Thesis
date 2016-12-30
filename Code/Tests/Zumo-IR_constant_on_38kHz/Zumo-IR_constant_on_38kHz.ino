#include <TimerThree.h>


/******************************
 * Vil udsende 38kHz moduleret IR fra begge sider ved fuld styrke
 * (Når den ene side er tændt, vil den anden være slukket)
 ******************************/




#define PWM 5
#define LED_SELECT 19



bool left = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM, OUTPUT);
  pinMode(LED_SELECT, OUTPUT);

  Timer3.initialize((1/(38000*2))*1000000); //run twice every 1/38000 second
  Timer3.attachInterrupt(blink);
  digitalWrite(PWM, HIGH);
}

void blink(){
   digitalWrite(LED_SELECT, left);
   left = !left;
}

void loop() {
   // put your main code here, to run repeatedly:
}
