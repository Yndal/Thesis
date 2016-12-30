#include <FastGPIO.h>
#include <PololuBuzzer.h>
#include <Zumo32U4.h>
//#include <Zumo32U4Buttons.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4ProximitySensors.h>

/**********************************
   Helping functions
 **********************************/
#define DegToRad(x) (x*(PI/180))


//#define HEARTBEAT_CHAR        '*'
//#define HEARTBEAT_INTERVAL_MS 500

#define MAX_SPEED 150 //Max is 400
#define MIN_SPEED 70
#define DELTA_SPEED (MAX_SPEED - MIN_SPEED)
#define SCALE_SPEED 10

/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!

uint8_t _minDist = 0;
double _otherHeading = 0;
#define ROBOT_WIDTH_MM 98
#define ROBOT_LENGTH_MM 86


void setup(void) {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println(F("Zumo stands ready!"));
}


void beep() {
  //Serial.println(F("Beep!"));
  // Start playing a tone with frequency 440 Hz at maximum
  // volume (15) for 200 milliseconds.
  buzzer.playFrequency(440, 200, 15);

  // Delay to give the tone time to finish.
  delay(300);
}


/*
   Can drive 2.2m before the encoders will overflow
*/
void driveMm(double mm, int speed) {
  //Serial.print("Driving mm: "); Serial.println(mm);
  double encoderCPR = 12.0;
  double gearRatio = 150.58;
  double wheelCircumferenceInMm = 39 * PI;
  float tickCountForOneMm =  (encoderCPR * gearRatio) / wheelCircumferenceInMm * -1;
  float totalTicks = mm * tickCountForOneMm;
  totalTicks *= 1.07; //Minor adjustments...

  //Serial.print("Driving ticks: "); Serial.println(totalTicks);
  //TODO Make sure that both wheels drive the exact same distance
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();

  //If speed and distance is going in each direction...
  if (mm * speed < 0)
    speed *= -1;
  drive(speed, speed);
  bool passed = false;
  Serial.println(F("How about using getDistanceFront() here?"));
  Serial.print("Total ticks to drive: ");
  Serial.println(totalTicks);
  if (0 < mm) {
    while (!passed) {
      passed = encoders.getCountsRight() < totalTicks;
    }
  } else {
    while (!passed) {
      passed = totalTicks < encoders.getCountsRight();
    }
  }
  Serial.print("Total ticks driven: ");
  Serial.println(encoders.getCountsRight());
  
  stop();
}


void drive(int left, int right) {
  //Serial.print(F("Drive called: ")); Serial.print(left); Serial.print(F(",")); Serial.println(right);
  //Pre-conditions
  if (SCALE_SPEED < abs(left))
    left = (left / abs(left)) * SCALE_SPEED; //Fancy way to set it to the max
  if (SCALE_SPEED < abs(right))
    right = (right / abs(right)) * SCALE_SPEED; //Fancy way to set it to the max


  int l = left * (DELTA_SPEED / SCALE_SPEED);
  if (l < 0)
    l -= MIN_SPEED;
  else
    l += MIN_SPEED;

  int r = right * (DELTA_SPEED / SCALE_SPEED);
  if (r < 0)
    r -= MIN_SPEED;
  else
    r += MIN_SPEED;
  //int l = MIN_SPEED dd+ left * (DELTA_SPEED/ SCALE_SPEED); // map(left, -SCALE_SPEED, SCALE_SPEED, -MAX_SPEED, MAX_SPEED);
  //int r = right * (DELTA_SPEED/ SCALE_SPEED); //map(right, -SCALE_SPEED, SCALE_SPEED, -MAX_SPEED, MAX_SPEED);

  /*Serial.print("Before conditions: "); Serial.print(l); Serial.print(","); Serial.println(r);
    if (abs(l) < MIN_SPEED)
    l = l < 0 ? -MIN_SPEED : MIN_SPEED;
    if (abs(r) < MIN_SPEED)
    r = r < 0 ? -MIN_SPEED : MIN_SPEED;*/

  if (left == 0)
    l = 0;
  if (right == 0)
    r = 0;
  //Serial.print(F("Driving with: ")); Serial.print(l); Serial.print(F(",")); Serial.println(r);
  motors.setSpeeds(l, r);
}

void stop() {
  motors.setSpeeds(0, 0);
}

char buf[32];
int counter = 0;

void loop(void) {
  while(Serial.available()){
    char c = Serial.read();
    if(c == ';')  {
      buf[counter] = '\0';
      counter = 0;
      double dist = String(buf).toFloat();
      Serial.print("Driving mm: "); Serial.println(dist);
      driveMm(dist,1);
      
    } else {
      buf[counter++] = c;
    }
  }

}

