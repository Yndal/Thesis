#include <EEPROM.h>
#include <FastGPIO.h>
#include <Zumo32U4.h>


#define MAX_SPEED 250 //Max is 400
#define MIN_SPEED 65
#define DELTA_SPEED (MAX_SPEED - MIN_SPEED)
#define SCALE_SPEED 10

/*************************
   IR search related
 *************************/
#define DISTANCE_FRONT_THRESHOLD 3 //Readings


#define usFrontTrigPin 17
#define usFrontEchoPin 13

#define usLeftTrigPin 12
#define usLeftEchoPin 14

#define usRightTrigPin 2
#define usRightEchoPin 3

enum DistanceDirection {
  Left, Front, Right
};



/*****************************
   Classes
 *****************************/
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
//Zumo32U4IRPulses irPulses;
Zumo32U4ProximitySensors proxSensors; //Make interfere with irPulses!
//LSM303 compass;


#define MIN_DISTANCE_FRONT 8

#define ROBOT_WIDTH_MM 98
#define ROBOT_LENGTH_MM 86


//Drive XXX mm away from the robot
#define ALIGNMENT_SAFETY_DISTANCE  15 //cm


void setup(void) {
  pinMode(usLeftTrigPin, OUTPUT);
  pinMode(usLeftEchoPin, INPUT);
  pinMode(usFrontTrigPin, OUTPUT);
  pinMode(usFrontEchoPin, INPUT);
  pinMode(usRightTrigPin, OUTPUT);
  pinMode(usRightEchoPin, INPUT);


  Serial.begin(9600);
 // while (!Serial);
 delay(5000);
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
   Can drive 2.2 meters before the encoders will overflow
*/
void driveMm(double mm, int speed) {
  float tickCountForOneMm =  (12 * 150.58) / (39 * PI) * -1;
  float totalTicks = mm * tickCountForOneMm;
  totalTicks *= 1.07; //Minor adjustments...

  //If speed and distance is going in each direction...
  if (mm * speed < 0)
    speed *= -1;
  drive(speed, speed);
  bool passed = false;
  bool stalling = false;
  int stallInterval = 1000;
  long lastStallCheckTicksLeft = encoders.getCountsAndResetLeft();
  long lastStallCheckTicksRight = encoders.getCountsAndResetRight();
  delay(50); //Drive a bit before checking...
  long nextStallingCheck = millis() + stallInterval;
  //Serial.println(F("How about using getDistanceFront() here?"));

  if (0 < mm) {
    while (!passed && !stalling) {
      passed = encoders.getCountsRight() < totalTicks;

      if (nextStallingCheck < millis()) {
        if ((encoders.getCountsRight() == lastStallCheckTicksRight) ||
            (encoders.getCountsLeft() == lastStallCheckTicksLeft)) {
          stalling = true;
        }
        lastStallCheckTicksRight = encoders.getCountsRight();
        lastStallCheckTicksLeft = encoders.getCountsLeft();
        nextStallingCheck = millis() + stallInterval;
      }
    }
  } else {
    while (!passed && !stalling) {
      passed = totalTicks < encoders.getCountsRight();
      if (nextStallingCheck < millis()) {
        if ((encoders.getCountsRight() == lastStallCheckTicksRight) ||
            (encoders.getCountsLeft() == lastStallCheckTicksLeft))
          stalling = true;
        lastStallCheckTicksRight = encoders.getCountsRight();
        lastStallCheckTicksLeft = encoders.getCountsLeft();
        nextStallingCheck = millis() + stallInterval;
      }
    }
  }
  stop();
}


void drive(int left, int right) {
  Serial.print(F("Drive called: ")); Serial.print(left); Serial.print(F(",")); Serial.println(right);
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

  if (left == 0)
    l = 0;
  if (right == 0)
    r = 0;
  Serial.print(F("Driving with: ")); Serial.print(l); Serial.print(F(",")); Serial.println(r);
  motors.setSpeeds(l, r);
}

void stop() {
  motors.setSpeeds(0, 0);
}

double getDistance(DistanceDirection dir, int times) {
  int usTrigPin;
  int usEchoPin;
  switch (dir) {
    case Left: {
        usTrigPin = usLeftTrigPin;
        usEchoPin = usLeftEchoPin;
        break;
      }
    case Front: {
        usTrigPin = usFrontTrigPin;
        usEchoPin = usFrontEchoPin;
        break;
      }
    case Right: {
        usTrigPin = usRightTrigPin;
        usEchoPin = usRightEchoPin;
        break;
      }
  }


  double duration = 0,
         distance = 0;

  for (int i = 0; i < times; i++) {
    delay(10);
    digitalWrite(usTrigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(usTrigPin, HIGH);
    //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(usTrigPin, LOW);
    duration = pulseIn(usEchoPin, HIGH);
    //distance += (duration / 2) / 29.1;
    distance += ((duration / 2) / 29.1) / times;
    delay(10);
  }
  //distance /= times;

  return distance;
}

double getDistance(DistanceDirection dir) {
  return getDistance(dir, 5);
}


//unsigned long start = millis();
void loop(void) {
  //Allow complete halt to prevent noisy readings
  beep();


  bool keepDriving = true;
  bool isCorrectingDecreaseDist = false;
  bool isCorrectingIncreaseDist = false;
  unsigned long lastSwitch = millis();
  drive(2, 2);
  DistanceDirection sensor = Right;

  while (keepDriving) {
    double dist = getDistance(sensor);
    Serial.print("Dist: ");
    Serial.print(dist);
    if (dist < ALIGNMENT_SAFETY_DISTANCE && !isCorrectingIncreaseDist) {
      Serial.print(" - Increasing");
      lastSwitch = millis();
      if (sensor == Left)
        drive(3, 0);
      else
        drive(0, 3);
      isCorrectingIncreaseDist = true;
      isCorrectingDecreaseDist = false;
    } else if (dist > ALIGNMENT_SAFETY_DISTANCE && !isCorrectingDecreaseDist) {
      Serial.print(" - Decreasing");
      lastSwitch = millis();
      if (sensor == Left)
        drive(0, 3);
      else
        drive(3, 0);
      isCorrectingIncreaseDist = false;
      isCorrectingDecreaseDist = true;
    }
    if (lastSwitch + 1500 < millis()) {
      drive(1, 1);
      isCorrectingIncreaseDist = false;
      isCorrectingDecreaseDist = false;
      delay(2000);
      continue;
    }
    Serial.println();
  }




  //  driveWithDistance(sensor, distance);
  //   driveWithDistance(Right, SAFETY_DISTANCE);
  //   turnToHeading(awayH, turnDir);
  //   driveMm(80, 1);


}



//          Back up until distance right / left is more than SAFETY_DISTANCE + XXXX

//          Move xxxxx mm plus half of robot's width forward





