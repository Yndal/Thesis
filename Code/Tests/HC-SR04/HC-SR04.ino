/*
  HC-SR04 Ping distance sensor]
  VCC to arduino 5v GND to arduino GND
  Echo to Arduino pin 13 Trig to Arduino pin 12
  Red POS to Arduino pin 11
  Green POS to Arduino pin 10
  560 ohm resistor to both LED NEG and GRD power rail
  More info at: http://goo.gl/kJ8Gl
  Original code improvements to the Ping sketch sourced from Trollmaker.com
  Some code and wiring inspired by http://en.wikiversity.org/wiki/User:Dstaub/robotcar
*/

#define usFrontTrigPin 17
#define usFrontEchoPin 13

#define usLeftTrigPin 12
#define usLeftEchoPin 14

#define usRightTrigPin 2
#define usRightEchoPin 3

enum DistanceDirection {
  Left, Front, Right
};




void setup() {
  Serial.begin (9600);
  pinMode(usLeftTrigPin, OUTPUT);
  pinMode(usFrontTrigPin, OUTPUT);
  pinMode(usRightTrigPin, OUTPUT);
  pinMode(usLeftEchoPin, INPUT);
  pinMode(usFrontEchoPin, INPUT);
  pinMode(usRightEchoPin, INPUT);
}

void loop() {

  int left = getDistance(Left);
  delay(20);
  int front = getDistance(Front);
  delay(20);
  int right = getDistance(Right);
  delay(20);

  Serial.print(left);
  Serial.print('\t');
  Serial.print(front);
  Serial.print('\t');
  Serial.println(right);

  //delay(50);
}


int getDistance(DistanceDirection dir) {
  int trigPin;
  int echoPin;

  switch (dir) {
    case Left: {
        trigPin = usLeftTrigPin;
        echoPin = usLeftEchoPin;
        break;
      }
    case Front: {
        trigPin = usFrontTrigPin;
        echoPin = usFrontEchoPin;
        break;
      }
    case Right: {
        trigPin = usRightTrigPin;
        echoPin = usRightEchoPin;
        break;
      }
  }


  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  return distance;
}
