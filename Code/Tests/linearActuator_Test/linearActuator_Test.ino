#include <Servo.h> 

#define DELAY 3000
Servo myservo;

void setup() 
{ 
  myservo.attach(5);
} 

void loop() {
  myservo.writeMicroseconds(810);  // set servo to start-point
  delay(DELAY+1000);
  myservo.writeMicroseconds(1500);  // set servo to mid-point
  delay(DELAY);
  myservo.writeMicroseconds(2125);  // set servo to end-point
  delay(DELAY); 
} 
