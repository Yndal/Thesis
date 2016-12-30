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

#define IS_TRANSMITTER false

#define trigPin 3
#define echoPin 2
#define ledPin 13

bool ledStatus = HIGH;
void setup() {
  Serial.begin (9600);

  //if (IS_TRANSMITTER){
    pinMode(trigPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
  //} else {
    pinMode(echoPin, INPUT);
   // pinMode(ledPin, OUTPUT);
 // }
}

void loop() {
  if (IS_TRANSMITTER) {
    digitalWrite(trigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    //  delayMicroseconds(1000); - Removed this line
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);

    Serial.println("Transmittet...");
    digitalWrite(ledPin, ledStatus);
    ledStatus = !ledStatus;
    delay(500);
  } else {
    long duration;//, distance;
    duration = pulseIn(echoPin, HIGH);
    //distance = (duration/2) / 29.1;

    Serial.print("Got signal! Time=");
    Serial.println(duration);
    digitalWrite(ledPin, ledStatus);
    ledStatus = !ledStatus;
  }
}




