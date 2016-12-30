
#define usTrigPin 13
#define usEchoPin 17


void setup(void) {
  pinMode(usTrigPin, OUTPUT);
  pinMode(usEchoPin, INPUT);
  
  Serial.begin(9600);
  delay(1000);
 
}



double getDistanceFront() {
  int times = 5;
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
    distance += (duration / 2) / 29.1;
  }
  distance /= times;

 /* Serial.print("Avr: ");
  Serial.print(distance);
  Serial.println(" cm");*/

  return distance;
}



void loop(void) {
  double val = getDistanceFront();
  Serial.println(val);
  delay(50);
}

