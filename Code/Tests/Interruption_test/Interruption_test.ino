#define pin 2


void setup() {
  Serial.begin(9600);
  delay(2000);
  // put your setup code here, to run once:
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin),interrupted, FALLING);
}


volatile unsigned long counter = 0;
uint32_t c = 0;
void interrupted() {
  Serial.print("Pin interrupted: ");
  Serial.println(counter++);
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(c++);
  Serial.print(" - Starting delay... ");
  unsigned long endTime = micros() + 10 * 1000 * 1000;
  NVIC_DISABLE_IRQ(digitalPinToInterrupt(pin));
  while(micros() < endTime);
  //interrupts();
  Serial.println("Ended delay!");
}
