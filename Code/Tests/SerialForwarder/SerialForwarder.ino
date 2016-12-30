#define bt Serial2

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  while(Serial.available())
    bt.write(Serial.read());

  while(bt.available())
    Serial.write(bt.read());
  // put your main code here, to run repeatedly:

}
