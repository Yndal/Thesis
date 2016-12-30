#define hm Serial3

/************
 * Remember to use both NL and CR
 ************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  hm.begin(9600);
  delay(1000);
  Serial.println("Ready :)");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(hm.available())
    Serial.print((char)hm.read());
  while(Serial.available())
    hm.write(Serial.read());

}
