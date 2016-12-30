#define PWM 5
#define LED_SELECT 19


bool left = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM, OUTPUT);
  pinMode(LED_SELECT, OUTPUT);

  digitalWrite(PWM, HIGH);
}

void loop() {
   digitalWrite(LED_SELECT, left);
   left = !left;
   Serial.println(left ? "Left" : "Right");
   delay(10000);
  // put your main code here, to run repeatedly:

}
