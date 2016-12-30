

#define MODULE_PWM_LEFT_PIN    3
#define MODULE_DIR_LEFT_PIN    5
#define MODULE_ENC_A_LEFT_PIN  11
#define MODULE_ENC_B_LEFT_PIN  12

#define MODULE_PWM_RIGHT_PIN   4
#define MODULE_DIR_RIGHT_PIN   6
#define MODULE_ENC_A_RIGHT_PIN 14
#define MODULE_ENC_B_RIGHT_PIN 15

boolean direction = HIGH;

void setup() {
  pinMode(13, OUTPUT);
  // put your setup code here, to run once:
  pinMode(MODULE_PWM_LEFT_PIN,  OUTPUT);
  pinMode(MODULE_DIR_LEFT_PIN,  OUTPUT);
  pinMode(MODULE_PWM_RIGHT_PIN, OUTPUT);
  pinMode(MODULE_DIR_RIGHT_PIN, OUTPUT);

  digitalWrite(MODULE_DIR_LEFT_PIN, direction);
  digitalWrite(MODULE_DIR_RIGHT_PIN, direction);



  digitalWrite(13, HIGH);
}

boolean turn = true;
void loop() {
  if (turn) {
    analogWrite(MODULE_PWM_RIGHT_PIN, 0);
    analogWrite(MODULE_PWM_LEFT_PIN, 150);
  } else {
    analogWrite(MODULE_PWM_RIGHT_PIN, 150);
    analogWrite(MODULE_PWM_LEFT_PIN, 0);
  }

  digitalWrite(MODULE_DIR_LEFT_PIN, !direction);
  digitalWrite(MODULE_DIR_RIGHT_PIN, direction);
  digitalWrite(13, direction);
  /*else {
    if (turn) {
      analogWrite(MODULE_PWM_RIGHT_PIN, 0);
      analogWrite(MODULE_PWM_LEFT_PIN, 150);
    } else {
      //analogWrite(MODULE_PWM_RIGHT_PIN, 150);
      //analogWrite(MODULE_PWM_LEFT_PIN, 0);
    }
    }*/
  turn = !turn;
  if (turn)
    direction = !direction;
  delay(5000);

}
