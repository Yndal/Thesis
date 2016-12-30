#define LDR_PIN A1


void setup() {   
  Serial.begin(9600);
  pinMode(12, INPUT);
  pinMode(20,OUTPUT);
  pinMode(A10,OUTPUT);
  digitalWrite(A10, LOW);
  analogWrite(20,0);
}
#define counts 20
long value;
void loop() {
  int value = analogRead(LDR_PIN);
  /*value = 0;
  for(int i=0; i<counts;i++){
  value += analogRead(LDR_PIN);
  delay(10);
  }
  value /= counts;*/
  Serial.println(value);

//  int dig = digitalRead(12);
//Serial.print("\t");
//Serial.println(dig);

  

  delay(40);
}

