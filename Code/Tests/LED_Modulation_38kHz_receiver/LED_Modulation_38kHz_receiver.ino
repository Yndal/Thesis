// Create an IntervalTimer object 
IntervalTimer myTimer;

#define BUFFER_LENGTH 12
volatile bool buffer[BUFFER_LENGTH];
volatile uint8_t bufferCounter = 0;

const int pin = A1;  

void setup(void) {
  pinMode(pin, INPUT);
  pinMode(13, OUTPUT);
 // Serial.begin(9600);
  for(int i=0; i<BUFFER_LENGTH; i++)
    buffer[0] = LOW;
  delay(1000);
  Serial.println("This is left untested");
  myTimer.begin(blinkLED,(1/(38000.0*2))*1000000);  // blinkLED at 38kHz
}

int ledState = LOW;
volatile int val;

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void blinkLED(void) {
  int value = digitalRead(pin);
  val = value;
 // digitalWrite(13, value);
  //digitalWrite(ledPin, ledState);

  buffer[bufferCounter++] = value;
  if(BUFFER_LENGTH <= bufferCounter)
    bufferCounter = 0;
}

bool checkBuffer(){
  bool lastState = buffer[0];
  for(int i=1; i<BUFFER_LENGTH; i++){
    if(lastState != buffer[i]){
      lastState = buffer[i];
    } else {
      return false;
    }
  }
  return true;
}


void loop(void) {
  noInterrupts();
 // Serial.println(val);
  if(checkBuffer())
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);
  interrupts();

 // delay(100);
}
