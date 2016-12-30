//#include "BT.h"

#define bt Serial2

//BT bt;

String robotName = "Robot_1";
//char robotMac[12] = {'8','8','4','A','E','A','3','B','9','6','1','8'}; //884AEA3B9618
String slaveMac = "884AEA3B9618";
String cmdsMaster[] = {
        "AT",
        "AT+RENEW",
        "AT+IMME1",
        "AT+NOTI1",
        "AT+ROLE1",
        "AT+SHOW1",
        "AT+CON884AEA3B9618",
        }; 

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  delay(5000);
  Serial.println("Starting robot...");
  char buf[64];
  for(String cmd : cmdsMaster){
    cmd.toCharArray(buf, sizeof(buf));
    Serial.write(buf);
    bt.write(buf);
    
    delay(500);
    checkForMsg();
    
    delay(1000);
    checkForMsg();
    
  }
}
void checkForMsg() {
  while(bt.available())
    Serial.write(bt.read());
  while(Serial.available())
    bt.write(Serial.read());
  // put your main code here, to run repeatedly:
  Serial.println();
}

/*bool tryConnectTo(String mac){
//  if(!initialized)
  //  init();
  makeMaster();
  String cmd = "AT+CON" + mac;
  //bt.write("AT+CON");
  bt.print(cmd);

    

   return true;
}
*/

#define BT_BUFFER_SIZE 32
#define END_CHAR ';'
char btBuffer[BT_BUFFER_SIZE];
int btBufferCounter = 0;
String btString;
String strSoFar;
bool isMaster = true;
bool newStringFromHM10(){
  while(bt.available()){ //Only once per loop to avoid overwriting an unhandled cmd
    char c = bt.read();
    if(c == END_CHAR){
      btBuffer[btBufferCounter] = '\0'; //Terminate the string
      btString = String(btBuffer);
      btBufferCounter = 0;
      return true;
    } else {
      btBuffer[btBufferCounter++] = c;
      btBuffer[btBufferCounter] = '\0';
      strSoFar = String(btBuffer);
      if(-1 < strSoFar.indexOf("OK+LOST")){
        if(isMaster){
          tryReestablishLastConnection();
          btBufferCounter = 0;
        }
      } else if(-1 < strSoFar.indexOf("OK+CONN")){
        Serial.println(F("Connected!"));
        btBufferCounter = 0;
      }
    }
    
  }
  
  return false;
}


void tryReestablishLastConnection(){
  //TODO
  Serial.println(F("Should try to reestablish connect to last device now"));
}
void handleHM10String(){
/*  String msg = bt.msg;
  Serial.print("From BT: ");
  Serial.println(msg);
*/
}


void loop() {
  if(newStringFromHM10()){
    handleHM10String();
  }
  
  // put your main code here, to run repeatedly:
  //checkForMsg();
    //handleIncomingBT();
  //}

//  bt.sendMsg(MsgCommand::OK);
   //checkForMsg();
//delay(500);
}
