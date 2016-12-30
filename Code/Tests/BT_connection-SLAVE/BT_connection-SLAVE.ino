//#include "BT.h"

#define bt Serial2

//BT bt;

String robotName = "Robot_1";
//char robotMac[12] = {'8','8','4','A','E','A','3','B','9','6','1','8'}; //884AEA3B9618
String slaveMac = "884AEA3B9618";
String cmdsSlave[] = {
        "AT",
        "AT+RESET",
        "AT+IMME1",
        "AT+NOTI1",
        "AT+ROLE0",
        "AT+FILT",
        "AT+SHOW1",
        }; 

void setup() {
  Serial.begin(9600);
  bt.begin(9600);
  delay(1000);
  Serial.println("Starting SLAVE robot...");
  char buf[64];
  for(String cmd : cmdsSlave){
    cmd.toCharArray(buf, sizeof(buf));
    Serial.write(buf);
    bt.write(buf);
    
    delay(1000);
    checkForMsg();
    
  }
  /*//bt.init();
  init();
//bt.checkForMsg();
  Serial.print(" Done!");
  Serial.println("Establishing connection...");
  makeSlave();
  tryConnectTo(slaveMac);
  Serial.println(" Done!");*/
}
void checkForMsg() {
  while(bt.available())
    Serial.write(bt.read());
  // put your main code here, to run repeatedly:
  Serial.println();
}



void makeMaster(){
  bt.write("AT");
  checkForMsg();
  delay(1000);
  bt.write("AT+ROLE1"); //Act as central
  checkForMsg();
  delay(1000);
  bt.write("AT+SHOW1"); //Show name
  checkForMsg();
  delay(1000);
}

void makeSlave(){
  bt.write("AT");
  checkForMsg();
  delay(1000);
  bt.write("AT+ROLE0"); //Act as peripheral
  checkForMsg();
  delay(1000);
  bt.write("AT+SHOW1"); //Show name
  checkForMsg();
  delay(1000);
}

/*bool tryConnectTo(char mac[12]){
//  if(!initialized)
  //  init();
  makeMaster();
  bt.write("AT+CON");
  bt.write(mac);

    

   return true;
}*/
bool tryConnectTo(String mac){
//  if(!initialized)
  //  init();
  makeMaster();
  String cmd = "AT+CON" + mac;
  //bt.write("AT+CON");
  bt.print(cmd);

    

   return true;
}

void handleIncomingBT(){
/*  String msg = bt.msg;
  Serial.print("From BT: ");
  Serial.println(msg);
*/
}

long counter = 0;
void loop() {
  // put your main code here, to run repeatedly:
  //checkForMsg();
    //handleIncomingBT();
  //}

//  bt.sendMsg(MsgCommand::OK);
bt.write("Hello from slave!\n;");
delay(500);
}
