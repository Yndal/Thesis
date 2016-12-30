#include <EEPROM.h>
#include <Wire.h>
#include <Zumo32U4.h>
//#include <LSM303.h>

#define SPEED           100 // Maximum motor speed when going straight; variable speed when turning
#define TURN_BASE_SPEED 90 // Base speed when turning (added to variable speed)

#define CALIBRATION_SAMPLES 700  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

#define EEPROM_ADDRESS 0
unsigned int robotNumber = 0;

LSM303 compass;
Zumo32U4ButtonA button;
Zumo32U4Motors motors;
Zumo32U4LCD lcd;

char report[120];

void setup() {
  Serial.begin(9600);
  EEPROM.get(EEPROM_ADDRESS, robotNumber);

  lcd.clear();
  lcd.print("Robot ");
  lcd.print(robotNumber);
  //lcd.gotoXY(4,1);
  //lcd.print(robotNumber);

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  button.waitForButton();
  
  calibrate();
  button.waitForButton();
}

void calibrate(){
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  

  Serial.println("starting calibration");
  lcd.clear();
  lcd.gotoXY(1,0);
  lcd.print("Please");
  lcd.gotoXY(2,1);
  lcd.print("wait");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  int index;
  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();
   
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_min.z = min(running_min.z, compass.m.z);
  
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    running_max.z = max(running_max.z, compass.m.z);

 //   Serial.println(index);

    delay(20);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_max.z = running_max.z;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
  compass.m_min.z = running_min.z;

  lcd.clear();
  lcd.gotoXY(2,0);
  lcd.print("Done");
}

void printHeading(float h){
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Heading: ");
  lcd.gotoXY(2,1);
  lcd.print(h,2);
}

void loop() {
  compass.read();
  
  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector and
  north, in degrees.
  
  The default vector is chosen by the library to point along the
  surface of the PCB, in the direction of the top of the text on the
  silkscreen. This is the +X axis on the Pololu LSM303D carrier and
  the -Y axis on the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH
  carriers.
  
  To use a different vector as a reference, use the version of heading()
  that takes a vector argument; for example, use
  
    compass.heading((LSM303::vector<int>){0, 0, 1});
  
  to use the +Z axis as a reference.
  */
  
  
  float heading = compass.heading();
  printHeading(heading);
  //Serial.println(h);
  
  delay(250);
}
