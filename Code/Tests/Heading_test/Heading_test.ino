#include <EEPROM.h>
#include <Wire.h>
#include <LSM303.h>

#define EEPROM_ADDRESS 0
unsigned int robotNumber = 0;

LSM303 compass;

void setup() {
  Serial.begin(9600);
  EEPROM.get(EEPROM_ADDRESS, robotNumber);
  
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  switch (robotNumber) {
    case 1: {
        //min: {  -593,  +4182, -18334}    max: { +1028,  +5640, -17652}
        compass.m_min = (LSM303::vector<int16_t>) {  -593,  +4182, -18334};
        compass.m_max = (LSM303::vector<int16_t>) { +1028,  +5640, -17652};
        break;
      }

    case 2: {
        //min: { -3326,  +6866, -22804}    max: { -2146,  +7941, -22446}
        //min: { +3171,  +3966,  -8193}    max: { +6582,  +6003,  -5702}
        //min: { +3749,  +3833,  -7654}    max: { +5446,  +5446,  -7021}
        //min: { +3702,  +3744,  -7698}    max: { +5645,  +5937,  -6917}
        //min: { +1982,  +2382,  -7572}    max: { +6078,  +6905,  -1515}

        compass.m_min = (LSM303::vector<int16_t>) { +3702,  +3744,  -7698};
        compass.m_max = (LSM303::vector<int16_t>) { +5645,  +5937,  -6917};
        break;
      }
  }
  delay(5000);
  Serial.print("Zumo number: "); Serial.println(robotNumber);

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
  
  Serial.println(heading);
  delay(300);
}
