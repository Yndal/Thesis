/*
   Heavily based on Pololu code: https://github.com/pololu/minimu-9-ahrs-arduino
*/

#ifndef MinIMU9_v5_h
#define MinIMU9_v5_h

#include "Arduino.h"

#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>




// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second



#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw


class MinIMU9_v5 {
  public:
    
    void init();
    void calibrateMag(int ms);
    void calibrateAccGyro();
    void setOffset(int xMin, int xMax, int yMin, int yMax, int zMin, int zMax);

    //void Read();
    float getHeading();

  private:
    // LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
    // the Pololu LSM303 or LIS3MDL library to find the right values for your board
    int M_X_MIN  = -1000;
    int M_Y_MIN = -1000;
    int M_Z_MIN = -1000;
    int M_X_MAX = +1000;
    int M_Y_MAX = +1000;
    int M_Z_MAX = +1000;


    LSM6 gyro_acc;
    LIS3MDL mag;
    bool _initialized = false;
    
    void Normalize(void);
    void Drift_correction(void);
    /*
      void Accel_adjust(void)
      {
      Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
      Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
      }
    */
    void Compass_Heading();
    void Read_Gyro();
    void Read_Accel();
    void Read_Compass();
    void printdata(void);

    void Matrix_update(void);
    void Euler_angles(void);

    void I2C_Init();
    void Gyro_Init();
    void Accel_Init();
    void Compass_Init();

    //Computes the dot product of two vectors
    float Vector_Dot_Product(float vector1[3], float vector2[3]);

    //Computes the cross product of two vectors
    void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);

    //Multiply the vector by a scalar.
    void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);

    void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);

    //Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's.
    void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]);


    // Uncomment the below line to use this axis definition:
    // X axis pointing forward
    // Y axis pointing to the right
    // and Z axis pointing down.
    // Positive pitch : nose up
    // Positive roll : right wing down
    // Positive yaw : clockwise
    //int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
    // Uncomment the below line to use this axis definition:
    // X axis pointing forward
    // Y axis pointing to the left
    // and Z axis pointing up.
    // Positive pitch : nose down
    // Positive roll : right wing down
    // Positive yaw : counterclockwise
    int SENSOR_SIGN[9] = {1, -1, -1, -1, 1, 1, 1, -1, -1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

    float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

    long timer = 0; //general purpuse timer
    long timer_old;
    long timer24 = 0; //Second timer used to print values
    int AN[6]; //array that stores the gyro and accelerometer data
    long AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

    int gyro_x;
    int gyro_y;
    int gyro_z;
    int accel_x;
    int accel_y;
    int accel_z;
    int magnetom_x;
    int magnetom_y;
    int magnetom_z;
    float c_magnetom_x;
    float c_magnetom_y;
    float c_magnetom_z;
    float MAG_Heading;

    float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
    float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
    float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
    float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
    float Omega_I[3] = {0, 0, 0}; //Omega Integrator
    float Omega[3] = {0, 0, 0};

    // Euler angles
    float roll;
    float pitch;
    float yaw;

    float errorRollPitch[3] = {0, 0, 0};
    float errorYaw[3] = {0, 0, 0};

    // unsigned int counter = 0;
    byte gyro_sat = 0;

    float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


    float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

};


#endif
