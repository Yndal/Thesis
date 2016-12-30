
#include "MinIMU9_v5.h"

void MinIMU9_v5::init() {
  if (_initialized)
    return;
  _initialized = true;

  Serial.println("MinIMU9_v5 init()");
  I2C_Init();
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  delay(20);
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  //Very dirty calibration!!
  for (int i = 0; i < 32; i++) // We take some readings...
  {
    Read_Gyro();
    Read_Accel();
    for (int y = 0; y < 6; y++) // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }

  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / 32;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  /* Serial.println("Offset:");
    for (int y = 0; y < 6; y++)
     Serial.println(AN_OFFSET[y]);*/


  timer = millis();
  /*delay(20);
    counter = 0;*/

}

void MinIMU9_v5::calibrateAccGyro() {
  init();

  //Set all offsets (acc and gyro) to Zero
  for (int i = 0; i < 6; i++)
    AN_OFFSET[i] = 0;

  //Do this for 3 seconds
  unsigned long end = 3000 + millis();
  int counter = 0;
  while (millis() <= end) {

    // We take some readings...
    counter++;
    Read_Gyro();
    Read_Accel();

    // Cumulate values
    for (int y = 0; y < 6; y++) {
      AN_OFFSET[y] += AN[y];

      if (2147483647 - 10000 < abs(AN_OFFSET[y]) )
        Serial.println("EXTREME RISK OF OVERFLOW IN MinIMU9_v5 calibrationAccGyro()!");
    }
    delay(20);
  }

  //Average the values
  for (int y = 0; y < 6; y++)
    AN_OFFSET[y] = AN_OFFSET[y] / counter;

  AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

  /*Serial.println("Offsets:");
    for (int y = 0; y < 6; y++)
    Serial.println(AN_OFFSET[y]);*/
}

void MinIMU9_v5::calibrateMag(int ms) {
  init();

  //Set all offsets to opposite extreme
  M_X_MIN = 32767;
  M_Y_MIN = 32767;
  M_Z_MIN = 32767;
  M_X_MAX = -32768;
  M_Y_MAX = -32768;
  M_Z_MAX = -32768;


  unsigned long end = millis() + ms;
  while (millis() <= end) {
    Read_Compass();

    if (mag.m.x < M_X_MIN)
      M_X_MIN = mag.m.x;
    else if (M_X_MAX < mag.m.x)
      M_X_MAX = mag.m.x;

    if (mag.m.y < M_Y_MIN)
      M_Y_MIN = mag.m.y;
    else if (M_Y_MAX < mag.m.y)
      M_Y_MAX = mag.m.y;

    if (mag.m.z < M_Z_MIN)
      M_Z_MIN = mag.m.z;
    else if (M_Z_MAX < mag.m.z)
      M_Z_MAX = mag.m.z;

    delay(50);
  }
}


void MinIMU9_v5::setOffset(int xMin, int xMax, int yMin, int yMax, int zMin, int zMax) {
  init();
  //The values MUST be set after init() - otherwise they will be overwritten!
  M_X_MIN = xMin;
  M_Y_MIN = yMin;
  M_Z_MIN = zMin;
  M_X_MAX = xMax;
  M_Y_MAX = yMax;
  M_Z_MAX = zMax;
}




float MinIMU9_v5::getHeading()
{
  init();
  //for(int counter=0; counter<5; counter++){
  while ((millis() - timer) <= 20); //Idle to prevent calling too often (*think* it will give better readings...)

  timer_old = timer;
  timer = millis();
  if (timer > timer_old)
  {
    G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    if (G_Dt > 0.2)
      G_Dt = 0; // ignore integration times over 200 ms
  }
  else
    G_Dt = 0;


  // *** DCM algorithm
  // Data adquisition
  Read_Gyro();   // This read gyro data
  Read_Accel();     // Read I2C accelerometer

  //  if (counter >= 4)  // Read compass data at 10Hz... (5 loop runs)
  {
    //    counter = 0;
    Read_Compass();    // Read I2C magnetometer
    Compass_Heading(); // Calculate magnetic heading
  }

  // Calculations...
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();
  // ***

  //printdata();
  //}

  //Serial.print(F("Return heading: ")); Serial.println(ToDeg(yaw));
  return ToDeg(yaw);
}











void MinIMU9_v5::Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6] * M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6] * 0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7] * M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7] * 0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8] * M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8] * 0.5;

  // Tilt compensated Magnetic filed X:
  MAG_X = c_magnetom_x * cos_pitch + c_magnetom_y * sin_roll * sin_pitch + c_magnetom_z * cos_roll * sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = c_magnetom_y * cos_roll - c_magnetom_z * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y, MAG_X);
}

void MinIMU9_v5::Normalize(void)
{
  float error = 0;
  float temporary[3][3];
  float renorm = 0;

  error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

  Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

  renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}



/**************************************************/
void MinIMU9_v5::Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;


  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

  Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

  Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading

  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x); //Calculating YAW error
  Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW); //.01proportional of YAW.
  Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.

  Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001Integrator
  Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I
}
/**************************************************/
/*
  void MinIMU9_v5::Accel_adjust(void)
  {
  Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
  Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
  }
*/
/**************************************************/

void MinIMU9_v5::Matrix_update(void)
{
  Gyro_Vector[0] = Gyro_Scaled_X(gyro_x); //gyro x roll
  Gyro_Vector[1] = Gyro_Scaled_Y(gyro_y); //gyro y pitch
  Gyro_Vector[2] = Gyro_Scaled_Z(gyro_z); //gyro Z yaw

  Accel_Vector[0] = accel_x;
  Accel_Vector[1] = accel_y;
  Accel_Vector[2] = accel_z;

  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement

#if OUTPUTMODE==1
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
  Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; //y
  Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
  Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
  Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; //x
  Update_Matrix[2][2] = 0;
#else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0] = 0;
  Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
  Update_Matrix[0][2] = G_Dt * Gyro_Vector[1]; //y
  Update_Matrix[1][0] = G_Dt * Gyro_Vector[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
  Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
  Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
  Update_Matrix[2][2] = 0;
#endif

  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

  for (int x = 0; x < 3; x++) //Matrix Addition (update)
  {
    for (int y = 0; y < 3; y++)
    {
      DCM_Matrix[x][y] += Temporary_Matrix[x][y];
    }
  }
}

void MinIMU9_v5::Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}



void MinIMU9_v5::I2C_Init()
{
  Wire.begin();
}

void MinIMU9_v5::Gyro_Init()
{
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
}

/*void MinIMU9_v5::Read(){
  Read_Gyro();
  Read_Accel();
  Read_Compass();
  }*/

void MinIMU9_v5::Read_Gyro()
{
  gyro_acc.readGyro();

  AN[0] = gyro_acc.g.x;
  AN[1] = gyro_acc.g.y;
  AN[2] = gyro_acc.g.z;

  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void MinIMU9_v5::Accel_Init()
{
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
}

// Reads x,y and z accelerometer registers
void MinIMU9_v5::Read_Accel()
{
  gyro_acc.readAcc();

  AN[3] = gyro_acc.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = gyro_acc.a.y >> 4;
  AN[5] = gyro_acc.a.z >> 4;

  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void MinIMU9_v5::Compass_Init()
{
  mag.init();
  mag.enableDefault();
}

void MinIMU9_v5::Read_Compass()
{
  mag.read();

  magnetom_x = SENSOR_SIGN[6] * mag.m.x;
  magnetom_y = SENSOR_SIGN[7] * mag.m.y;
  magnetom_z = SENSOR_SIGN[8] * mag.m.z;
}

void MinIMU9_v5::printdata(void)
{
  Serial.print("!");

#if PRINT_EULER == 1
  Serial.print("ANG:");
  Serial.print(ToDeg(roll));
  Serial.print(",");
  Serial.print(ToDeg(pitch));
  Serial.print(",");
  Serial.print(ToDeg(yaw));
#endif
#if PRINT_ANALOGS==1
  Serial.print(",AN:");
  Serial.print(AN[0]);  //(int)read_adc(0)
  Serial.print(",");
  Serial.print(AN[1]);
  Serial.print(",");
  Serial.print(AN[2]);
  Serial.print(",");
  Serial.print(AN[3]);
  Serial.print (",");
  Serial.print(AN[4]);
  Serial.print (",");
  Serial.print(AN[5]);
  Serial.print(",");
  Serial.print(c_magnetom_x);
  Serial.print (",");
  Serial.print(c_magnetom_y);
  Serial.print (",");
  Serial.print(c_magnetom_z);
#endif
#if PRINT_DCM == 1
  Serial.print (",DCM:");
  Serial.print(DCM_Matrix[0][0]);
  Serial.print (",");
  Serial.print(DCM_Matrix[0][1]);
  Serial.print (",");
  Serial.print(DCM_Matrix[0][2]);
  Serial.print (",");
  Serial.print(DCM_Matrix[1][0]);
  Serial.print (",");
  Serial.print(DCM_Matrix[1][1]);
  Serial.print (",");
  Serial.print(DCM_Matrix[1][2]);
  Serial.print (",");
  Serial.print(DCM_Matrix[2][0]);
  Serial.print (",");
  Serial.print(DCM_Matrix[2][1]);
  Serial.print (",");
  Serial.print(DCM_Matrix[2][2]);
#endif
  Serial.println();

}

//Computes the dot product of two vectors
float MinIMU9_v5::Vector_Dot_Product(float vector1[3], float vector2[3])
{
  float op = 0;

  for (int c = 0; c < 3; c++)
  {
    op += vector1[c] * vector2[c];
  }

  return op;
}

//Computes the cross product of two vectors
void MinIMU9_v5::Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
  vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar.
void MinIMU9_v5::Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
  for (int c = 0; c < 3; c++)
  {
    vectorOut[c] = vectorIn[c] * scale2;
  }
}

void MinIMU9_v5::Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
  for (int c = 0; c < 3; c++)
  {
    vectorOut[c] = vectorIn1[c] + vectorIn2[c];
  }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's.
void MinIMU9_v5::Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
  for (int x = 0; x < 3; x++)
  {
    for (int y = 0; y < 3; y++)
    {
      mat[x][y] = 0;

      for (int w = 0; w < 3; w++)
      {
        mat[x][y] += a[x][w] * b[w][y];
      }
    }
  }
}

