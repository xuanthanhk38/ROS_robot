#include "mpu9250_register.h"
#include <Encoder.h>
#define MPU_I2C Wire
Madgwick filter;

Encoder Left(2, 3);
Encoder Right(4, 5);

static double prev_process_time = 0;
static double cur_process_time = 0;

long positionLeft  = -999;
long positionRight = -999;

double process_time;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
long newLeft, newRight;

void setup() {
  Serial.begin(115200);
  initIMU();
}

void loop() {
  cur_process_time = micros();
  updateIMU();
  update_EN();
  String content = String(newLeft) + "|" +String(newRight) + "|" + String(q[0]) + "|" + String(q[1]) + "|" + String(q[2]) + "|" + String(q[3]) + "|" + String(gx) + "|" + String(gy) + "|" + String(gz)+ "|" + String(ax) + "|" + String(ay) + "|" + String(az) + "|" + String(process_time);
  Serial.println(content);
  delay(50);
}

/*********************init imu*********************/
void initIMU()
{
  MPU_I2C.setClock(100000);
  //
  MPU_I2C.begin();
  //

  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71)
  {
    calibrateMPU9250(gyroBias, accelBias);
    initMPU9250();
    getAres();
    getGres();
  }

  readAccelData(accelCount);
  memcpy((void*)accelCount_p, (const void*)accelCount, 6);
}
/*********************init imu*********************/

/*********************update imu*********************/
void updateIMU()
{
  //double process_time;
  
  readAccelData(accelCount); // Read the x/y/z adc values
  for (byte i = 0; i < 3; i++)
  {
    accelCount[i] -= accelBias[i];
  }
  // // Now we'll calculate the accleration value into actual g's
  ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
  ay = (float)accelCount[1] * aRes; // - accelBias[1];
  az = (float)accelCount[2] * aRes; // - accelBias[2];

  readGyroData(gyroCount);
  gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
  gy = (float)gyroCount[1] * gRes;
  gz = (float)gyroCount[2] * gRes;

  if((gyroCount[0] > -30) && (gyroCount[0] <30)) gyroCount[0] = 0;
  if((gyroCount[1] > -30) && (gyroCount[1] <30)) gyroCount[1] = 0;
  if((gyroCount[2] > -30) && (gyroCount[2] <30)) gyroCount[2] = 0;

  if((accelCount[0] > -30) && (accelCount[0] <30)) accelCount[0] = 0;
  if((accelCount[1] > -30) && (accelCount[1] <30)) accelCount[1] = 0;
  if((accelCount[2] > -30) && (accelCount[2] <30)) accelCount[2] = 0;


  static byte temp_count_debounce = 0;
  static uint32_t debounce_time = 0;
  if((accelCount[2] - accelCount_p[2] > 10000) || (accelCount_p[2] - accelCount[2] > 10000))
  {
    memcpy((void*)accelCount, (const void*)accelCount_p, 6);
    memcpy((void*)gyroCount, (const void *)gyroCount_p, 6);
  }
  else
  {
    memcpy((void*)accelCount_p, (const void*)accelCount, 6);
    memcpy((void*)gyroCount_p, (const void*)gyroCount, 6);
  }
  process_time = cur_process_time - prev_process_time;
  prev_process_time = cur_process_time;

  filter.invSampleFreq = (float)process_time / 1000000.0f;
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  q[0] = filter.q0;
  q[1] = filter.q1;
  q[2] = filter.q2;
  q[3] = filter.q3;

  //String content = String(q[0]) + "|" + String(q[1]) + "|" + String(q[2]) + "|" + String(q[3]) + "|" + String(gx) + "|" + String(gy) + "|" + String(gz)+ "|" + String(ax) + "|" + String(ay) + "|" + String(az) + "|" + String(process_time);

  //Serial.println(q[0]);
  //Serial.println(q[1]);
  //Serial.println(q[2]);
  //Serial.println(q[3]);
  //Serial.println(content);
  //Serial.println("--------------------------------------");
}
/*********************update imu*********************/

void update_EN()
{
  //long newLeft, newRight;
  newLeft = Left.read();
  newRight = Right.read();
  if (newLeft != positionLeft || newRight != positionRight) {
  positionLeft = newLeft;
  positionRight = newRight;
  }
  //String content = String(newLeft) + "|" +String(newRight);
  //Serial.println(content);
}
