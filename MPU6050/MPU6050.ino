

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "filter.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define LPF_PERIOD 8
//#define DEBUG_PRINTF

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroPreX[LPF_PERIOD]={0},gyroPreY[LPF_PERIOD]={0},gyroPreZ[LPF_PERIOD]={0};
double gyroOffsetX,gyroOffsetY,gyroOffsetZ;
double gyroMaX,gyroMaY,gyroMaZ;
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
double lpfAngleX,lpfAngleY,lpfAngleZ;
double preAngleX=0,preAngleY=0,preAngleZ=0;

String str_out;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int cntMainloop=0;

// TODO: Make calibration routine

void setup() {
  Serial.begin(9600);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once  
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees


#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double yaw = atan2(accY,accX) * RAD_TO_DEG;
  /*
  double yaw = atan2(accY,accX) * RAD_TO_DEG;
  double roll = atan2(accZ,accX) * RAD_TO_DEG;
  double pitch = atan2(accZ,accY) * RAD_TO_DEG;
  */

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros();
}

void loop() {

  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  for(int i=LPF_PERIOD-1;i>0;i--){
    gyroPreX[i]=gyroPreX[i-1];
    gyroPreY[i]=gyroPreY[i-1];
    gyroPreZ[i]=gyroPreZ[i-1];
  }
  gyroPreX[0]=gyroX;
  gyroPreY[0]=gyroY;
  gyroPreZ[0]=gyroZ;
  gyroMaX=0;gyroMaY=0;gyroMaY=0;
  for(int i=0;i<LPF_PERIOD;i++){
    gyroMaX+=gyroPreX[i];
    gyroMaY+=gyroPreY[i];
    gyroMaZ+=gyroPreZ[i];
  }
  gyroMaX/=LPF_PERIOD;
  gyroMaY/=LPF_PERIOD;
  gyroMaZ/=LPF_PERIOD;

  if(cntMainloop==LPF_PERIOD-1){
    gyroOffsetX=gyroMaX;
    gyroOffsetY=gyroMaY;
    gyroOffsetZ=gyroMaZ;
  }

  if(cntMainloop>=LPF_PERIOD){
    
      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();
    
      // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
      // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
      // It is then converted from radians to degrees
    
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(accY, accZ) * RAD_TO_DEG;
      double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
      double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif
    double yaw = atan2(accY,accX) * RAD_TO_DEG;
    /*
  double yaw = atan2(accY,accX) * RAD_TO_DEG;
  double roll = atan2(accZ,accX) * RAD_TO_DEG;
  double pitch = atan2(accZ,accY) * RAD_TO_DEG;

  str_out = String(yaw) + "," + String(roll) + "," + String(pitch);
  Serial.println(str_out);
  */
    
      double gyroXrate = (gyroMaX-gyroOffsetX) / 131.0; // Convert to deg/s
      double gyroYrate = (gyroMaY-gyroOffsetY) / 131.0; // Convert to deg/s
      double gyroZrate = (gyroMaZ-gyroOffsetZ) / 131.0; // Convert to deg/s

    #ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    
      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    #else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
    
      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif

      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
        kalmanZ.setAngle(yaw);
        compAngleZ = yaw;
        kalAngleZ = yaw;
        gyroZangle = yaw;
      } else
        kalAngleZ = kalmanZ.getAngle(pitch, gyroZrate, dt); // Calculate the angle using a Kalman filter
    
      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      gyroZangle += gyroZrate * dt;
      gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      gyroYangle += kalmanY.getRate() * dt;
      gyroZangle += kalmanZ.getRate() * dt;
    
      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
      compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
    
      // Reset the gyro angle when it has drifted too much
      if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
      if (gyroZangle < -180 || gyroZangle > 180)
        gyroZangle = kalAngleZ;
    
      lpfAngleX=0.95 * preAngleX  + 0.05 * gyroX;
      lpfAngleY=0.95 * preAngleY  + 0.05 * gyroY;
    
      #ifdef DEBUG_PRINTF
      #endif
      
        /* Print Data */
      #if 0 // Set to 1 to activate
      /*センサの生データ*/
        Serial.print(accX); Serial.print("\t");
        Serial.print(accY); Serial.print("\t");
        Serial.print(accZ); Serial.print("\t");
      
        //Serial.print(gyroX); Serial.print("\t");
        //Serial.print(gyroY); Serial.print("\t");
        //Serial.print(gyroZ); Serial.print("\t");
        
        //Serial.print(gyroMaX); Serial.print("\t");
        //Serial.print(gyroMaY); Serial.print("\t");
        //Serial.print(gyroMaZ); Serial.print("\t");
      
        Serial.print("\t");
      #endif

      /*
        
      //Serial.print(roll); Serial.print("\t");
      //Serial.print(gyroXangle); Serial.print("\t");
      //Serial.print(compAngleX); Serial.print("\t");
      Serial.print(kalAngleX); Serial.print("\t");
      //Serial.print(lpfAngleX); Serial.print("\t");
    
      Serial.print("\t");
    
      //Serial.print(pitch); Serial.print("\t");
      //Serial.print(gyroYangle); Serial.print("\t");
      //Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");
      //Serial.print(lpfAngleY); Serial.print("\t");

      Serial.print("\t");
      
      Serial.print(kalAngleZ); Serial.print("\t");
      
    
      #if 0 // Set to 1 to print the temperature
      Serial.print("\t");
    
      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.print(temperature); Serial.print("\t");
      #endif

      #else

      Serial.print("roll = ");
      Serial.print(kalAngleX);
      Serial.print(", pitch = ");
      Serial.print(kalAngleY);
      Serial.print(", yaw = ");
      Serial.print(yaw);

      #endif

      */
  str_out = String(kalAngleX) + "," + String(kalAngleY);
  Serial.println(str_out);
    
     // Serial.print("\r\n");
    
      preAngleX=gyroX;
      preAngleY=gyroY;
      preAngleZ=gyroZ;


  }

  cntMainloop++;

}


