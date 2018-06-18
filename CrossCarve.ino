// Alexander Warren
// The gyro (MPU-6050) code that is used is based off of the example
// (MPU6050_DMP.ino) from the I2CDev library. The copyright below is
// because of that

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Fastwire.h" 


MPU6050 mpu;
int mpuInterruptPin = 2;
int packetSize = 42;
uint8_t mpuIntStatus;
bool dmpReady = false;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
volatile bool mpuIntVal = false;     // indicates whether MPU interrupt pin has gone high
void mpuIntTriggered() {
  mpuIntVal = true;
}
#define MPU6050_DMP_FIFO_RATE_DIVISOR 99
#define I2CDEV_IMPLEMENTATION I2CDEV_BUILTIN_FASTWIRE

Quaternion q;
VectorFloat gravity;
float ypr[3];
//
MPU6050 mpu2(0x69);
int mpuInterruptPin2 = 3;
uint8_t mpuIntStatus2;
bool dmpReady2 = false;
uint16_t fifoCount2;
uint8_t fifoBuffer2[64];
volatile bool mpuIntVal2 = false;     // indicates whether MPU interrupt pin has gone high
void mpuIntTriggered2() {
  mpuIntVal2 = true;
}

Quaternion q2;
VectorFloat gravity2;
float ypr2[3];

unsigned long startMillis;

void setup() {
  Fastwire::setup(50, true);
  Serial.begin(115200);
  //  mpu.initialize();
  //  mpu2.initialize();
  //  pinMode(mpuInterruptPin, INPUT);
  //  pinMode(mpuInterruptPin2, INPUT);
  //  uint8_t dmpInitStatus = mpu.dmpInitialize();
  //  uint8_t dmpInitStatus2 = mpu2.dmpInitialize();

  bool mpuSetupSuccess = setupMPU(&mpu, mpuInterruptPin, &mpuIntStatus, &dmpReady);
  if (mpuSetupSuccess) {
    attachInterrupt(digitalPinToInterrupt(mpuInterruptPin), mpuIntTriggered, RISING);
  }

  bool mpuSetupSuccess2 = setupMPU(&mpu2, &mpuInterruptPin2, &mpuIntStatus2, &dmpReady2);
  if (mpuSetupSuccess2) {
    attachInterrupt(digitalPinToInterrupt(mpuInterruptPin2), mpuIntTriggered2, RISING);
  }
  startMillis = millis();
}

bool setupMPU(MPU6050* gyro, int intPin, uint8_t* intStatus, bool* dmpReadyVal) {
  gyro->initialize();
  pinMode(intPin, INPUT);
  uint8_t dmpInitStatus = gyro->dmpInitialize();

  switch (dmpInitStatus) {
    case 0:
      Serial.println("success");
      gyro->setDMPEnabled(true);
      *intStatus = gyro->getIntStatus();
      *dmpReadyVal = true;
      return true;
    case 1:
      Serial.println("Error Code 1: main binary block loading failed");
      return false;
    case 2:
      Serial.println("Error Code 2: configuration block loading failed");
      return false;
    default:
      return false;

  }

}

//float* getAngles(MPU6050* gyro, bool* dmpReadyVal, uint8_t* intStatus, bool* intVal, int16_t* fifoCountVal, uint8_t* fifoBufferVal) {
//  Serial.println("GetAngles");
//  if (*dmpReadyVal) {
//    Serial.print("dmpReadyVal");
//    Serial.println(*dmpReadyVal);
//
//    while (!*intVal && *fifoCountVal < packetSize) {
//       Serial.print("fifoCountVal (before)");
//       Serial.println(*fifoCountVal);
//
//    }
//    *intVal = false;
//    *intStatus = gyro->getIntStatus();
//
//    *fifoCountVal = gyro->getFIFOCount();
//     Serial.print("fifoCountVal (after)");
//     Serial.println(*fifoCountVal);
//
//    if ((*intStatus & 0x10) || *fifoCountVal == 1024) {
//      Serial.println("FIFO overflow!");
//
//      gyro->resetFIFO();
//    } else if (*intStatus & 0x02) {
//
//      while (*fifoCountVal < packetSize) {
//        *fifoCountVal = gyro->getFIFOCount();
//        
//      }
//
//      gyro->getFIFOBytes(*fifoBufferVal, packetSize);
//      *fifoCountVal -= packetSize;
//
//      Quaternion q;
//      VectorFloat gravity;
//      float angles[3];
//      gyro->dmpGetQuaternion(&q, *fifoBufferVal);
//      gyro->dmpGetGravity(&gravity, &q);
//      gyro->dmpGetYawPitchRoll(angles, &q, &gravity);
//      return angles;
//    }
//  }
//}

void loop() {
  mpu.resetFIFO();
  mpu2.resetFIFO();
//  Serial.println(mpu.getFIFOCount());
//  float *yprL = getAngles(&mpu, &dmpReady, &mpuIntStatus, &mpuIntVal, &fifoCount, fifoBuffer);
  if (dmpReady) {

    while (!mpuIntVal && fifoCount < packetSize) {

    }
    mpuIntVal = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      Serial.println("FIFO overflow!");

      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {

      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
        
      }

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / M_PI);
      Serial.print("\t");
      
      }
  }

//
//  float *yprL2 = getAngles(&mpu2, &dmpReady2, &mpuIntStatus2, &mpuIntVal2, &fifoCount2, fifoBuffer2);
//
//  ypr2[0] = yprL2[0];
//  ypr2[1] = yprL2[1];
//  ypr2[2] = yprL2[2];
//  Serial.print("ypr2\t");
//  Serial.print(ypr2[0] * 180 / M_PI);
//  Serial.print("\t");
//  Serial.print(ypr2[1] * 180 / M_PI);
//  Serial.print("\t");
//  Serial.print(ypr2[2] * 180 / M_PI);
//  Serial.println("\t");


  if (dmpReady2) {

    while (!mpuIntVal2 && fifoCount2 < packetSize) {

    }
    mpuIntVal2 = false;
    mpuIntStatus2 = mpu2.getIntStatus();

    fifoCount2 = mpu2.getFIFOCount();

    if ((mpuIntStatus2 & 0x10) || fifoCount2 == 1024) {
      Serial.println("FIFO overflow!");

      mpu2.resetFIFO();
    } else if (mpuIntStatus2 & 0x02) {

      while (fifoCount2 < packetSize) {
        fifoCount2 = mpu2.getFIFOCount();
        
      }

      mpu2.getFIFOBytes(fifoBuffer2, packetSize);
      fifoCount2 -= packetSize;

      mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
      mpu2.dmpGetGravity(&gravity2, &q2);
      mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);

      Serial.print("ypr2\t");
      Serial.print(ypr2[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr2[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr2[2] * 180 / M_PI);
      Serial.println("\t");
      
      }
  }

  Serial.print("Uptime: ");
  Serial.println((millis() - startMillis) / 1000);

}
