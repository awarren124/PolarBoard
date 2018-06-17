
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

void setup() {
  Wire.begin();
  Wire.setClock(2);
  Serial.begin(115200);
  mpu.initialize();
  mpu2.initialize();
  pinMode(mpuInterruptPin, INPUT);
  pinMode(mpuInterruptPin2, INPUT);
  uint8_t dmpInitStatus = mpu.dmpInitialize();
  uint8_t dmpInitStatus2 = mpu2.dmpInitialize();
//  pinMode(LED_BUILTIN, OUTPUT);
  switch (dmpInitStatus) {
    case 0:
//      Serial.println("success");
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(mpuInterruptPin), mpuIntTriggered, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      //      packetSize = mpu.dmpGetFIFOPacketSize();
      break;
    case 1:
//      Serial.println("Error Code 1: main binary block loading failed");
      break;
    case 2:
//      Serial.println("Error Code 2: configuration block loading failed");
    default:
      break;

  }
  
  switch (dmpInitStatus2) {
    case 0:
      Serial.println("success2");
      mpu2.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(mpuInterruptPin2), mpuIntTriggered2, RISING);
      mpuIntStatus2 = mpu2.getIntStatus();
      dmpReady2 = true;
      //      packetSize = mpu2.dmpGetFIFOPacketSize();
//      Serial.println(packetSize);
      break;
    case 1:
//      Serial.println("Error Code 1: main binary block loading failed2");
      break;
    case 2:
//      Serial.println("Error Code 2: configuration block loading failed2");
    default:
      break;

  }
}

void loop() {
  if (dmpReady) {
//    Serial.println("2");
    while (!mpuIntVal && fifoCount < packetSize) {
//  Serial.println("3");

    }
    mpuIntVal = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        Serial.println("4");

      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
//  Serial.println("5");

      while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
//          Serial.print("6\t");
//          Serial.println(fifoCount);
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
//      Serial.println("7");

  }
  if (dmpReady2) {
    while (!mpuIntVal2 && fifoCount2 < packetSize) {
//      Serial.println(mpuIntVal2);
    }

    mpuIntVal2 = false;
    mpuIntStatus2 = mpu2.getIntStatus();

    fifoCount2 = mpu2.getFIFOCount();

    if ((mpuIntStatus2 & 0x10) || fifoCount2 == 1024) {
      mpu2.resetFIFO();

    } else if (mpuIntStatus2 & 0x02) {

      while (fifoCount2 < packetSize) {
        fifoCount2 = mpu2.getFIFOCount();
        Serial.println(mpu2.getFIFOCount());

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
      Serial.println(ypr2[2] * 180 / M_PI);
//      if(abs(ypr2[0] * 180 / M_PI) > 45){
//        digitalWrite(LED_BUILTIN, HIGH);
//        
//      }else{
//        digitalWrite(LED_BUILTIN, LOW);
//      }
    }
  }
  mpu.resetFIFO();
  mpu2.resetFIFO();

}
