#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 gyro1;
MPU6050 gyro2(0x69);

int16_t angle1;

const int rollingAvgSize = 200;

int16_t values1[rollingAvgSize];
int16_t values2[rollingAvgSize];

int16_t angle2;

int index1 = 0;
long total1 = 0;
int16_t avg1 = 0;
int index2 = 0;
long total2 = 0;
int16_t avg2 = 0;

float scaledAngle1 = 0;
float scaledAngle2 = 0;

int minVal = 265;
int maxVal = 402;

void setup() {

    Wire.begin();


    for(int i = 0; i < rollingAvgSize; i++){
      values1[i] = 0;
      values2[i] = 0;
    }

    Serial.begin(38400);

    gyro1.initialize();
    gyro2.initialize();

    pinMode(LED_BUILTIN, OUTPUT);

}

float floatMap(float input, float inputMin, float inputMax, float outputMin, float outputMax){
  float slope = (outputMax - outputMin) / (inputMax - inputMin);
  float output = slope * (input - inputMin) + outputMin;
  return output;
}

void loop() {


   
    
    angle1 = gyro1.getAccelerationX();
    angle2 = gyro2.getAccelerationX();

    total1 = total1 - values1[index1];
    values1[index1] = angle1;
    total1 = total1 + values1[index1];
    index1++;

    if(index1 >= rollingAvgSize){
      index1 = 0;
    }

    avg1 = total1/rollingAvgSize;

    total2 = total2 - values2[index2];
    values2[index2] = angle2;
    total2 = total2 + values2[index2];
    index2++;

    if(index2 >= rollingAvgSize){
      index2 = 0;
    }

    avg2 = total2/rollingAvgSize;

 
    float floatAvg1 = avg1/16384.;
    float floatAvg2 = avg2/16384.;
    scaledAngle1 = floatMap(floatAvg1, -0.3, 0.3, -1.0, 1.0);
    scaledAngle2 = floatMap(floatAvg2, -0.3, 0.3, -1.0, 1.0);
    Serial.print(scaledAngle1);
    Serial.print("\t");
    Serial.println(scaledAngle2);

    if(scaledAngle1 > 1.0){
      digitalWrite(LED_BUILTIN, HIGH);
    }else{
      digitalWrite(LED_BUILTIN, LOW);
    }

}

