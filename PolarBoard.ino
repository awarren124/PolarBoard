#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "HX711.h"
#include "SoftwareSerial.h"
#include "ODriveArduino.h"
#include "PID_v1.h"

template<class T> inline Print& operator <<(Print &obj,     T arg) {
    obj.print(arg);
    return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
    obj.print(arg, 4);
    return obj;
}


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

int lcDT1 = 4;
int lcCK1 = 5;
int lcDT2 = 6;
int lcCK2 = 7;

HX711 loadCell1(lcDT1, lcCK1);
HX711 loadCell2(lcDT2, lcCK2);

float scaleFactor = 6010;

float totalWeight = 100;
float weightDist = 0;

float weightDistCutoffToHov = .2;
float weightDistCutoffToSwrv = .4;

enum controlMode {
    hoverboard,
    swerve
};


int odrive1RX = 8;
int odrive1TX = 9;
int odrive2RX = 10;
int odrive2TX = 11;

SoftwareSerial odrive1Serial(odrive1TX, odrive1RX);
SoftwareSerial odrive2Serial(odrive2TX, odrive2RX);

ODriveArduino odrive1(odrive1Serial);
ODriveArduino odrive2(odrive2Serial);

SoftwareSerial odriveSerials[2] = {odrive1Serial, odrive2Serial};

float driveVelocityLimit = 22000.0;
float driveCurrentLimit = 11.0;

float turnVelocityLimit = 22000.0;
float turnCurrentLimit = 11.0;

float wheelMultiplier = 42.0;
float leftDriveVelocity = 0.0;
float rightDriveVelocity = 0.0;
float velocityConstant = 500.0;
controlMode mode;

int deltaTime = 0;
unsigned long prevMillis = 0;

void setup() {

    Wire.begin();


    for (int i = 0; i < rollingAvgSize; i++) {
        values1[i] = 0;
        values2[i] = 0;
    }

    Serial.begin(115200);

    gyro1.initialize();
    gyro2.initialize();

    pinMode(LED_BUILTIN, OUTPUT);

    loadCell1.set_scale(scaleFactor);
    loadCell2.set_scale(scaleFactor);
    loadCell1.tare();
    loadCell2.tare();

    odrive1Serial.begin(115200);
    odrive2Serial.begin(115200);

    for (int i = 0; i < 2; ++i) {

        //drive
        odriveSerials[i] << "w axis" << 0 << ".controller.config.vel_limit " << driveVelocityLimit << '\n';
        odriveSerials[i] << "w axis" << 0 << ".motor.config.current_lim " << driveCurrentLimit << '\n';

        //turn
        odriveSerials[i] << "w axis" << 1 << ".controller.config.vel_limit " << turnVelocityLimit << '\n';
        odriveSerials[i] << "w axis" << 1 << ".motor.config.current_lim " << turnCurrentLimit << '\n';
    }

    mode = hoverboard;

}

float floatMap(float input, float inputMin, float inputMax, float outputMin, float outputMax) {
    float slope = (outputMax - outputMin) / (inputMax - inputMin);
    float output = slope * (input - inputMin) + outputMin;
    return output;
}

void loop() {

    deltaTime = millis() - prevMillis;

    angle1 = gyro1.getAccelerationX();
    angle2 = gyro2.getAccelerationX();

    total1 = total1 - values1[index1];
    values1[index1] = angle1;
    total1 = total1 + values1[index1];
    index1++;

    if (index1 >= rollingAvgSize) {
        index1 = 0;
    }

    avg1 = total1 / rollingAvgSize;

    total2 = total2 - values2[index2];
    values2[index2] = angle2;
    total2 = total2 + values2[index2];
    index2++;

    if (index2 >= rollingAvgSize) {
        index2 = 0;
    }

    avg2 = total2 / rollingAvgSize;


    float floatAvg1 = avg1 / 16384.;
    float floatAvg2 = avg2 / 16384.;
    scaledAngle1 = floatMap(floatAvg1, -0.3, 0.3, -1.0, 1.0);
    scaledAngle2 = floatMap(floatAvg2, -0.3, 0.3, -1.0, 1.0);
    Serial.print(scaledAngle1);
    Serial.print("\t");
    Serial.println(scaledAngle2);

    float weight1 = loadCell1.get_units();
    float weight2 = loadCell2.get_units();

    weightDist = (weight1 - weight2) / totalWeight;

    float wheelAngleSetpoint = (scaledAngle1 + scaledAngle2) / 2;

    float newLeftVelocity, newRightVelocity, newVelocity;
    
    switch (mode) {
        case hoverboard:

            if (abs(weightDist) > weightDistCutoffToSwrv && (scaledAngle1 - scaledAngle2) < 0.1) {
                mode = swerve;
                break;
            }

            setAngle(0.0);

            newLeftVelocity = leftDriveVelocity + scaledAngle1 * velocityConstant * deltaTime;
            newRightVelocity = rightDriveVelocity + scaledAngle2 * velocityConstant * deltaTime;

            setVelocity(newLeftVelocity, newRightVelocity);

            break;
        case swerve:

            if (abs(weightDist) < weightDistCutoffToHov) {
                mode = hoverboard;
                break;
            }

            float averageScaledAngle = (scaledAngle1 + scaledAngle2) / 2;
            float angle = atan2(averageScaledAngle, weightDist) / PI;
            setAngle(angle);

            float magnitude = sqrt(pow(averageScaledAngle, 2) + pow(weightDist, 2));

            float newVelocity = ((leftDriveVelocity + rightDriveVelocity) / 2) + magnitude * velocityConstant * deltaTime;
            setVelocity(newVelocity, newVelocity);
            
            break;
    }





    prevMillis = millis();
}


void setAngle(float angle) {
    odrive1.SetPosition(1, angle * wheelMultiplier);
    odrive2.SetPosition(1, angle * wheelMultiplier);
}

void setVelocity(float leftVelocity, float rightVelocity) {
    leftDriveVelocity = leftVelocity;
    rightDriveVelocity = rightDriveVelocity;

    odrive1.SetVelocity(0, leftVelocity);
    odrive2.SetVelocity(0, rightVelocity);
}
