#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================
// 1) Global Constants & Pins
// ============================
#define MPU_ADDR     0x68
#define WHO_AM_I     0x75
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

#define IN1  5
#define IN2  6
#define IN3  11
#define IN4  3

#define FORWARDS   1
#define BACKWARDS  0

// ============================
// 2) External Global Variables
//    (We declare them here, then
//     actually define them in .ino.)
// ============================
extern volatile bool updateAnglesFlag;
extern float angle[3]; 
extern float bias[3];  
extern float P[3][2][2];

extern float accXOffset;
extern float accYOffset;
extern float accZOffset;
extern float gyroXOffset;
extern float gyroYOffset;
extern float gyroZOffset;

extern int leftWheelSpeed;  
extern int rightWheelSpeed; 
extern bool isTurning;
extern int gridHeading;    // e.g. 0=up, 90=right, 180=down, 270=left

// ============================
// 3) Function Prototypes
// ============================

// --- Kalman / IMU ---
float  kalmanFilter(float newAngle, float newRate, float* b, float PP[2][2], float dt);
void   calibrateMPU(int numSamples);
void   updateAngles();
int16_t readMPURegister16(uint8_t reg);
void   zeroYawAfterSettle();

// --- Motor Control ---
void leftwheel(int duty, int dir);
void rightwheel(int duty, int dir);
void straight(int duty);
void stopmotors();
void align();

// --- Turning ---
void fineAlignToTarget(float target);
void rotateRightGrid(int turnAngle);
void rotateLeftGrid(int turnAngle);

#endif
