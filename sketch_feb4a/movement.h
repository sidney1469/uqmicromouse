#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>  // Needed for ISR definitions

/*struct Node {
    int value;          // Data stored in the node
    bool visited;       // Flag to track if the node is visited
    Node* WEST;         // Pointer to the west child
    Node* EAST;        // Pointer to the east child
    Node* NORTH;           // Pointer to the north node (if needed)
    Node* SOUTH;         // Pointer to the south node (if needed)

    // Constructor to initialize the node
    Node(int val) {
        value = val;
        visited = false;
        left = right = up = down = nullptr;
    }
};*/

// Turn headings for map
#define NT 0
#define ET 1
#define WT 2
#define ST 3

// Maze sensor digital pins
#define FRONT 8          // Digital pin for the front sensor
#define LEFT  12         // Digital pin for the left sensor
#define RIGHT 2          // Digital pin for the right sensor

// -------------------------
// Motor Control Definitions
// -------------------------
#define IN1 5
#define IN2 6
#define IN3 10
#define IN4 9

#define PWR_MGMT_1 0x6B   // Power management register

// Motor direction definitions
#define FORWARDS  1
#define BACKWARDS 0

#define TURNING 100
#define FD 100

// -------------------------
// IR Sensor Definitions
// -------------------------
const int sensorPin1 = A0;  // Left IR sensor
const int sensorPin2 = A1;  // Right IR sensor

// -------------------------
// MPU Definitions
// -------------------------
#define MPU_ADDR      0x68
#define ACCEL_XOUT_H  0x3B
#define GYRO_XOUT_H   0x43

// -------------------------
// Global Variable Declarations
// -------------------------
extern volatile bool updateAnglesFlag;  // Set externally when new IMU data is available

// Angle (roll, pitch, yaw) and Kalman filter bias arrays
extern float angle[3];
extern float bias[3];
// Kalman filter error covariance matrices for roll, pitch, yaw
extern float P[3][2][2];


extern float accXOffset, accYOffset, accZOffset;
extern float gyroXOffset, gyroYOffset, gyroZOffset;

extern int leftWheelSpeed;
extern int rightWheelSpeed;
extern bool isTurning;
extern int gridHeading;  // e.g., 0=up, 90=right, etc.

// -------------------------
// Alignment Nudge (nonblocking) variables & constants
// -------------------------
// (These variables are declared as extern in the header if you wish to inspect them elsewhere.)
extern volatile bool alignNudgeActive;      // True when a temporary speed boost is in progress
extern volatile unsigned long alignNudgeStartTime; // When the nudge began (millis)
extern volatile int alignNudgeSide;           //  1 = left wheel boosted; -1 = right wheel boosted; 0 = none
extern volatile int originalNudgeSpeed;       // Stored wheel speed before boost

// Duration (in ms) for the temporary nudge and boost amount:
#define NUDGE_DURATION_MS 50
#define NUDGE_BOOST       20

// -------------------------
// Timer2 Alignment Flag
// -------------------------
extern volatile bool alignTick;             // Set by Timer2 ISR to trigger align processing
extern int isreverse;

// -------------------------
// Function Prototypes
// -------------------------
float kalmanFilter(float newAngle, float newRate, float* b, float PP[2][2], float dt);
void calibrateMPU(int numSamples);
void updateAngles();
int16_t readMPURegister16(uint8_t reg);
void zeroYawAfterSettle();
void checkStall();
void updateMovementTimeFromMPU();

void leftwheel(int duty, int dir);
void rightwheel(int duty, int dir);
void straight(int duty);
void stopmotors();
void backwards();
void align();                     // Nonblocking alignment using IR sensors
void checkAlignNudge();           // Checks if nudge duration is over and restores wheel speed
void fineAlignToTarget(float target);
void rotateRightGrid(int turnAngle);
void rotateLeftGrid(int turnAngle);
void turnAround();

// IR Sensor helper functions
float approximateDistanceCM(int adcValue);
void getdistances(int* distances);

// Timer2 setup and processing (for nonblocking align)
void setupTimer2();
void processAlignment();

#endif // MOVEMENT_H
