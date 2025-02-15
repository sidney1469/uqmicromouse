#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include "movement.h"

// Maze sensor digital pins
#define FRONT 8
#define LEFT  12
#define RIGHT 2

// IR sensor analog pins (for lane–alignment)
#define sensorPin1 A0
#define sensorPin2 A1

// Headings
#define NORTH 0
#define EAST  90
#define WEST  270
#define SOUTH 180


int numloops = 0;

// -------------------------
// Timer2 Compare Interrupt for MPU updates (~100 Hz)
// -------------------------
ISR(TIMER2_COMPA_vect) {
  updateAnglesFlag = true;
  // (Additional periodic tasks can be inserted here if needed.)
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== STARTING MAZE NAVIGATION SKETCH ===");

  Wire.begin();
  Serial.println("I2C initialized");

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set maze sensor pins as inputs
  pinMode(FRONT, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);

  // --- Set up Timer2 for ~100 Hz angle updates ---
  cli(); // Disable interrupts
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 156;                // For ~100 Hz on a 16MHz clock w/ 1024 prescaler
  TCCR2A |= (1 << WGM21);     // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK2 |= (1 << OCIE2A);    // Enable Timer2 compare match A interrupt
  sei(); // Re-enable interrupts
  Serial.println("Timer2 set up for MPU updates (~100 Hz)");

  // Initialize MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);  // Wake up MPU6050
  uint8_t error = Wire.endTransmission();
  Serial.print("MPU6050 init result: ");
  Serial.println(error);
  if (error != 0) {
    Serial.println("MPU6050 initialization failed. Check wiring/address!");
    while (1); // Halt if initialization fails
  }

  delay(100);
  Serial.println("Calibrating MPU...");
  calibrateMPU(500);
  zeroYawAfterSettle();
  Serial.println("MPU calibrated and yaw zeroed.");
  updateMovementTimeFromMPU();

  Serial.println("=== SETUP COMPLETE ===");
}

void loop() {
  // Update MPU angles if signaled.
  if (updateAnglesFlag) {
    updateAngles();
    updateAnglesFlag = false;
    
    // Example: periodically do a fine alignment if close to target.
    if (numloops > 10) {
      updateMovementTimeFromMPU();
      checkStall();
      Serial.print("Aligning lol");
    }
    numloops++;
  }

  // Read digital sensors
  int frontSensor = digitalRead(FRONT);
  int leftSensor  = digitalRead(LEFT);
  int rightSensor = digitalRead(RIGHT);

  // Naïve maze–search logic
  if (frontSensor == HIGH && !isreverse) {
    Serial.println("Path clear. Driving straight.");
    
    straight(FD);
  } else {
    stopmotors();
    if (leftSensor == HIGH) {
      rotateLeftGrid(90);
    } else if (rightSensor == HIGH) {
      rotateRightGrid(90);
    } else {
      // All blocked: attempt reverse
      backwards();
    }
  }

  // 1) Update movement timestamp from MPU (accelerometer).

  // For debugging: print current yaw
  Serial.print("Current Yaw: ");
  Serial.println(angle[2], 1);

  // Optionally, handle alignment every ~50ms
  processAlignment();
}
