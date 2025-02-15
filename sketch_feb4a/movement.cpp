#include "movement.h"

// -------------------------
// Global Variable Definitions
// -------------------------
int isreverse = 0;
int camefromadjust = 0;
volatile bool updateAnglesFlag = false;
float angle[3] = {0, 0, 0};      // roll, pitch, yaw
float bias[3]  = {0, 0, 0};
float P[3][2][2] = {
  { {0, 0}, {0, 0} },
  { {0, 0}, {0, 0} },
  { {0, 0}, {0, 0} }
};

float accXOffset = 0, accYOffset = 0, accZOffset = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

int leftWheelSpeed = 0;
int rightWheelSpeed = 0;
bool isTurning = false;
int gridHeading = 0;
int isright = 0;
int isleft = 0;


// -------------------------
// Alignment Nudge Globals (Nonblocking)
// -------------------------
volatile bool alignNudgeActive = false;
volatile unsigned long alignNudgeStartTime = 0;
volatile int alignNudgeSide = 0;       //  1 = left wheel boosted; -1 = right wheel boosted; 0 = none
volatile int originalNudgeSpeed = 0;   // Stores the wheel speed before boost

// -------------------------
// Timer2 Alignment Globals
// -------------------------
volatile bool alignTick = false;       // Set by Timer2 ISR to trigger align processing
volatile uint8_t timer2OverflowCount = 0;  // Count Timer2 overflows

// -------------------------
// Stall Detection Globals
// -------------------------
//
// We'll detect 'movement' by looking at the change in total acceleration
// from the MPU. If it doesn't change beyond a small threshold for 3s
// (while presumably commanded to move), we assume stall.
unsigned long lastMovementTime = 0;
float lastAccelMag = 0;
bool firstAccel = true;

// -------------------------
// IR Sensor Functions
// -------------------------
float approximateDistanceCM(int adcValue) {
  if (adcValue < 1) {  // Too low reading? Return a high distance.
    return 999.9;
  }
  float voltage = adcValue * (5.0 / 1023.0);
  float distance_cm;
  if (voltage < 2.3) {
    distance_cm = voltage * (10.0 / 2.3);  // Adjust factor as needed
  } else {
    distance_cm = voltage * (5.0 / 3.5);   // Adjust factor as needed
  }
  return distance_cm;
}

void getdistances(int* distances) {
  int rawVal1 = analogRead(sensorPin1);
  int rawVal2 = analogRead(sensorPin2);
  distances[0] = (int)approximateDistanceCM(rawVal1);
  distances[1] = (int)approximateDistanceCM(rawVal2);
}

// -------------------------
// Kalman Filter for Angle Estimation
// -------------------------
float kalmanFilter(float newAngle, float newRate, float* b, float PP[2][2], float dt) {
  float rate = newRate - *b;
  float angleKF = newAngle + dt * rate;

  PP[0][0] += dt * (dt * PP[1][1] - PP[0][1] - PP[1][0] + 0.001f);
  PP[0][1] -= dt * PP[1][1];
  PP[1][0] -= dt * PP[1][1];
  PP[1][1] += 0.003f * dt;

  float S = PP[0][0] + 0.03f;
  float K[2];
  K[0] = PP[0][0] / S;
  K[1] = PP[1][0] / S;

  float y = newAngle - angleKF;
  angleKF += K[0] * y;
  *b     += K[1] * y;

  float P00_temp = PP[0][0];
  float P01_temp = PP[0][1];
  PP[0][0] -= K[0] * P00_temp;
  PP[0][1] -= K[0] * P01_temp;
  PP[1][0] -= K[1] * P00_temp;
  PP[1][1] -= K[1] * P01_temp;

  return angleKF;
}

// -------------------------
// MPU & Angle Functions
// -------------------------
void calibrateMPU(int numSamples) {
  long accXSum = 0, accYSum = 0, accZSum = 0;
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  Serial.println("Calibrating... Keep sensor still.");
  for (int i = 0; i < numSamples; i++) {
    int16_t rawAccX = readMPURegister16(ACCEL_XOUT_H);
    int16_t rawAccY = readMPURegister16(ACCEL_XOUT_H + 2);
    int16_t rawAccZ = readMPURegister16(ACCEL_XOUT_H + 4);
    int16_t rawGyroX = readMPURegister16(GYRO_XOUT_H);
    int16_t rawGyroY = readMPURegister16(GYRO_XOUT_H + 2);
    int16_t rawGyroZ = readMPURegister16(GYRO_XOUT_H + 4);

    accXSum  += rawAccX;
    accYSum  += rawAccY;
    accZSum  += rawAccZ;
    gyroXSum += rawGyroX;
    gyroYSum += rawGyroY;
    gyroZSum += rawGyroZ;

    delay(1);
  }

  float accXMean = (float)accXSum / numSamples;
  float accYMean = (float)accYSum / numSamples;
  float accZMean = (float)accZSum / numSamples;
  float gyroXMean = (float)gyroXSum / numSamples;
  float gyroYMean = (float)gyroYSum / numSamples;
  float gyroZMean = (float)gyroZSum / numSamples;

  accXOffset = accXMean;
  accYOffset = accYMean;
  accZOffset = (accZMean - 16384.f);  // Adjust for 1g
  gyroXOffset = gyroXMean;
  gyroYOffset = gyroYMean;
  gyroZOffset = gyroZMean;
}

void updateAngles() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) * 0.001f;
  lastTime = now;

  int16_t rawAccX  = readMPURegister16(ACCEL_XOUT_H);
  int16_t rawAccY  = readMPURegister16(ACCEL_XOUT_H + 2);
  int16_t rawAccZ  = readMPURegister16(ACCEL_XOUT_H + 4);
  int16_t rawGyroX = readMPURegister16(GYRO_XOUT_H);
  int16_t rawGyroY = readMPURegister16(GYRO_XOUT_H + 2);
  int16_t rawGyroZ = readMPURegister16(GYRO_XOUT_H + 4);

  float ax = (float)rawAccX - accXOffset;
  float ay = (float)rawAccY - accYOffset;
  float az = (float)rawAccZ - accZOffset;
  float gx = (float)rawGyroX - gyroXOffset;
  float gy = (float)rawGyroY - gyroYOffset;
  float gz = (float)rawGyroZ - gyroZOffset;

  float accelX = ax / 16384.f;
  float accelY = ay / 16384.f;
  float accelZ = az / 16384.f;
  float gyroX_dps = gx / 131.f;
  float gyroY_dps = gy / 131.f;
  float gyroZ_dps = gz / 131.f;

  float accRoll  = atan2(accelY, accelZ) * 180.f / M_PI;
  float accPitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 180.f / M_PI;

  // Kalman filtering for roll and pitch
  angle[0] = kalmanFilter(accRoll,  gyroX_dps, &bias[0], P[0], dt);
  angle[1] = kalmanFilter(accPitch, gyroY_dps, &bias[1], P[1], dt);

  // Yaw is simply integrated (without filtering)
  angle[2] += (gyroZ_dps * dt);
  if (angle[2] < 0)       angle[2] += 360.f;
  if (angle[2] >= 360.f)  angle[2] -= 360.f;
}

int16_t readMPURegister16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("MPU write error!");
    return 0;
  }
  Wire.requestFrom((int)MPU_ADDR, 2);
  if (Wire.available() < 2) {
    Serial.println("MPU read error!");
    return 0;
  }
  int16_t val = (Wire.read() << 8) | Wire.read();
  return val;
}

void zeroYawAfterSettle() {
  Serial.println("Waiting 1s to stabilize...");
  delay(1000);
  updateAngles();
  angle[2] = 0.0f;
  gridHeading = 0;
  Serial.println("Yaw & gridHeading = 0 after settle.");
}

// -------------------------
// Motor Control Functions
// -------------------------
void leftwheel(int duty, int dir) {
  if (dir == FORWARDS) {
    leftWheelSpeed = duty;
    analogWrite(IN1, duty);
    analogWrite(IN2, 0);
  } else {
    leftWheelSpeed = -duty;
    analogWrite(IN1, 0);
    analogWrite(IN2, duty);
  }
}

void rightwheel(int duty, int dir) {
  if (dir == FORWARDS) {
    rightWheelSpeed = duty;
    analogWrite(IN4, duty);
    analogWrite(IN3, 0);
  } else {
    rightWheelSpeed = -duty;
    analogWrite(IN4, 0);
    analogWrite(IN3, duty);
  }
}

void straight(int duty) {
  leftwheel(duty, FORWARDS);
  rightwheel(duty, FORWARDS);
}

void stopmotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  leftWheelSpeed = 0;
  rightWheelSpeed = 0;
}

// -------------------------
// Alignment Functions (Nonblocking)
// -------------------------
void align() {
  if (isTurning || alignNudgeActive) {
    return;
  }

  int distances[2];
  getdistances(distances);
  float leftDist  = distances[0];
  float rightDist = distances[1];
  
  Serial.print("Left IR: ");
  Serial.print(leftDist);
  Serial.print(" cm, Right IR: ");
  Serial.print(rightDist);
  Serial.println(" cm");
  
  float error = leftDist - rightDist;
  const float threshold = 2.0;  // cm threshold to trigger a nudge
  
  if (fabs(error) > threshold) {
    if (error > 0) {
      // Robot is too close to the right wall → boost left wheel
      alignNudgeSide = 1; 
      originalNudgeSpeed = leftWheelSpeed;
      leftwheel(leftWheelSpeed + NUDGE_BOOST, FORWARDS);
      Serial.print("Align: Nudging LEFT, error = ");
      Serial.println(error);
    } else {
      // Robot is too close to the left wall → boost right wheel
      alignNudgeSide = -1;
      originalNudgeSpeed = rightWheelSpeed;
      rightwheel(rightWheelSpeed + NUDGE_BOOST, FORWARDS);
      Serial.print("Align: Nudging RIGHT, error = ");
      Serial.println(error);
    }
    alignNudgeActive = true;
    alignNudgeStartTime = millis();
  } else {
    Serial.println("Align: Lane is centered.");
  }
}

void checkAlignNudge() {
  if (alignNudgeActive) {
    unsigned long currentTime = millis();
    if (currentTime - alignNudgeStartTime >= NUDGE_DURATION_MS) {
      if (alignNudgeSide == 1) {
        leftwheel(originalNudgeSpeed, FORWARDS);
      } else if (alignNudgeSide == -1) {
        rightwheel(originalNudgeSpeed, FORWARDS);
      }
      alignNudgeActive = false;
      alignNudgeSide = 0;
      Serial.println("Align: Nudge complete, wheel speed restored.");
    }
  }
}

void fineAlignToTarget(float target) {
  int finalcheck = 0;
  while (true) {
    if (updateAnglesFlag) {
      camefromadjust = 1;
      updateAngles();
      checkStall();
      updateMovementTimeFromMPU();
      camefromadjust = 0;
      updateAnglesFlag = false;
    }
    float error = angle[2] - target;
    if (error > 180.f)  error -= 360.f;
    if (error < -180.f) error += 360.f;
    
    float absErr = fabs(error);
    if (absErr <= 0.1f) {
      if (finalcheck)
        break;
      finalcheck = 1;
    } else {
      finalcheck = 0;
    }
    
    if (!finalcheck) {
      if (error > 0) {
        leftwheel(TURNING, FORWARDS);
        rightwheel(TURNING, BACKWARDS);
      } else {
        leftwheel(TURNING, BACKWARDS);
        rightwheel(TURNING, FORWARDS);
      }
    }
  }
  stopmotors();
  while (!updateAnglesFlag) {}
  if (updateAnglesFlag) {
    updateAngles();
    updateAnglesFlag = false;
  }
  float error = angle[2] - target;
  if (error > 180.f)  error -= 360.f;
  if (error < -180.f) error += 360.f;
  
  float absErr = fabs(error);
  if (absErr <= 0.1f) {
    fineAlignToTarget(target);
  }
  angle[2] = target;
  Serial.println("fineAlignToTarget: within ±0.1°, done!");
}

void rotateRightGrid(int turnAngle) {
  isright = 1;
  isTurning = true;
  updateAngles();
  
  gridHeading = (gridHeading + 360 - turnAngle) % 360;
  float target = (float)gridHeading;
  isreverse = 0;
  Serial.print("rotateRightGrid: from yaw=");
  Serial.print(angle[2], 1);
  Serial.print(" to target=");
  Serial.println(target, 1);
  
  leftwheel(TURNING + 20, FORWARDS);
  rightwheel(TURNING, BACKWARDS);
  while (true) {
    leftwheel(TURNING + 20, FORWARDS);
    rightwheel(TURNING, BACKWARDS);
    if (updateAnglesFlag) {
      updateAngles();
      checkStall();
      updateMovementTimeFromMPU();
      updateAnglesFlag = false;
    }
    float error = fabs(angle[2] - target);
    float altError = 360.f - error;
    if (error < 2.f || altError < 2.f)
      break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning = false;
  Serial.println("rotateRightGrid: done!");
  isright = 0;
}

void turnAround() {
  int turnAngle = 180;
  isTurning = true;
  updateAngles();
  
  gridHeading = (gridHeading + 360 - turnAngle) % 360;
  float target = (float)gridHeading;
  
  Serial.print("rotateRightGrid: from yaw=");
  Serial.print(angle[2], 1);
  Serial.print(" to target=");
  Serial.println(target, 1);
  
  leftwheel(75, FORWARDS);
  rightwheel(75, BACKWARDS);
  while (true) {
    if (updateAnglesFlag) {
      updateAngles();
      updateAnglesFlag = false;
    }
    float error = fabs(angle[2] - target);
    float altError = 360.f - error;
    if (error < 2.f || altError < 2.f)
      break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning = false;
  Serial.println("rotateRightGrid: done!");
}

void rotateLeftGrid(int turnAngle) {
  isleft = 1;
  isTurning = true;
  updateAngles();
  isreverse = 0;
  gridHeading = (gridHeading + turnAngle) % 360;
  float target = (float)gridHeading;
  
  Serial.print("rotateLeftGrid: from yaw=");
  Serial.print(angle[2], 1);
  Serial.print(" to target=");
  Serial.println(target, 1);
  
  rightwheel(TURNING + 20, FORWARDS);
  leftwheel(TURNING, BACKWARDS);
  while (true) {
    rightwheel(TURNING + 20, FORWARDS);
    leftwheel(TURNING, BACKWARDS);
    if (updateAnglesFlag) {
      updateAngles();
      updateMovementTimeFromMPU();
      checkStall();
      updateAnglesFlag = false;
    }
    float error = fabs(angle[2] - target);
    float altError = 360.f - error;
    if (error < 2.f || altError < 2.f)
      break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning = false;
  Serial.println("rotateLeftGrid: done!");
  isleft = 0;
}

void backwards() {
  while (digitalRead(LEFT) == LOW && digitalRead(RIGHT) == LOW) {
    leftwheel(FD, BACKWARDS);
    rightwheel(FD, BACKWARDS);
  }
  stopmotors();
  float start = millis();
  float current = millis();
  while (current - start < 300) {
    rightwheel(FD, BACKWARDS);
    leftwheel(FD, BACKWARDS);
    current = millis();
  }
  isreverse = 1;
}

void turnToHeading(int direction) {
  int targetHeading;
  if (direction == NT) {
    targetHeading = 0;
  } else if (direction == ET) {
    targetHeading = 90;
  } else if (direction == ST) {
    targetHeading = 180;
  } else if (direction == WT) {
    targetHeading = 270;
  } else {
    Serial.println("Invalid direction! Use NORTH, EAST, SOUTH, or WEST.");
    return;
  }

  updateAngles();
  int currentHeading = gridHeading;
  int turnAngle = (targetHeading - currentHeading + 360) % 360;

  if (turnAngle == 0) {
    Serial.println("Already facing the correct direction.");
    return;
  } else if (turnAngle == 180) {
    turnAround();
  } else if (turnAngle < 180) {
    rotateLeftGrid(turnAngle);
  } else {
    rotateRightGrid(360 - turnAngle);
  }

  gridHeading = targetHeading;
  Serial.print("Turned to heading: ");
  Serial.println(direction);
}

// -------------------------
// Timer2 Setup & Processing for Alignment
// -------------------------
ISR(TIMER2_OVF_vect) {
  timer2OverflowCount++;
  if (timer2OverflowCount >= 3) {  // ~3 * 16.384ms ≈ 50ms
    timer2OverflowCount = 0;
    if (!isTurning) {
      alignTick = true;
    }
  }
}

void setupTimer2() {
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  // Set Timer2 prescaler to 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  // Enable Timer2 overflow interrupt
  TIMSK2 |= (1 << TOIE2);
  sei();
}

void processAlignment() {
  if (alignTick) {
    alignTick = false;
    align();
    checkAlignNudge();
  }
}

// -------------------------------------------------
// NEW: Stall Detection via MPU Accelerometer
// -------------------------------------------------
/**
 * updateMovementTimeFromMPU() - Reads the accelerometer data, computes its
 * magnitude, and if the change from the last reading exceeds a small threshold,
 * updates lastMovementTime = millis().
 */
void updateMovementTimeFromMPU() {
  // Read raw accelerometer values (using the same register offsets).
  Serial.println("here");
  int16_t rawAccX  = readMPURegister16(ACCEL_XOUT_H);
  int16_t rawAccY  = readMPURegister16(ACCEL_XOUT_H + 2);
  int16_t rawAccZ  = readMPURegister16(ACCEL_XOUT_H + 4);

  float ax = ((float)rawAccX - accXOffset) / 16384.0f;
  float ay = ((float)rawAccY - accYOffset) / 16384.0f;
  float az = ((float)rawAccZ - accZOffset) / 16384.0f;

  // Calculate the total acceleration magnitude (in g's).
  float accelMag = sqrt(ax*ax + ay*ay + az*az);

  // Initialize on first run
  if (firstAccel) {
    lastAccelMag = accelMag;
    lastMovementTime = millis();
    firstAccel = false;
    return;
  }

  // Threshold for a "change" in acceleration
  const float ACC_CHANGE_THRESHOLD = 0.1;

  if (fabs(accelMag - lastAccelMag) > ACC_CHANGE_THRESHOLD) {
    lastMovementTime = millis();
  }
  lastAccelMag = accelMag;
}

void back() {
  float start = millis();
  while (millis() - start < 500) {
    rightwheel(FD, BACKWARDS);
    leftwheel(FD, BACKWARDS);
  }
  stopmotors();
}

int rotateRightGridFIX(int turnAngle) {
  isright = 1;
  isTurning = true;
  updateAngles();
  
  gridHeading = (gridHeading + 360 - turnAngle) % 360;
  float target = (float)gridHeading;
  isreverse = 0;
  Serial.print("rotateRightGrid: from yaw=");
  Serial.print(angle[2], 1);
  Serial.print(" to target=");
  Serial.println(target, 1);
  
  leftwheel(TURNING + 20, FORWARDS);
  rightwheel(TURNING, BACKWARDS);
  float starttime = millis();
  while (true) {
    if (updateAnglesFlag) {
      updateAngles();
      checkStall();
      updateMovementTimeFromMPU();
      updateAnglesFlag = false;
    }
    float error = fabs(angle[2] - target);
    float altError = 360.f - error;
    if (error < 2.f || altError < 2.f)
      break;
    if (millis() - starttime > 500) {
      return 1;
    }
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning = false;
  Serial.println("rotateRightGrid: done!");
  isright = 0;
  return 0;
}

void rotateLeftGridFIX(int turnAngle) {
  isleft = 1;
  isTurning = true;
  updateAngles();
  isreverse = 0;
  gridHeading = (gridHeading + turnAngle) % 360;
  float target = (float)gridHeading;
  
  Serial.print("rotateLeftGrid: from yaw=");
  Serial.print(angle[2], 1);
  Serial.print(" to target=");
  Serial.println(target, 1);
  
  rightwheel(TURNING + 20, FORWARDS);
  leftwheel(TURNING, BACKWARDS);
  float starttime = millis();

  while (true) {
    if (updateAnglesFlag) {
      updateAngles();
      updateMovementTimeFromMPU();
      checkStall();
      updateAnglesFlag = false;
    }
    float error = fabs(angle[2] - target);
    float altError = 360.f - error;
    if (millis() - starttime > 500) {
      return 1;
    }
    if (error < 2.f || altError < 2.f)
      break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning = false;
  Serial.println("rotateLeftGrid: done!");
  isleft = 0;
}


void fd() {
  float start = millis();
  while (millis() - start < 300) {
    rightwheel(FD, FORWARDS);
    leftwheel(FD, FORWARDS);
  }
  stopmotors();
}
/**
 * checkStall() - If more than 3s pass without movement, assume stall.
 * Reverse the robot, then rotate it by 10° to try to free it.
 */
void checkStall() {
  Serial.println(lastMovementTime);
  if (millis() - lastMovementTime >= 500) { // 3 seconds
    Serial.println("Stall detected: Reversing and turning 10 degrees.");

    // Reverse
    back();
    if (!camefromadjust) {
      fineAlignToTarget(gridHeading);
    }

    if (isTurning) {
      fd();
    }

    // Reset timer to avoid repeated triggers
    lastMovementTime = millis();
  }
}
