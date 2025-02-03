#include "movement.h"

// ============================
// 1) Definitions of extern vars
// ============================
volatile bool updateAnglesFlag = false;
float angle[3] = {0, 0, 0};
float bias[3]  = {0, 0, 0};
float P[3][2][2] = {
  {{0,0},{0,0}},
  {{0,0},{0,0}},
  {{0,0},{0,0}}
};

float accXOffset=0, accYOffset=0, accZOffset=0;
float gyroXOffset=0, gyroYOffset=0, gyroZOffset=0;

int leftWheelSpeed  = 0;
int rightWheelSpeed = 0;
bool isTurning      = false;
int gridHeading     = 0; // e.g. 0=up, 90=right, 180=down, 270=left

// ============================
// 2) Implement the functions
// ============================
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

void calibrateMPU(int numSamples){
  long accXSum=0, accYSum=0, accZSum=0;
  long gyroXSum=0, gyroYSum=0, gyroZSum=0;

  Serial.println("Calibrating... Keep sensor still.");
  for(int i=0; i<numSamples; i++){
    int16_t rawAccX = readMPURegister16(ACCEL_XOUT_H);
    int16_t rawAccY = readMPURegister16(ACCEL_XOUT_H+2);
    int16_t rawAccZ = readMPURegister16(ACCEL_XOUT_H+4);
    int16_t rawGyroX= readMPURegister16(GYRO_XOUT_H);
    int16_t rawGyroY= readMPURegister16(GYRO_XOUT_H+2);
    int16_t rawGyroZ= readMPURegister16(GYRO_XOUT_H+4);

    accXSum  += rawAccX;
    accYSum  += rawAccY;
    accZSum  += rawAccZ;
    gyroXSum += rawGyroX;
    gyroYSum += rawGyroY;
    gyroZSum += rawGyroZ;

    delay(1);
  }

  float accXMean=(float)accXSum / numSamples;
  float accYMean=(float)accYSum / numSamples;
  float accZMean=(float)accZSum / numSamples;
  float gyroXMean=(float)gyroXSum/ numSamples;
  float gyroYMean=(float)gyroYSum/ numSamples;
  float gyroZMean=(float)gyroZSum/ numSamples;

  accXOffset  = accXMean;
  accYOffset  = accYMean;
  accZOffset  = (accZMean - 16384.f); 
  gyroXOffset = gyroXMean;
  gyroYOffset = gyroYMean;
  gyroZOffset = gyroZMean;
}

void updateAngles(){
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) * 0.001f;
  lastTime = now;

  int16_t rawAccX  = readMPURegister16(ACCEL_XOUT_H);
  int16_t rawAccY  = readMPURegister16(ACCEL_XOUT_H+2);
  int16_t rawAccZ  = readMPURegister16(ACCEL_XOUT_H+4);
  int16_t rawGyroX = readMPURegister16(GYRO_XOUT_H);
  int16_t rawGyroY = readMPURegister16(GYRO_XOUT_H+2);
  int16_t rawGyroZ = readMPURegister16(GYRO_XOUT_H+4);

  float ax = (float)rawAccX - accXOffset;
  float ay = (float)rawAccY - accYOffset;
  float az = (float)rawAccZ - accZOffset;
  float gx = (float)rawGyroX - gyroXOffset;
  float gy = (float)rawGyroY - gyroYOffset;
  float gz = (float)rawGyroZ - gyroZOffset;

  float accelX    = ax / 16384.f;
  float accelY    = ay / 16384.f;
  float accelZ    = az / 16384.f;
  float gyroX_dps = gx / 131.f;
  float gyroY_dps = gy / 131.f;
  float gyroZ_dps = gz / 131.f;

  float accRoll  = atan2(accelY, accelZ) * 180.f / M_PI;
  float accPitch = atan(-accelX / sqrt(accelY*accelY + accelZ*accelZ)) * 180.f / M_PI;

  // roll
  angle[0] = kalmanFilter(accRoll,  gyroX_dps, &bias[0], P[0], dt);
  // pitch
  angle[1] = kalmanFilter(accPitch, gyroY_dps, &bias[1], P[1], dt);
  // yaw => integrate
  angle[2] += (gyroZ_dps * dt);
  if(angle[2]<0)       angle[2]+=360.f;
  if(angle[2]>=360.f)  angle[2]-=360.f;
}

int16_t readMPURegister16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if(Wire.endTransmission(false)!=0){
    Serial.println("MPU write error!");
    return 0;
  }
  Wire.requestFrom((int)MPU_ADDR, 2);
  if(Wire.available()<2){
    Serial.println("MPU read error!");
    return 0;
  }
  int16_t val = (Wire.read()<<8)|Wire.read();
  return val;
}

void zeroYawAfterSettle() {
  Serial.println("Waiting 1s to stabilize...");
  delay(1000);

  updateAngles();
  angle[2]    = 0.0f;
  gridHeading = 0;
  Serial.println("Yaw & gridHeading = 0 after settle.");
}

// Motor control
void leftwheel(int duty,int dir){
  if(dir==FORWARDS){
    leftWheelSpeed=duty;
    analogWrite(IN1,duty);
    analogWrite(IN2,0);
  } else {
    leftWheelSpeed=-duty;
    analogWrite(IN1,0);
    analogWrite(IN2,duty);
  }
}
void rightwheel(int duty,int dir){
  if(dir==FORWARDS){
    rightWheelSpeed=duty;
    analogWrite(IN4,duty);
    analogWrite(IN3,0);
  } else {
    rightWheelSpeed=-duty;
    analogWrite(IN4,0);
    analogWrite(IN3,duty);
  }
}
void straight(int duty){
  leftwheel(duty, FORWARDS);
  rightwheel(duty, FORWARDS);
}
void stopmotors(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  leftWheelSpeed  = 0;
  rightWheelSpeed = 0;
}

// Basic alignment "nudge"
void align(){
  float drift = fmod(angle[2],90.f);
  if(drift>45.f){
    Serial.print("Align => Nudging LEFT. drift=");
    Serial.println(drift);
    int oldLeft=leftWheelSpeed;
    int dir = (oldLeft>=0)? FORWARDS:BACKWARDS;
    leftwheel(oldLeft+50, FORWARDS);
    delay(50);
    leftwheel(abs(oldLeft), dir);
  }
  else if(drift>0.f && drift<45.f){
    Serial.print("Align => Nudging RIGHT. drift=");
    Serial.println(drift);
    int oldRight=rightWheelSpeed;
    int dir = (oldRight>=0)? FORWARDS:BACKWARDS;
    rightwheel(oldRight+50, FORWARDS);
    delay(50);
    rightwheel(abs(oldRight), dir);
  }
}

// Fine alignment + rotate
void fineAlignToTarget(float target) {
  int finalcheck = 0;
  while(true) {
    if(updateAnglesFlag){
      updateAngles();
      updateAnglesFlag=false;
    }
    float error = angle[2] - target;
    if(error>180.f)  error-=360.f;
    if(error<-180.f) error+=360.f;

    float absErr=fabs(error);
    if(absErr<=0.1f) {
      if(finalcheck) break;
      finalcheck=1;
    } else {
      finalcheck=0;
    }
    if(!finalcheck) {
      if(error>0) {
        leftwheel(75, FORWARDS);
        rightwheel(75, BACKWARDS);
      } else {
        leftwheel(75, BACKWARDS);
        rightwheel(75, FORWARDS);
      }
    }
  }
  stopmotors();
  angle[2]=target;
  Serial.println("fineAlignToTarget => within ±0.1°, done!");
}

void rotateRightGrid(int turnAngle){
  isTurning=true;
  updateAngles();

  // e.g. gridHeading is your "grid" direction
  // adjust as needed
  gridHeading=(gridHeading + 360 - turnAngle)%360;
  float target=(float)gridHeading;

  Serial.print("rotateRightGrid => from yaw=");
  Serial.print(angle[2],1);
  Serial.print(" to target=");
  Serial.println(target,1);

  leftwheel(75,FORWARDS);
  rightwheel(75,BACKWARDS);
  while(true){
    if(updateAnglesFlag){
      updateAngles();
      updateAnglesFlag=false;
    }
    float error=fabs(angle[2]-target);
    float altError=360.f - error;
    if(error<2.f || altError<2.f) break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning=false;
  Serial.println("rotateRightGrid => done!");
}

void rotateLeftGrid(int turnAngle){
  isTurning=true;
  updateAngles();

  gridHeading=(gridHeading + turnAngle) % 360;
  float target=(float)gridHeading;

  Serial.print("rotateLeftGrid => from yaw=");
  Serial.print(angle[2],1);
  Serial.print(" to target=");
  Serial.println(target,1);

  rightwheel(75,FORWARDS);
  leftwheel(75,BACKWARDS);
  while(true){
    if(updateAnglesFlag){
      updateAngles();
      updateAnglesFlag=false;
    }
    float error=fabs(angle[2]-target);
    float altError=360.f - error;
    if(error<2.f || altError<2.f) break;
  }
  stopmotors();
  fineAlignToTarget(target);
  isTurning=false;
  Serial.println("rotateLeftGrid => done!");
}
