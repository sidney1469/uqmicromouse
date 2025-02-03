#include <Wire.h>
#include <math.h> // For atan2, sqrt, fmod
#include <prioritystack.h>
#include <movement.h>

// ============================
// Maze & DFS Definitions
// ============================
#define MAZEX 9
#define MAZEY 9

// Priority-based stack for DFS
PriorityStack order;

// Maze array: 0 = unvisited, 1 = visited
int Maze[MAZEX][MAZEY] = {0}; 
int currentx = 0;
int currenty = 0;

// Timer1 ISR (example at ~100 Hz)
ISR(TIMER1_COMPA_vect){
  updateAnglesFlag = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // If needed, setup Timer1 for ~100 Hz:
  noInterrupts();
  TCCR1A = 0; 
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 2499; 
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);  // prescaler 64
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  // Initialize / Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Calibrate IMU
  calibrateMPU(2000);
  Serial.println("Calibration done!");

  // Zero yaw
  zeroYawAfterSettle();
  Serial.println("Setup complete!");

  DFS();
}

void DFS() {

}

void loop() {
  // Example usage:
  // if(updateAnglesFlag) {
  //    updateAngles();
  //    updateAnglesFlag = false;
  // }
  //
  // your code that uses motor functions, turning, etc.
}

