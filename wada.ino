#include <SharpDistSensor.h>
#include <Wire.h>
#include <MPU6050.h>

// ─── MOTOR & ENCODER PINS ──────────────────────────────────
// Left Motor
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 8;
const int leftEncoderPhaseA = 2; // Hardware Interrupt 0
const int leftEncoderPhaseB = 4; 

// Right Motor
const int PWMB = 6;
const int BIN1 = 9;
const int BIN2 = 10;
const int rightEncoderPhaseA = 3; // Hardware Interrupt 1
const int rightEncoderPhaseB = 11;

// Motor speed baseline
const int speed = 60;

// Encoder ticks counters
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ─── HARDWARE SENSORS ──────────────────────────────────────
SharpDistSensor sensorFront(A0, 5);
SharpDistSensor sensorLeft(A2, 5);
SharpDistSensor sensorRight(A1, 5);

float frontDist = 0.0;
float leftDist = 0.0;
float rightDist = 0.0;

// ─── PID CONTROL SYSTEM ────────────────────────────────────
const float Kp = 10.5; 
const float Ki = 0.05; 
const float Kd = 2.0;  

float integralError = 0.0;
float previousError = 0.0;

// ─── GYRO (MPU6050) ────────────────────────────────────────
MPU6050 mpu;
float angleZ       = 0;
float gyroZ_offset = 0;
unsigned long lastTime = 0;
const float Kp_gyro = 2.0; 

// ─── MAZE GRID SETTINGS ────────────────────────────────────
int n = 280;  // Encoder ticks per cell translation step

const byte INF = 255; 
const byte WIDTH = 5;
const byte HEIGHT = 5;
const byte CENTER_X = 2;
const byte CENTER_Y = 2;

byte distances[WIDTH][HEIGHT];
byte walls[WIDTH][HEIGHT];

// Global Direction Mappings [0: North, 1: East, 2: South, 3: West]
const int FORWARD = 0;
const int RIGHT   = 1;
const int BACK    = 2;
const int LEFT    = 3;

const int dx[4] = {0, 1, 0, -1};
const int dy[4] = {1, 0, -1, 0};
const byte wall_bits[4] = {1, 2, 4, 8};

// Robot positional tracking states
byte x = 0;
byte y = 0;
int heading = FORWARD; 

struct Coordinate {
  byte x;
  byte y;
};

// ─── SETUP FUNCTION ────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize pins for motors
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize encoder pins
  pinMode(leftEncoderPhaseA, INPUT);
  pinMode(leftEncoderPhaseB, INPUT);
  pinMode(rightEncoderPhaseA, INPUT);
  pinMode(rightEncoderPhaseB, INPUT);

  // Attach Interrupt routines
  attachInterrupt(digitalPinToInterrupt(leftEncoderPhaseA), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPhaseA), countRight, RISING);

  // Set up infrared sensor baselines
  sensorFront.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sensorLeft.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  sensorRight.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);

  // Initialize IMU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  calibrateGyro(); 
  lastTime = millis();

  // Populate map layout with starting Manhattan distances
  for (int i = 0; i < WIDTH; i++) {
    for (int j = 0; j < HEIGHT; j++) {
      walls[i][j] = 0;
      distances[i][j] = abs(CENTER_X - i) + abs(CENTER_Y - j);
    }
  }
  Serial.println("System Booted successfully. Starting Maze Exploration.");
}

// ─── MAIN execution LOOP ────────────────────────────────────
void loop() {
  // 1. Refresh distance data before making algorithmic choices
  sensorReadings();

  // 2. Goal Validation Check
  if (x == CENTER_X && y == CENTER_Y) {
    Brake();
    Serial.println("Mouse reached center! Exploration Phase Successful.");
    while (1); // Trap execution safely
  }

  // 3. Scan environment walls and update maze memory
  bool wall_added = mapWall();
  
  // 4. Dynamic Reflooding
  if (wall_added) {
    floodfill(CENTER_X, CENTER_Y);
  }

  // 5. Calculate path alternatives and run alignment turns
  turn();

  // 6. Drive execution forward into next cell block
  goForward();
}

// ─── MAZE RUNNER CORE ALGORITHMS ───────────────────────────
bool mapWall() {
  bool wall_added = false;

  // Front Wall Processing
  if (frontDist < 7.0) {
    if ((walls[x][y] & wall_bits[heading]) == 0) {
      update_walls(x, y, heading);
      wall_added = true;
    }
  }
  // Right Wall Processing
  if (rightDist < 14.0) {
    int right_dir = (heading + 1) % 4;
    if ((walls[x][y] & wall_bits[right_dir]) == 0) {
      update_walls(x, y, right_dir);
      wall_added = true;
    }
  }
  // Left Wall Processing
  if (leftDist < 14.0) {
    int left_dir = (heading + 3) % 4;
    if ((walls[x][y] & wall_bits[left_dir]) == 0) {
      update_walls(x, y, left_dir);
      wall_added = true;
    }
  }
  return wall_added;
}

int set_heading() {
  int best_heading = heading;
  int min_dist = INF;

  for (int i = 0; i < 4; i++) {
    // Check if wall exists in that heading direction
    if ((walls[x][y] & wall_bits[i]) != 0) {
      continue;
    }
    
    int nx = x + dx[i];
    int ny = y + dy[i];

    if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
      if (distances[nx][ny] < min_dist) {
        min_dist = distances[nx][ny];
        best_heading = i;
      }
    }
  }
  return best_heading;   
}

void turn() {
  int best_heading = set_heading();
  int turn_diff = (best_heading - heading) % 4;
  
  if (turn_diff < 0) {
    turn_diff += 4; 
  }

  if (turn_diff == 1) {
    Brake();
    turnRight();
    Brake();
  } else if (turn_diff == 2) {
    Brake();
    turnRight();
    Brake();
    turnRight();
    Brake();
  } else if (turn_diff == 3) {
    Brake();
    turnLeft();
    Brake();
  }
  
  heading = best_heading;
}

void goForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  leftTicks = 0;
  rightTicks = 0;
  
  bool stepCompleted = true; 

  while (leftTicks < n && rightTicks < n) {
    updateGyro();
    sensorReadings(); 
    
    // SAFETY GUARD: Emergency active crash avoidance shutdown
    if (frontDist != 999 && frontDist < 4.2) {
      stepCompleted = false; 
      break; 
    }

    float error = centreAdjustment();

    int leftMotorSpeed = speed - error;
    int rightMotorSpeed = speed + error;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(PWMA, leftMotorSpeed);
    analogWrite(PWMB, rightMotorSpeed);
  }

  Brake(); 

  // Location logic handling
  if (stepCompleted) {
    x += dx[heading];
    y += dy[heading];
  } else {
    Serial.println("Emergency stop triggered. Backing up for clearance.");
    backUpALittle(); // FIXED: Dynamically clears spatial conflicts before turning
  }
}

// NEW FUNCTION: Creates physical distance from wall to avoid turn friction stalls
void backUpALittle() {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  
  // Nudge back with enough power to break initial wheel friction
  analogWrite(PWMA, 75); 
  analogWrite(PWMB, 75);
  
  delay(180); // Small, highly controlled duration to step back ~1.5 cm
  
  // Hard clamp brake to kill momentum
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(300);
}

void floodfill(int center_x, int center_y) {
  for (int i = 0; i < WIDTH; i++) {
    for (int j = 0; j < HEIGHT; j++) {
      distances[i][j] = INF;
    }
  }

  distances[center_x][center_y] = 0;
  
  Coordinate customQueue[30]; 
  int head = 0;
  int tail = 0;

  customQueue[tail++] = { (byte)center_x, (byte)center_y };

  while (head < tail) {
    Coordinate current = customQueue[head++];
    int cx = current.x;
    int cy = current.y;

    for (int i = 0; i < 4; i++) {
      if ((walls[cx][cy] & wall_bits[i]) == 0) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
          if (distances[nx][ny] == INF) {
            distances[nx][ny] = distances[cx][cy] + 1;
            customQueue[tail++] = { (byte)nx, (byte)ny };
          }
        }
      }
    }
  }
}

void update_walls(int x, int y, int direction) {
  walls[x][y] |= wall_bits[direction];

  int nx = x + dx[direction];
  int ny = y + dy[direction];

  if (0 <= nx && nx < WIDTH && 0 <= ny && ny < HEIGHT) {
    int opposite_dir = (direction + 2) % 4;
    walls[nx][ny] |= wall_bits[opposite_dir];
  }
}

// ─── HARDWARE SENSOR PID CONTROLS ──────────────────────────
float centreAdjustment() {
  int adjustment = 0;

  if (leftDist < 14.0 && rightDist < 14.0) {
    float error = leftDist - rightDist;
    float P = error * Kp;
    integralError += error;
    integralError = constrain(integralError, -50.0, 50.0); 
    float I = integralError * Ki;
    float derivative = error - previousError;
    previousError = error; 
    float D = derivative * Kd;
    adjustment = P + I + D;
  } 
  else if (leftDist == 999 && rightDist < 14.0) {
    float rightWallError = 4.5 - rightDist; 
    adjustment = rightWallError * Kp;
    integralError = 0; 
    previousError = 0;
  } 
  else if (leftDist < 14.0 && rightDist == 999) {
    float leftWallError = leftDist - 4.5;
    adjustment = leftWallError * Kp;
    integralError = 0; 
    previousError = 0;
  } 
  else {
    adjustment = leftTicks - rightTicks;
    integralError = 0; 
    previousError = 0;
  }
  return adjustment;
}

void sensorReadings() {
  float rawFrontList[5], rawLeftList[5], rawRightList[5];

  for (int i = 0; i < 5; i++) {
    rawFrontList[i] = sensorFront.getDist() / 10.0;
    rawLeftList[i]  = sensorLeft.getDist()  / 10.0;
    rawRightList[i] = sensorRight.getDist() / 10.0;
    delay(1); 
  }

  float sumFront = 0.0, sumLeft = 0.0, sumRight = 0.0;
  for (int i = 0; i < 5; i++) {
    sumFront += rawFrontList[i];
    sumLeft  += rawLeftList[i];
    sumRight += rawRightList[i];
  }

  frontDist = mapFloat((sumFront / 5.0), 2.9, 5.6, 4.0, 7.0);
  leftDist  = mapFloat((sumLeft  / 5.0), 3.0, 6.0, 4.0, 7.0);
  rightDist = mapFloat((sumRight / 5.0), 2.7, 5.1, 3.5, 6.5); 

  if (frontDist > 7.0) { frontDist = 999; }
  if (leftDist > 7.0)  { leftDist = 999;  }
  if (rightDist > 7.0) { rightDist = 999; }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Brake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);

  leftTicks = 0;
  rightTicks = 0;

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(500);
}

// ─── IMU KINEMATICS & TURNS ─────────────────────────────────
void turnRight() {
  angleZ = 0;  
  unsigned long turnStart = millis(); // Track runtime
  lastTime = millis();

  digitalWrite(AIN1, HIGH);   digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);    digitalWrite(BIN2, HIGH);

  leftTicks  = 0;
  rightTicks = 0;

  while (abs(angleZ) < 60.0) {
    // FIXED: Active Timeout protection to break out if wheel/chassis stalls against a corner
    if (millis() - turnStart > 1500) {
      Serial.println("Turn right execution stalled. Safety timeout triggered.");
      break; 
    }

    updateGyro();
    float remaining   = 90.0 - abs(angleZ);
    int currentSpeed  = (remaining < 20.0) ? speed / 2 : speed;

    int error = leftTicks - abs(rightTicks);
    int leftMotorSpeed  = constrain(currentSpeed - error, 0, 255);
    int rightMotorSpeed = constrain(currentSpeed + error, 0, 255);

    analogWrite(PWMA, leftMotorSpeed);
    analogWrite(PWMB, rightMotorSpeed);
  }
  Brake();
}

void turnLeft() {
  angleZ = 0;  
  unsigned long turnStart = millis(); // Track runtime
  lastTime = millis();

  digitalWrite(AIN1, LOW);    digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);   digitalWrite(BIN2, LOW);

  leftTicks  = 0;
  rightTicks = 0;

  while (abs(angleZ) < 60.0) {
    // FIXED: Active Timeout protection to break out if wheel/chassis stalls against a corner
    if (millis() - turnStart > 1500) {
      Serial.println("Turn left execution stalled. Safety timeout triggered.");
      break; 
    }

    updateGyro();
    float remaining   = 90.0 - abs(angleZ);
    int currentSpeed  = (remaining < 20.0) ? speed / 2 : speed;

    int error = abs(leftTicks) - rightTicks;
    int leftMotorSpeed  = constrain(currentSpeed - error, 0, 255);
    int rightMotorSpeed = constrain(currentSpeed + error, 0, 255);

    analogWrite(PWMA, leftMotorSpeed);
    analogWrite(PWMB, rightMotorSpeed);
  }
  Brake();
}

void calibrateGyro() {
  Serial.println("Calibrating... keep robot STILL");
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(3);
  }
  gyroZ_offset = sum / 500.0;
}

void updateGyro() {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float gyroZ_dps = (gz - gyroZ_offset) / 131.0;
  if (abs(gyroZ_dps) < 0.5) gyroZ_dps = 0;  

  angleZ += gyroZ_dps * dt;
}

void adjust() {
  float targetAngle = round(angleZ / 90.0) * 90.0;
  float angleError = targetAngle - angleZ;

  while (abs(angleError) > 1.0) {
    updateGyro(); 
    angleError = targetAngle - angleZ;

    int pivotSpeed = angleError * Kp_gyro;
    
    if (pivotSpeed > 0 && pivotSpeed < 35)  pivotSpeed = 35;
    if (pivotSpeed < 0 && pivotSpeed > -35) pivotSpeed = -35;
    pivotSpeed = constrain(pivotSpeed, -80, 80); 

    if (pivotSpeed > 0) {
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    }

    analogWrite(PWMA, abs(pivotSpeed));
    analogWrite(PWMB, abs(pivotSpeed));
  }
  Brake(); 
}

// ─── ISR INTERRUPTS ────────────────────────────────────────
void countLeft() {
  if (digitalRead(leftEncoderPhaseB) == HIGH) leftTicks++;
  else leftTicks--;
}

void countRight() {
  if (digitalRead(rightEncoderPhaseB) == HIGH) rightTicks++;
  else rightTicks--;
}
