#include <Wire.h>
#include <SoftwareSerial.h>

// ─── BLUETOOTH (HC-05) ───────────────────────────────────────
const byte BT_RX = 12;
const byte BT_TX = 13;
SoftwareSerial BTSerial(BT_RX, BT_TX);

// ─── MOTOR DRIVER (TB6612FNG) ───────────────────────────────
const byte PWMA = 5;
const byte AIN1 = 7;
const byte AIN2 = 8;

const byte PWMB = 6;
const byte BIN1 = 9;
const byte BIN2 = 10;

// ─── QUADRATURE ENCODERS ────────────────────────────────────
const byte ENC_L_A = 2; // INT0
const byte ENC_R_A = 3; // INT1

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

void leftEncoderISR()  { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

void resetTicks() {
  noInterrupts();
  leftTicks  = 0;
  rightTicks = 0;
  interrupts();
}

// ─── KINEMATICS ─────────────────────────────────────────────
#define TICKS_PER_CELL 605L // 180 mm travel per cell
const byte CELL_CRUISE_PWM = 90;
const byte CELL_MIN_PWM    = 42;

// ─── SENSOR PIN DEFINITIONS ─────────────────────────────────
const byte PIN_IR_FR = A0; // Front Right
const byte PIN_IR_AL = A1; // Angled Left
const byte PIN_IR_AR = A2; // Angled Right
const byte PIN_IR_FL = A3; // Front Left

// ─── CALIBRATION & THRESHOLDS ───────────────────────────────
const int AL_NOMINAL = 168; // Centered 4.5cm setpoint
const int AR_NOMINAL = 151; // Centered 4.5cm setpoint

const int SIDE_WALL_THRESHOLD       = 80;  // > 80 confirms diagonal wall present
const int FRONT_WALL_THRESHOLD      = 120; // Front wall present in front cell (~11cm)
const int FRONT_COLLISION_THRESHOLD = 260; // Abort limit (~5-6cm)

const float IR_KP = 0.22f;

// ─── MAZE BIT-MASK DEFINITIONS ──────────────────────────────
#define WALL_N  0x01
#define WALL_E  0x02
#define WALL_S  0x04
#define WALL_W  0x08
#define VISITED 0x80

const byte DIR_MASK[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
const int8_t DX[4] = { 0,  1,  0, -1 };
const int8_t DY[4] = { 1,  0, -1,  0 };

enum Heading { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

uint8_t maze[16][16];
uint8_t dist[16][16];
byte currentX = 0;
byte currentY = 0;
Heading currentHeading = NORTH;

// Standard Micromouse 2x2 Goal Box
const byte GOAL_MIN_X = 7;
const byte GOAL_MAX_X = 8;
const byte GOAL_MIN_Y = 7;
const byte GOAL_MAX_Y = 8;

// ─── CIRCULAR QUEUE (XY PACKED IN 1 BYTE FOR 16x16) ─────────
uint8_t queue[256];
uint8_t qHead = 0;
uint8_t qTail = 0;

inline void qPush(byte x, byte y) {
  queue[qHead++] = (x << 4) | (y & 0x0F);
}

inline void qPop(byte &x, byte &y) {
  uint8_t val = queue[qTail++];
  x = val >> 4;
  y = val & 0x0F;
}

inline bool qIsEmpty() {
  return qHead == qTail;
}

// ─── ICM-20602 GYROSCOPE REGISTERS ──────────────────────────
const byte ICM_ADDR        = 0x68;
const byte ICM_PWR_MGMT_1  = 0x6B;
const byte ICM_GYRO_CONFIG = 0x1B;
const byte ICM_GYRO_ZOUT_H = 0x47;

const float GYRO_SCALE    = 32.8f;
const float GYRO_DEADBAND = 0.5f;

float angleZ = 0.0f;
float gyroZ_offset = 0.0f;
unsigned long lastGyroTime = 0;

// ─── PD CONTROLLER GAINS ────────────────────────────────────
const float STRAIGHT_KP     = 2.5f;
const float STRAIGHT_KD     = 0.08f;
const float TURN_KP         = 1.8f;
const float TURN_KD         = 0.12f;
const byte  TURN_MAX_PWM    = 110;
const byte  TURN_MIN_PWM    = 45;
const float ANGLE_TOLERANCE = 1.0f;

// ─── FUNCTION PROTOTYPES ────────────────────────────────────
void initICM20602();
void calibrateGyro();
int16_t readGyroZRaw();
float updateGyro();
void brakeMotors();
void turnToAngle(float targetAngle);
void turnRelative(int degrees);
bool advanceCells(uint8_t cells); // fixed: void -> bool
int readCleanADC(byte pin);

void initMaze();
void initDistances();
void setWall(byte x, byte y, byte direction);
void updateCurrentCellWalls();
void floodFill();
Heading getNextMoveDirection();
void orientTo(Heading targetDir);
void moveOneCellForward();
void runFloodFillAutonomous();
void printMazeBT(byte size);

// ─── SETUP ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  brakeMotors();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), rightEncoderISR, RISING);
  resetTicks();

  analogReference(DEFAULT);

  Wire.begin();
  Wire.setClock(400000);

  initICM20602();
  BTSerial.println(F("KEEP ROBOT STILL: Calibrating Gyro..."));
  calibrateGyro();

  initMaze();
  initDistances();

  BTSerial.println(F("\n=================================="));
  BTSerial.println(F("MICROMOUSE READY (FLOOD-FILL)"));
  BTSerial.println(F("=================================="));
  BTSerial.println(F(" 'a' -> Start Full Autonomous Run"));
  BTSerial.println(F(" '1' -> Step 1 Cell (Manual Test)"));
  BTSerial.println(F(" 'l' -> Turn Left 90 deg"));
  BTSerial.println(F(" 'r' -> Turn Right 90 deg"));
  BTSerial.println(F(" 'u' -> Turn 180 deg"));
  BTSerial.println(F(" 'm' -> Print 4x4 Maze"));
  BTSerial.println(F(" 'M' -> Print 16x16 Maze"));
  BTSerial.println(F(" 's' -> Check Sensor Values"));
  BTSerial.println(F(" 'z' -> Reset Heading to 0"));
  BTSerial.println(F("==================================\n"));

  lastGyroTime = micros();
}

// ─── MAIN LOOP ──────────────────────────────────────────────
void loop() {
  updateGyro();

  if (BTSerial.available()) {
    char cmd = BTSerial.read();
    if (cmd == '\r' || cmd == '\n' || cmd < 32) return;

    switch (cmd) {
      case 'a':
      case 'A':
        runFloodFillAutonomous();
        break;

      case '1':
      case 'f':
      case 'F':
        BTSerial.println(F("Advancing 1 Cell & Updating Walls..."));
        moveOneCellForward();
        BTSerial.print(F("Node: ("));
        BTSerial.print(currentX);
        BTSerial.print(F(", "));
        BTSerial.print(currentY);
        BTSerial.print(F(") Dist: "));
        BTSerial.println(dist[currentX][currentY]);
        break;

      case 'l':
      case 'L':
        turnRelative(90);
        updateCurrentCellWalls();
        break;

      case 'r':
      case 'R':
        turnRelative(-90);
        updateCurrentCellWalls();
        break;

      case 'u':
      case 'U':
        turnRelative(180);
        updateCurrentCellWalls();
        break;

      case 'm':
        printMazeBT(4);
        break;

      case 'M':
        printMazeBT(16);
        break;

      case 's':
      case 'S': {
        int al = readCleanADC(PIN_IR_AL);
        int ar = readCleanADC(PIN_IR_AR);
        int fl = readCleanADC(PIN_IR_FL);
        int fr = readCleanADC(PIN_IR_FR);

        BTSerial.print(F("AL: ")); BTSerial.print(al);
        BTSerial.print(F(" | AR: ")); BTSerial.print(ar);
        BTSerial.print(F(" | FL: ")); BTSerial.print(fl);
        BTSerial.print(F(" | FR: ")); BTSerial.print(fr);
        BTSerial.println();
        break;
      }

      case 'z':
      case 'Z':
        angleZ = 0.0f;
        BTSerial.println(F("Heading reset to 0.0 deg"));
        break;
    }
  }
}

// ─── 3-SAMPLE FAST MEDIAN ADC FILTER ────────────────────────
int readCleanADC(byte pin) {
  analogRead(pin);
  delayMicroseconds(40);

  int a = analogRead(pin);
  int b = analogRead(pin);
  int c = analogRead(pin);

  if ((a <= b && b <= c) || (c <= b && b <= a)) return b; // fixed
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a; // fixed
  return c;
}

// ─── MAZE & FLOOD-FILL LOGIC ────────────────────────────────
void initMaze() {
  memset(maze, 0, sizeof(maze));

  for (byte i = 0; i < 16; i++) {
    maze[i][15] |= WALL_N;
    maze[15][i] |= WALL_E;
    maze[i][0]  |= WALL_S;
    maze[0][i]  |= WALL_W;
  }

  maze[0][0] |= (WALL_W | WALL_S | WALL_E | VISITED);
  maze[1][0] |= WALL_W;
  maze[0][1] |= WALL_S;

  currentX = 0;
  currentY = 0;
  currentHeading = NORTH;

  updateCurrentCellWalls();
}

void initDistances() {
  for (byte x = 0; x < 16; x++) {
    for (byte y = 0; y < 16; y++) {
      int distToX = 0;
      if (x < GOAL_MIN_X) distToX = GOAL_MIN_X - x;
      else if (x > GOAL_MAX_X) distToX = x - GOAL_MAX_X;

      int distToY = 0;
      if (y < GOAL_MIN_Y) distToY = GOAL_MIN_Y - y;
      else if (y > GOAL_MAX_Y) distToY = y - GOAL_MAX_Y;

      dist[x][y] = distToX + distToY;
    }
  }
}

void setWall(byte x, byte y, byte direction) {
  maze[x][y] |= DIR_MASK[direction];

  switch (direction) {
    case NORTH:
      if (y < 15) maze[x][y + 1] |= WALL_S;
      break;
    case EAST:
      if (x < 15) maze[x + 1][y] |= WALL_W;
      break;
    case SOUTH:
      if (y > 0)  maze[x][y - 1] |= WALL_N;
      break;
    case WEST:
      if (x > 0)  maze[x - 1][y] |= WALL_E;
      break;
  }
}

void updateCurrentCellWalls() {
  int alVal = readCleanADC(PIN_IR_AL);
  int arVal = readCleanADC(PIN_IR_AR);
  int flVal = readCleanADC(PIN_IR_FL);
  int frVal = readCleanADC(PIN_IR_FR);

  bool leftWall  = (alVal > SIDE_WALL_THRESHOLD);
  bool rightWall = (arVal > SIDE_WALL_THRESHOLD);
  bool frontWall = (flVal > FRONT_WALL_THRESHOLD || frVal > FRONT_WALL_THRESHOLD);

  maze[currentX][currentY] |= VISITED;

  if (frontWall) {
    setWall(currentX, currentY, currentHeading);
  }
  if (rightWall) {
    byte rightDir = (currentHeading + 1) % 4;
    setWall(currentX, currentY, rightDir);
  }
  if (leftWall) {
    byte leftDir = (currentHeading + 3) % 4;
    setWall(currentX, currentY, leftDir);
  }
}

void floodFill() {
  qHead = 0;
  qTail = 0;
  qPush(currentX, currentY);

  while (!qIsEmpty()) {
    byte x, y;
    qPop(x, y);

    if (x >= GOAL_MIN_X && x <= GOAL_MAX_X && y >= GOAL_MIN_Y && y <= GOAL_MAX_Y) {
      continue;
    }

    uint8_t minNeighborDist = 255;
    for (byte d = 0; d < 4; d++) {
      if (!(maze[x][y] & DIR_MASK[d])) {
        byte nx = x + DX[d];
        byte ny = y + DY[d];
        if (nx < 16 && ny < 16) {
          if (dist[nx][ny] < minNeighborDist) {
            minNeighborDist = dist[nx][ny];
          }
        }
      }
    }

    if (dist[x][y] != minNeighborDist + 1) {
      dist[x][y] = minNeighborDist + 1;

      for (byte d = 0; d < 4; d++) {
        if (!(maze[x][y] & DIR_MASK[d])) {
          byte nx = x + DX[d];
          byte ny = y + DY[d];
          if (nx < 16 && ny < 16) {
            qPush(nx, ny);
          }
        }
      }
    }
  }
}

Heading getNextMoveDirection() {
  uint8_t bestDist = 255;
  Heading bestDir = currentHeading;

  byte checkOrder[4] = {
    currentHeading,
    (byte)((currentHeading + 1) % 4), // Right
    (byte)((currentHeading + 3) % 4), // Left
    (byte)((currentHeading + 2) % 4)  // Back
  };

  for (byte i = 0; i < 4; i++) {
    byte d = checkOrder[i];
    if (!(maze[currentX][currentY] & DIR_MASK[d])) {
      byte nx = currentX + DX[d];
      byte ny = currentY + DY[d];
      if (nx < 16 && ny < 16) {
        if (dist[nx][ny] < bestDist) {
          bestDist = dist[nx][ny];
          bestDir = (Heading)d;
        }
      }
    }
  }

  return bestDir;
}

void orientTo(Heading targetDir) {
  int diff = (int)targetDir - (int)currentHeading;

  if (diff == 1 || diff == -3) {
    turnRelative(-90);
  } else if (diff == -1 || diff == 3) {
    turnRelative(90);
  } else if (diff == 2 || diff == -2) {
    turnRelative(180);
  }
}

// void moveOneCellForward() {
//   advanceCells(1);

//   switch (currentHeading) {
//     case NORTH: currentY++; break;
//     case EAST:  currentX++; break;
//     case SOUTH: currentY--; break;
//     case WEST:  currentX--; break;
//   }

//   updateCurrentCellWalls();
// }

void moveOneCellForward() {

  bool completed = advanceCells(1);

  if (!completed) {
    BTSerial.println(F("Move failed - position NOT updated."));
    updateCurrentCellWalls();
    return;
  }

  switch (currentHeading) {
    case NORTH:
      if (currentY < 15) currentY++;
      break;

    case EAST:
      if (currentX < 15) currentX++;
      break;

    case SOUTH:
      if (currentY > 0) currentY--;
      break;

    case WEST:
      if (currentX > 0) currentX--;
      break;
  }

  updateCurrentCellWalls();
}

void runFloodFillAutonomous() {
  BTSerial.println(F("STARTING AUTONOMOUS EXPLORATION..."));

  while (true) {
    updateCurrentCellWalls();

    // Goal reached
    if (currentX >= GOAL_MIN_X && currentX <= GOAL_MAX_X &&
        currentY >= GOAL_MIN_Y && currentY <= GOAL_MAX_Y) {
      brakeMotors();
      BTSerial.println(F("\n>>> GOAL REACHED! <<<"));
      printMazeBT(16);
      break;
    }

    floodFill();
    Heading nextDir = getNextMoveDirection();

    if (nextDir != currentHeading) {
      orientTo(nextDir);
    }

    moveOneCellForward();

    // Emergency abort check via Bluetooth
    if (BTSerial.available()) {
      char abortCmd = BTSerial.read();
      if (abortCmd == 'x' || abortCmd == 'X') {
        brakeMotors();
        BTSerial.println(F("Autonomous Run Aborted by User."));
        break;
      }
    }
  }
}

// ─── ADVANCE CELLS WITH WALL CENTERING ──────────────────────
bool advanceCells(uint8_t cells) {
  long targetTicks = (long)cells * TICKS_PER_CELL;
  bool completed = true;
  
  resetTicks();
  float targetHeading = angleZ;
  lastGyroTime = micros();

  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);

  while (true) {
    long currentL, currentR;
    noInterrupts();
    currentL = leftTicks;
    currentR = rightTicks;
    interrupts();

    long avgTicks = (currentL + currentR) / 2;
    long remainingTicks = targetTicks - avgTicks;

    if (remainingTicks <= 0) {
      break;
    }

    // Debounced front collision safeguard
    int flVal = readCleanADC(PIN_IR_FL);
    int frVal = readCleanADC(PIN_IR_FR);

    if (flVal > FRONT_COLLISION_THRESHOLD || frVal > FRONT_COLLISION_THRESHOLD) {
      int confirmFL = readCleanADC(PIN_IR_FL);
      int confirmFR = readCleanADC(PIN_IR_FR);
      if (confirmFL > FRONT_COLLISION_THRESHOLD || confirmFR > FRONT_COLLISION_THRESHOLD) {
        BTSerial.println(F("Collision Abort: Front Wall!"));
        completed = false;
        break;
      }
    }

    int alVal = readCleanADC(PIN_IR_AL);
    int arVal = readCleanADC(PIN_IR_AR);

    bool hasLeftWall  = (alVal > SIDE_WALL_THRESHOLD);
    bool hasRightWall = (arVal > SIDE_WALL_THRESHOLD);

    float irError = 0.0f;
    if (hasLeftWall && hasRightWall) {
      float leftDev  = (float)(alVal - AL_NOMINAL);
      float rightDev = (float)(arVal - AR_NOMINAL);
      irError = leftDev - rightDev;
    } else if (hasLeftWall) {
      irError = (float)(alVal - AL_NOMINAL) * 1.5f;
    } else if (hasRightWall) {
      irError = -(float)(arVal - AR_NOMINAL) * 1.5f;
    }

    float dps = updateGyro();
    float headingError = targetHeading - angleZ;
    float gyroCorrection = (STRAIGHT_KP * headingError) - (STRAIGHT_KD * dps);

    float irCorrection = IR_KP * irError;
    float steerCorrection = gyroCorrection - irCorrection;

    int baseSpeed = CELL_CRUISE_PWM;
    const long decelZone = 120L;
    if (remainingTicks < decelZone) {
      baseSpeed = map(remainingTicks, 0, decelZone, CELL_MIN_PWM, CELL_CRUISE_PWM);
    }

    int leftSpeed  = baseSpeed - steerCorrection;
    int rightSpeed = baseSpeed + steerCorrection;

    analogWrite(PWMA, constrain(leftSpeed, 0, 255));
    analogWrite(PWMB, constrain(rightSpeed, 0, 255));
  }

  brakeMotors();
  delay(80);

  return completed
}

// ─── PIVOT TURNS ────────────────────────────────────────────
void turnRelative(int degrees) {
  turnToAngle(angleZ + degrees);
  if (degrees == 90) {
    currentHeading = (Heading)((currentHeading + 3) % 4);
  } else if (degrees == -90) {
    currentHeading = (Heading)((currentHeading + 1) % 4);
  } else if (degrees == 180 || degrees == -180) {
    currentHeading = (Heading)((currentHeading + 2) % 4);
  }
}

void turnToAngle(float targetAngle) {
  unsigned long turnStartTime = millis();
  const unsigned long TIMEOUT = 2500;

  while (millis() - turnStartTime < TIMEOUT) {
    float dps = updateGyro();
    float error = targetAngle - angleZ;

    if (abs(error) <= ANGLE_TOLERANCE && abs(dps) < 15.0f) {
      break;
    }

    float output = (TURN_KP * error) - (TURN_KD * dps);
    int speed = abs(output);
    speed = constrain(speed, TURN_MIN_PWM, TURN_MAX_PWM);

    if (output > 0) {
      digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
      digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    } else {
      digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    }

    analogWrite(PWMA, speed);
    analogWrite(PWMB, speed);
  }

  brakeMotors();
  delay(80);
}

// ─── ASCII MAZE VISUALIZER ──────────────────────────────────
void printMazeBT(byte size) {
  BTSerial.println(F("\n=== MAZE MAP ==="));

  for (int y = size - 1; y >= 0; y--) {
    for (byte x = 0; x < size; x++) {
      BTSerial.print(F("+"));
      if (maze[x][y] & WALL_N) {
        BTSerial.print(F("---"));
      } else {
        BTSerial.print(F("   "));
      }
    }
    BTSerial.println(F("+"));

    for (byte x = 0; x < size; x++) {
      if (maze[x][y] & WALL_W) {
        BTSerial.print(F("|"));
      } else {
        BTSerial.print(F(" "));
      }

      if (x == currentX && y == currentY) {
        switch (currentHeading) {
          case NORTH: BTSerial.print(F(" ^ ")); break;
          case EAST:  BTSerial.print(F(" > ")); break;
          case SOUTH: BTSerial.print(F(" v ")); break;
          case WEST:  BTSerial.print(F(" < ")); break;
        }
      } else if (maze[x][y] & VISITED) {
        BTSerial.print(F(" . "));
      } else {
        BTSerial.print(F("   "));
      }
    }

    if (maze[size - 1][y] & WALL_E) {
      BTSerial.println(F("|"));
    } else {
      BTSerial.println(F(" "));
    }
  }

  for (byte x = 0; x < size; x++) {
    BTSerial.print(F("+"));
    if (maze[x][0] & WALL_S) {
      BTSerial.print(F("---"));
    } else {
      BTSerial.print(F("   "));
    }
  }
  BTSerial.println(F("+"));

  BTSerial.print(F("Position: ("));
  BTSerial.print(currentX);
  BTSerial.print(F(", "));
  BTSerial.print(currentY);
  BTSerial.print(F(") Heading: "));
  const char* dirNames[] = {"N", "E", "S", "W"};
  BTSerial.println(dirNames[currentHeading]);
  BTSerial.println(F("================\n"));
}

// ─── ICM-20602 FUNCTIONS ────────────────────────────────────
void initICM20602() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_PWR_MGMT_1);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_GYRO_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission();
}

void calibrateGyro() {
  delay(500);
  long sum = 0;
  const int SAMPLES = 500;

  for (int i = 0; i < SAMPLES; i++) {
    sum += readGyroZRaw();
    delay(2);
  }
  gyroZ_offset = (float)sum / (float)SAMPLES;
}

int16_t readGyroZRaw() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

float updateGyro() {
  unsigned long now = micros();
  float dt = (now - lastGyroTime) * 0.000001f;
  lastGyroTime = now;

  int16_t rawZ = readGyroZRaw();
  float dps = (rawZ - gyroZ_offset) / GYRO_SCALE;

  if (abs(dps) > GYRO_DEADBAND) {
    angleZ += dps * dt;
  }
  return dps;
}

// ─── MOTOR BRAKE PRIMITIVE ──────────────────────────────────
void brakeMotors() {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}