

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Encoder.h>

// Motor driver pins
const int ENA = 10, IN1 = 5, IN2 = 4;
const int ENB = 9, IN3 = 3, IN4 = 2;

// Encoder pins
const int ENC_PIN_A = A1, ENC_PIN_B = A0;
Encoder enc(ENC_PIN_A, ENC_PIN_B);
int cellDistanceTicks = 200; // Adjust after calibration

// Ultrasonic sensors pins
const int TRIG_FRONT = 6, ECHO_FRONT = 7;
const int TRIG_LEFT = 8, ECHO_LEFT = 11;
const int TRIG_RIGHT = 12, ECHO_RIGHT = 13;

// Maze parameters
const int MAZE_SIZE = 10;
const int START_X = 0, START_Y = 0, GOAL_X = 9, GOAL_Y = 9;
const int WALL_DISTANCE_CM = 15;

uint8_t distanceGrid[MAZE_SIZE][MAZE_SIZE];
uint8_t wallsGrid[MAZE_SIZE][MAZE_SIZE];
int currentDirection = 0, posX = START_X, posY = START_Y;

// MPU6050
Adafruit_MPU6050 mpu;
float gyroZoffset = 0, yaw = 0;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1);
  }
  calibration();

  // Initialize maze boundaries
  for (int i = 0; i < MAZE_SIZE; i++) {
    wallsGrid[0][i] |= 0x04; wallsGrid[MAZE_SIZE-1][i] |= 0x01;
    wallsGrid[i][0] |= 0x08; wallsGrid[i][MAZE_SIZE-1] |= 0x02;
  }

  computeDistances(GOAL_X, GOAL_Y);
}

void loop() {
  scanWalls();
  computeDistances(GOAL_X, GOAL_Y);
  decideAndMove();

  if (posX == GOAL_X && posY == GOAL_Y) {
    driveStop();
    Serial.println("Reached Goal!");
    EEPROM.update(0, 0xA5);
    for (int y = 0; y < MAZE_SIZE; y++)
      for (int x = 0; x < MAZE_SIZE; x++)
        EEPROM.update(1 + y * MAZE_SIZE + x, wallsGrid[y][x]);
    while (1);
  }
}

void computeDistances(int goalX, int goalY) {
  for (int y = 0; y < MAZE_SIZE; y++)
    for (int x = 0; x < MAZE_SIZE; x++)
      distanceGrid[y][x] = 255;
  distanceGrid[goalY][goalX] = 0;

  bool updated = true;
  while (updated) {
    updated = false;
    for (int y = 0; y < MAZE_SIZE; y++)
      for (int x = 0; x < MAZE_SIZE; x++) {
        uint8_t minDist = distanceGrid[y][x];
        if (y < MAZE_SIZE-1 && !(wallsGrid[y][x] & 0x01)) minDist = min(minDist, distanceGrid[y+1][x]+1);
        if (x < MAZE_SIZE-1 && !(wallsGrid[y][x] & 0x02)) minDist = min(minDist, distanceGrid[y][x+1]+1);
        if (y > 0 && !(wallsGrid[y][x] & 0x04)) minDist = min(minDist, distanceGrid[y-1][x]+1);
        if (x > 0 && !(wallsGrid[y][x] & 0x08)) minDist = min(minDist, distanceGrid[y][x-1]+1);
        if (distanceGrid[y][x] != minDist) distanceGrid[y][x] = minDist, updated = true;
      }
  }
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.0343 / 2;
}

void scanWalls() {
  int dF = measureDistance(TRIG_FRONT, ECHO_FRONT);
  int dL = measureDistance(TRIG_LEFT, ECHO_LEFT);
  int dR = measureDistance(TRIG_RIGHT, ECHO_RIGHT);

  uint8_t wallBits = 0;
  if (dF < WALL_DISTANCE_CM) wallBits |= (1 << currentDirection);
  if (dR < WALL_DISTANCE_CM) wallBits |= (1 << ((currentDirection+1)%4));
  if (dL < WALL_DISTANCE_CM) wallBits |= (1 << ((currentDirection+3)%4));
  wallsGrid[posY][posX] |= wallBits;
}

void moveForwardOneCell(int speed) {
  enc.write(0);
  driveForward(speed);
  while (abs(enc.read()) < cellDistanceTicks);
  driveStop();
  if (currentDirection == 0) posY++;
  else if (currentDirection == 1) posX++;
  else if (currentDirection == 2) posY--;
  else if (currentDirection == 3) posX--;
}

void driveForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}

void driveStop() { analogWrite(ENA, 0); analogWrite(ENB, 0); }

void calibration() {
  // Simple calibration logic (expand if needed)
  gyroZoffset = 0;
  yaw = 0;
}

void decideAndMove() {
  uint8_t currDist = distanceGrid[posY][posX];
  uint8_t nextDist = 255;

  int nextDir = currentDirection;
  int nx = posX, ny = posY;

  for (int i = 0; i < 4; i++) {
    int dir = (currentDirection + i) % 4;
    int tx = posX, ty = posY;
    if (dir == 0 && ty < MAZE_SIZE-1) ty++;
    else if (dir == 1 && tx < MAZE_SIZE-1) tx++;
    else if (dir == 2 && ty > 0) ty--;
    else if (dir == 3 && tx > 0) tx--;

    if (!(wallsGrid[posY][posX] & (1 << dir)) && distanceGrid[ty][tx] < nextDist) {
      nextDist = distanceGrid[ty][tx];
      nextDir = dir; nx = tx; ny = ty;
    }
  }

  int turnAngle = ((nextDir - currentDirection) + 4) % 4;
  if (turnAngle == 1) currentDirection = (currentDirection + 1) % 4; // Turn right (add motor logic)
  else if (turnAngle == 3) currentDirection = (currentDirection + 3) % 4; // Turn left
  else if (turnAngle == 2) currentDirection = (currentDirection + 2) % 4; // Turn around

  moveForwardOneCell(100);
}
