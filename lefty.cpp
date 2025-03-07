// ======================
// Global Definitions and Includes
// ======================
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> 
#include <Wire.h>
#include <EEPROM.h>
#include <Ultrasonic.h>
#include <Encoder.h>

// ======================
// Hardware Pin Assignments
// ======================
// Rotary Encoder pins
const int ENC_PIN_A = A1;
const int ENC_PIN_B = A0;

// Motor driver pins (assuming dual H-bridge for two motors)
const int ENA = 10;   // Left motor PWM
const int IN1 = 5;    // Left motor direction
const int IN2 = 4;
const int ENB = 9;   // Right motor PWM
const int IN3 = 3;   // Right motor direction
const int IN4 = 2;

// Ultrasonic sensor pins (3 sensors: front, left, right)
const int TRIG_FRONT = 6;
const int ECHO_FRONT = 7;
const int TRIG_LEFT  = 8;
const int ECHO_LEFT  = 11;
const int TRIG_RIGHT = 12;
const int ECHO_RIGHT = 13;

// ======================
// Maze & Competition Parameters
// ======================
const int MAZE_SIZE = 8;          // 8 x 8 grid
const int CELL_SIZE_CM = 25;      // Each cell is 25cm x 25cm

// Starting and goal positions
const int START_X = 0;
const int START_Y = 0;
const int GOAL_X  = 7;
const int GOAL_Y  = 7;

const int MAX_DISTANCE = 255;
const int WALL_DISTANCE_CM = 15;  // Threshold to detect a wall

// For encoder-based travel (if used)
int cellDistanceTicks = 200; // Example value; adjust as needed

// Global maze arrays (for mapping if needed)
int posX = START_X, posY = START_Y;  
uint8_t wallsGrid[MAZE_SIZE][MAZE_SIZE] = {0}; 
uint8_t distanceGrid[MAZE_SIZE][MAZE_SIZE];

// ======================
// Other Global Variables
// ======================
int currentDirection = 0;  // 0 = North, 1 = East, 2 = South, 3 = West
int baseSpeed = 100;       // Speed for motion commands

// ======================
// Sensors, Encoders, and Other Objects
// ======================
Adafruit_MPU6050 mpu;
Encoder enc(ENC_PIN_A, ENC_PIN_B);

// ======================
// Function Prototypes
// ======================
long measureDistance(int trigPin, int echoPin);
void driveStop();
void driveForward(int speed);
void driveTurnLeft(int speed);
void driveTurnRight(int speed);
void updatePosition(int dx, int dy);
void turnByDegrees(int degrees, int speed);
void moveForwardOneCell(int speed);
void updateDirection(int degrees);
void updatePositionAccordingToDirection();
void leftHandFollow();

// ======================
// Function: measureDistance()
// Measures distance (in cm) using the given ultrasonic sensor pins.
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// ======================
// Motor Drive Functions
// ======================
void driveStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void driveTurnLeft(int speed) {
  // In-place left turn: left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void driveTurnRight(int speed) {
  // In-place right turn: left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// ======================
// Function: updatePosition()
// Updates the robot's grid position by the given offsets.
void updatePosition(int dx, int dy) {
  posX += dx;
  posY += dy;
  Serial.print("New Grid Position: (");
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(posY);
  Serial.println(")");
}

// ======================
// Helper Function: updateDirection()
// Updates currentDirection based on the turn degrees (assumes multiples of 90).
void updateDirection(int degrees) {
  int turns = (degrees / 90); // Positive for right turns, negative for left
  currentDirection = (currentDirection + turns + 4) % 4;
}

// ======================
// Function: turnByDegrees()
// Turns the robot by the specified degrees (must be a multiple of 90)
// and updates the currentDirection.
void turnByDegrees(int degrees, int speed) {
  if (degrees < 0) {
    driveTurnLeft(speed);
  } else {
    driveTurnRight(speed);
  }
  delay(500);  // Adjust delay to achieve approximately 90° per 500ms
  updateDirection(degrees);
}

// ======================
// Function: moveForwardOneCell()
// Moves the robot forward one cell and then stops.
void moveForwardOneCell(int speed) {
  driveForward(speed);
  delay(1000);  // Adjust delay to travel one cell (e.g., 25 cm)
  driveStop();
}

// ======================
// Helper Function: updatePositionAccordingToDirection()
// Updates grid position based on currentDirection.
void updatePositionAccordingToDirection() {
  // currentDirection: 0 = North, 1 = East, 2 = South, 3 = West
  if (currentDirection == 0) {
    updatePosition(0, 1);
  } else if (currentDirection == 1) {
    updatePosition(1, 0);
  } else if (currentDirection == 2) {
    updatePosition(0, -1);
  } else if (currentDirection == 3) {
    updatePosition(-1, 0);
  }
}

// ======================
// Left-Hand Following Algorithm
// ======================
// At each decision point, check sensors in the following order:
// 1. Left: if clear, turn left and move forward.
// 2. Front: if clear, go straight.
// 3. Right: if clear, turn right and move forward.
// 4. Dead-end: turn around.
void leftHandFollow() {
  long distLeft = measureDistance(TRIG_LEFT, ECHO_LEFT);
  long distFront = measureDistance(TRIG_FRONT, ECHO_FRONT);
  long distRight = measureDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  Serial.print("Left: "); Serial.print(distLeft);
  Serial.print(" cm, Front: "); Serial.print(distFront);
  Serial.print(" cm, Right: "); Serial.print(distRight);
  Serial.println(" cm");
  
  if (distLeft > WALL_DISTANCE_CM) {
    // Left is open: turn left and move one cell
    turnByDegrees(-90, baseSpeed);
    moveForwardOneCell(baseSpeed);
    updatePositionAccordingToDirection();
  } 
  else if (distFront > WALL_DISTANCE_CM) {
    // Front is open: go straight
    moveForwardOneCell(baseSpeed);
    updatePositionAccordingToDirection();
  } 
  else if (distRight > WALL_DISTANCE_CM) {
    // Right is open: turn right and move one cell
    turnByDegrees(90, baseSpeed);
    moveForwardOneCell(baseSpeed);
    updatePositionAccordingToDirection();
  } 
  else {
    // Dead end: turn around (180°) and move forward
    turnByDegrees(180, baseSpeed);
    moveForwardOneCell(baseSpeed);
    updatePositionAccordingToDirection();
  }
}

// ======================
// Setup Function
// ======================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  driveStop();
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // (Optional) Initialize maze mapping arrays if you wish to use them
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      wallsGrid[y][x] = 0;
      distanceGrid[y][x] = MAX_DISTANCE;
    }
  }
  
  Serial.println("Setup complete. Starting left-hand following mode.");
}

// ======================
// Main Loop: Use Left-Hand Following Algorithm
// ======================
void loop() {
  leftHandFollow();
  delay(100); // Small delay between moves
}
