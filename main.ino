// ======================================================
// ADVANCED OBSTACLE AVOIDING ROVER
// Arduino Uno + 3 HC-SR04 + L293D
// Two DC motors + one caster wheel
// ======================================================

// ==================== NAVIGATION STATES ===============

const int PATH_CLEAR = 0;
const int OBSTACLE_FRONT = 1;
const int OBSTACLE_LEFT = 2;
const int OBSTACLE_RIGHT = 3;
const int OBSTACLE_BOTH_SIDES = 4;
const int OBSTACLE_ALL_SIDES = 5;
const int RECOVERING = 6;

int currentState = PATH_CLEAR;

// ==================== SENSOR PINS =====================

const int trigLeft = 2;
const int echoLeft = 3;

const int trigCenter = 4;
const int echoCenter = 5;

const int trigRight = 6;
const int echoRight = 7;

// ==================== MOTOR PINS ======================

const int leftEnable = 9;
const int leftIn1 = 8;
const int leftIn2 = 10;

const int rightEnable = 11;
const int rightIn1 = 12;
const int rightIn2 = 13;

// ==================== MOTOR SETTINGS ==================

int leftSpeed = 160;
int rightSpeed = 160;

int slowSpeed = 110;
int turnSpeed = 170;

// Motor calibration: adjust if the rover drifts.
int leftCalibration = 0;
int rightCalibration = 0;

// ==================== DISTANCE SETTINGS ===============

const float stopDistance = 25.0;
const float sideWarningDistance = 22.0;
const float emergencyDistance = 12.0;

// ==================== TIMING SETTINGS ==================

const unsigned long reverseTime = 250;
const unsigned long turnTime = 350;
const unsigned long sharpTurnTime = 550;
const unsigned long recoveryReverseTime = 600;

const unsigned long sensorTimeout = 18000UL;

// ==================== RECOVERY SETTINGS ===============

const int maxRepeatedBlocks = 3;

int repeatedBlocks = 0;
int repeatedTurns = 0;

// ==================== SENSOR VARIABLES ================

float leftDistance = 400.0;
float centerDistance = 400.0;
float rightDistance = 400.0;

float previousLeft = 400.0;
float previousCenter = 400.0;
float previousRight = 400.0;

bool leftSensorValid = false;
bool centerSensorValid = false;
bool rightSensorValid = false;

// ======================================================
// READ ULTRASONIC DISTANCE
// ======================================================

float readDistance(int trigPin, int echoPin, bool &valid) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH, sensorTimeout);

  if (duration == 0) {
    valid = false;
    return 400.0;
  }

  float distance = duration * 0.0343 / 2.0;

  if (distance < 2.0 || distance > 400.0) {
    valid = false;
    return 400.0;
  }

  valid = true;
  return distance;
}

// ======================================================
// FILTER SENSOR READINGS
// ======================================================

float filterDistance(float current, float previous,
                     bool valid) {
  if (!valid) {
    return 400.0;
  }

  if (previous >= 400.0) {
    return current;
  }

  return 0.65 * current + 0.35 * previous;
}

// ======================================================
// READ ALL THREE SENSORS
// ======================================================

void readAllSensors() {
  float rawLeft =
      readDistance(trigLeft, echoLeft, leftSensorValid);

  delay(35);

  float rawCenter =
      readDistance(trigCenter, echoCenter, centerSensorValid);

  delay(35);

  float rawRight =
      readDistance(trigRight, echoRight, rightSensorValid);

  delay(35);

  leftDistance =
      filterDistance(rawLeft, previousLeft, leftSensorValid);

  centerDistance =
      filterDistance(rawCenter, previousCenter, centerSensorValid);

  rightDistance =
      filterDistance(rawRight, previousRight, rightSensorValid);

  previousLeft = leftDistance;
  previousCenter = centerDistance;
  previousRight = rightDistance;
}

// ======================================================
// DISPLAY SENSOR READINGS
// ======================================================

void printSensorData() {
  Serial.print("LEFT: ");

  if (leftSensorValid) {
    Serial.print(leftDistance);
    Serial.print(" cm");
  } else {
    Serial.print("NO ECHO");
  }

  Serial.print(" | CENTER: ");

  if (centerSensorValid) {
    Serial.print(centerDistance);
    Serial.print(" cm");
  } else {
    Serial.print("NO ECHO");
  }

  Serial.print(" | RIGHT: ");

  if (rightSensorValid) {
    Serial.print(rightDistance);
    Serial.print(" cm");
  } else {
    Serial.print("NO ECHO");
  }

  Serial.println();
}

// ======================================================
// STOP MOTORS
// ======================================================

void stopMotors() {
  analogWrite(leftEnable, 0);
  analogWrite(rightEnable, 0);

  digitalWrite(leftIn1, LOW);
  digitalWrite(leftIn2, LOW);

  digitalWrite(rightIn1, LOW);
  digitalWrite(rightIn2, LOW);
}

// ======================================================
// CONTROL LEFT MOTOR
// Positive = forward, negative = reverse
// ======================================================

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(leftIn1, HIGH);
    digitalWrite(leftIn2, LOW);
    analogWrite(leftEnable, speed);
  }
  else if (speed < 0) {
    digitalWrite(leftIn1, LOW);
    digitalWrite(leftIn2, HIGH);
    analogWrite(leftEnable, -speed);
  }
  else {
    digitalWrite(leftIn1, LOW);
    digitalWrite(leftIn2, LOW);
    analogWrite(leftEnable, 0);
  }
}

// ======================================================
// CONTROL RIGHT MOTOR
// Positive = forward, negative = reverse
// ======================================================

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(rightIn1, HIGH);
    digitalWrite(rightIn2, LOW);
    analogWrite(rightEnable, speed);
  }
  else if (speed < 0) {
    digitalWrite(rightIn1, LOW);
    digitalWrite(rightIn2, HIGH);
    analogWrite(rightEnable, -speed);
  }
  else {
    digitalWrite(rightIn1, LOW);
    digitalWrite(rightIn2, LOW);
    analogWrite(rightEnable, 0);
  }
}

// ======================================================
// MOVEMENT FUNCTIONS
// ======================================================

void moveForward() {
  setLeftMotor(constrain(
      leftSpeed + leftCalibration, 0, 255));

  setRightMotor(constrain(
      rightSpeed + rightCalibration, 0, 255));
}

void moveSlow() {
  setLeftMotor(constrain(
      slowSpeed + leftCalibration, 0, 255));

  setRightMotor(constrain(
      slowSpeed + rightCalibration, 0, 255));
}

void moveBackward() {
  setLeftMotor(-leftSpeed);
  setRightMotor(-rightSpeed);
}

void turnLeft() {
  setLeftMotor(-turnSpeed);
  setRightMotor(turnSpeed);
}

void turnRight() {
  setLeftMotor(turnSpeed);
  setRightMotor(-turnSpeed);
}

// ======================================================
// STEERING FUNCTIONS
// ======================================================

void steerLeft() {
  setLeftMotor(slowSpeed);
  setRightMotor(rightSpeed);
}

void steerRight() {
  setLeftMotor(leftSpeed);
  setRightMotor(slowSpeed);
}

// ======================================================
// CHOOSE THE CLEAREST SIDE
// true = left, false = right
// ======================================================

bool chooseLeft() {
  Serial.println("Comparing available space...");

  if (!leftSensorValid && rightSensorValid) {
    return false;
  }

  if (!rightSensorValid && leftSensorValid) {
    return true;
  }

  if (!leftSensorValid && !rightSensorValid) {
    Serial.println("SIDE READINGS UNAVAILABLE");
    return true;
  }

  if (leftDistance >= rightDistance) {
    Serial.println("LEFT SIDE HAS MORE SPACE");
    return true;
  }

  Serial.println("RIGHT SIDE HAS MORE SPACE");
  return false;
}

// ======================================================
// TURN TOWARD THE CLEAREST SIDE
// ======================================================

void turnTowardClearSide(unsigned long duration) {
  if (chooseLeft()) {
    Serial.println("TURNING LEFT");
    turnLeft();
  }
  else {
    Serial.println("TURNING RIGHT");
    turnRight();
  }

  delay(duration);

  stopMotors();
  delay(100);
}

// ======================================================
// AVOID FRONT OBSTACLE
// ======================================================

void avoidFrontObstacle() {
  Serial.println("FRONT OBSTACLE DETECTED");

  stopMotors();
  delay(100);

  if (leftSensorValid &&
      rightSensorValid &&
      leftDistance < sideWarningDistance &&
      rightDistance < sideWarningDistance) {

    Serial.println("BOTH SIDES RESTRICTED");

    moveBackward();
    delay(recoveryReverseTime);
  }
  else {
    moveBackward();
    delay(reverseTime);
  }

  stopMotors();
  delay(100);

  if (repeatedBlocks >= maxRepeatedBlocks) {
    Serial.println("REPEATED BLOCKAGE - SHARP TURN");
    turnTowardClearSide(sharpTurnTime);
  }
  else {
    turnTowardClearSide(turnTime);
  }

  repeatedBlocks++;
  repeatedTurns++;
}

// ======================================================
// RECOVERY MODE
// ======================================================

void recoveryMode() {
  currentState = RECOVERING;

  Serial.println("RECOVERY MODE ACTIVATED");

  stopMotors();
  delay(200);

  moveBackward();
  delay(recoveryReverseTime);

  stopMotors();
  delay(150);

  turnTowardClearSide(sharpTurnTime);

  repeatedBlocks = 0;
  repeatedTurns = 0;

  currentState = PATH_CLEAR;
}

// ======================================================
// CLASSIFY SURROUNDINGS
// ======================================================

int classifyEnvironment() {
  // Don't assume the front is clear if its sensor fails.
  if (!centerSensorValid) {
    Serial.println("WARNING: CENTRE SENSOR INVALID");
    return OBSTACLE_FRONT;
  }

  bool leftBlocked =
      leftSensorValid &&
      leftDistance < sideWarningDistance;

  bool centerBlocked =
      centerDistance < stopDistance;

  bool rightBlocked =
      rightSensorValid &&
      rightDistance < sideWarningDistance;

  if (leftBlocked &&
      centerBlocked &&
      rightBlocked) {
    return OBSTACLE_ALL_SIDES;
  }

  if (centerBlocked) {
    return OBSTACLE_FRONT;
  }

  if (leftBlocked && rightBlocked) {
    return OBSTACLE_BOTH_SIDES;
  }

  if (leftBlocked) {
    return OBSTACLE_LEFT;
  }

  if (rightBlocked) {
    return OBSTACLE_RIGHT;
  }

  return PATH_CLEAR;
}

// ======================================================
// NAVIGATION CONTROLLER
// ======================================================

void navigate() {
  currentState = classifyEnvironment();

  switch (currentState) {

    case PATH_CLEAR:
      Serial.println("PATH CLEAR - MOVING FORWARD");

      repeatedBlocks = 0;
      repeatedTurns = 0;

      moveForward();
      break;

    case OBSTACLE_FRONT:
      avoidFrontObstacle();
      break;

    case OBSTACLE_LEFT:
      Serial.println("LEFT BLOCKED - STEERING RIGHT");

      steerRight();
      delay(100);
      break;

    case OBSTACLE_RIGHT:
      Serial.println("RIGHT BLOCKED - STEERING LEFT");

      steerLeft();
      delay(100);
      break;

    case OBSTACLE_BOTH_SIDES:
      Serial.println("BOTH SIDES CLOSE");

      if (centerSensorValid &&
          centerDistance > stopDistance + 10.0) {

        moveSlow();
        delay(100);
      }
      else {
        avoidFrontObstacle();
      }

      break;

    case OBSTACLE_ALL_SIDES:
      Serial.println("BOXED IN");
      recoveryMode();
      break;

    case RECOVERING:
      stopMotors();
      break;
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigCenter, OUTPUT);
  pinMode(echoCenter, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  pinMode(leftEnable, OUTPUT);
  pinMode(leftIn1, OUTPUT);
  pinMode(leftIn2, OUTPUT);

  pinMode(rightEnable, OUTPUT);
  pinMode(rightIn1, OUTPUT);
  pinMode(rightIn2, OUTPUT);

  digitalWrite(trigLeft, LOW);
  digitalWrite(trigCenter, LOW);
  digitalWrite(trigRight, LOW);

  Serial.begin(9600);

  stopMotors();

  Serial.println();
  Serial.println("==============================");
  Serial.println(" OBSTACLE AVOIDING ROVER");
  Serial.println("==============================");

  Serial.println("Initializing sensors...");

  delay(500);

  readAllSensors();
  printSensorData();

  Serial.println("Initialization complete");

  delay(1000);
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
  readAllSensors();
  printSensorData();

  // Emergency response to a very close front obstacle.
  if (centerSensorValid &&
      centerDistance < emergencyDistance) {

    Serial.println("!!! EMERGENCY STOP !!!");

    stopMotors();
    delay(100);

    moveBackward();
    delay(reverseTime);

    stopMotors();
    delay(100);

    turnTowardClearSide(sharpTurnTime);

    repeatedBlocks++;

    if (repeatedBlocks >= maxRepeatedBlocks) {
      recoveryMode();
    }
  }
  else {
    navigate();
  }

  delay(30);
}
