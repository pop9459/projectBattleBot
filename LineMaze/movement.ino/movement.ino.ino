#define DEBUG false

// Motor pins
#define MOT_A1 10
#define MOT_A2 9
#define MOT_B1 6
#define MOT_B2 5
#define MOT_R1 2
#define MOT_R2 3

// Distance sensor pins
#define trigPin 12
#define echoPin 13

// Distance constants
#define CLOSE 20
#define NORMAL 30
#define FAR 100

// Gripper pin
#define GRIP 11

// Turns
#define TURN_90_LEFT 40
#define TURN_90_RIGHT 40

// Movement
#define MOTOR_TURN_SPEED 170
#define CHECK_STRAIGT_LINE_MOVEMENT 5

// A - right wheel 
// B -left wheel 
#define MOTOR_A_SPEED 245
#define MOTOR_B_SPEED 245

#define MOTOR_A_SLOW_SPEED 215
#define MOTOR_B_SLOW_SPEED 215

#define MOTOR_A_SLOW_TURN_SPEED 223
#define MOTOR_B_SLOW_TURN_SPEED 223

#define DELAYVAL 500

int BLACK_LIMIT = 775;

const int sensorPins[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int sensor_A0, sensor_A1, sensor_A2, sensor_A3, sensor_A4, sensor_A5, sensor_A6, sensor_A7;

// Define distance threshold (in cm)
const int _mazeDistanceThreshold = 10;

// Calibration values for the line sensors
int _minValues[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int _maxValues[] = {500, 500, 500, 500, 500, 500, 500, 500};
int _lineTresholds[] = {800, 800, 800, 800, 800, 800, 800, 800};

// Bot state
bool started = false;
bool solved = false;
bool ended = false;

// Wheel rotation state
volatile int countL = 0;
volatile int countR = 0;

void ISR_L() { countL++; }
void ISR_R() { countR++; }

void setup() {
  Serial.begin(9600);

  pinMode(MOT_A1, OUTPUT);
  pinMode(MOT_A2, OUTPUT);
  pinMode(MOT_B1, OUTPUT);
  pinMode(MOT_B2, OUTPUT);
  pinMode(MOT_R1, INPUT_PULLUP);
  pinMode(MOT_R2, INPUT_PULLUP);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(GRIP, OUTPUT);
  digitalWrite(GRIP, LOW);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(MOT_R1), ISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT_R2), ISR_L, CHANGE);
}

void loop() {
  if (DEBUG) {
    started = true;
  }

  if (!started) {
    start();
  } else if (!solved) {
    maze();
  } else if (!ended) {
    end();
  }
}

void testSensors() {
  delay(1000);
  read();

  if (isRightSensors()) {
    Serial.print("I right all sensors\n");
  } else if (isLeftSensors()) {
    Serial.print("I left sensors\n");
  } else if (isNoSensors()) {
    Serial.print("I am no sensors detected\n");
  } else if (sensor_A5 >= BLACK_LIMIT) {
    Serial.print("I am A2 sensor, adjusting right\n");
  } else if (sensor_A2 >= BLACK_LIMIT) {
    smallTurnLeft();
    Serial.print("I am A5 sensor, adjusting left \n");
  } else {
    Serial.print("going straight\n");
  }
}

void start() {
  ungrab();
  int distance = culculateDistance();
  Serial.println(distance);

  int countToStart = 0;
  while (distance >= 30 || countToStart < 5) {
    distance = culculateDistance();
    Serial.println(distance);
    if (distance > 30) {
      countToStart = 0;
    } else {
      countToStart++;
    }
  }

  activationWait();
  calibrateSensors();
  grab();
  stop();
  delay(1000);
  goStraight(8);
  turnLeft(TURN_90_LEFT);
  delay(100);
  started = true;
}

void calibrateSensors() {
  int startTime = millis();
  int calibrationTime = 750;
  goStraight();

  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 8; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      if(sensorValue > _maxValues[i]) _maxValues[i] = sensorValue;
      if(sensorValue < _minValues[i]) _minValues[i] = sensorValue;
    }
  }
  stop();

  for (int i = 0; i < 8; i++) {
    _lineTresholds[i] = (_maxValues[i] + _minValues[i]) / 2;
  }

  int averageTreshold = 0;
  for (int i = 0; i < 8; i++) {
    averageTreshold += _lineTresholds[i];
  }
  BLACK_LIMIT = averageTreshold / 8;
}

void maze() {
  Serial.print("I am maze\n");
  read();
  int mazeDistance = getMazeDistance();
  Serial.print("Distance: ");
  Serial.println(mazeDistance);

  if (mazeDistance > _mazeDistanceThreshold || mazeDistance == 0) {
    if (isRightSensors()) {
      goStraight(CHECK_STRAIGT_LINE_MOVEMENT);
      read();
      if (isAllSensors()) {
        Serial.print("I am all sensors\n");
        solved = true;
      } else {
        turnRight(TURN_90_RIGHT);
        delay(DELAYVAL);
      }
    } else if (isLeftSensors()) {
      goStraight(CHECK_STRAIGT_LINE_MOVEMENT);
      stop();
      delay(10);
      read();
      if (!isCenterandOrSmallRightSensors()) {
        turnLeft(TURN_90_LEFT);
        delay(DELAYVAL);
      }
    } else if (isNoSensors()) {
      turnRightUntilClear();
    } else if (sensor_A5 >= BLACK_LIMIT) {
      smallTurnRight();
    } else if (sensor_A2 >= BLACK_LIMIT) {
      smallTurnLeft();
    } else {
      goStraight();
    }
  } else {
    turnRight(TURN_90_RIGHT);
    turnRightUntilClear();
  }
}

void end() {
  delay(500);
  goBack(5);
  ungrab();
  goBack(30);
  while (true) {
    delay(200);
    delay(200);
    delay(200);
  }
  ended = true;
}

void read() {
  sensor_A0 = analogRead(A0);
  sensor_A1 = analogRead(A1);
  sensor_A2 = analogRead(A2);
  sensor_A3 = analogRead(A3);
  sensor_A4 = analogRead(A4);
  sensor_A5 = analogRead(A5);
  sensor_A6 = analogRead(A6);
  sensor_A7 = analogRead(A7);
}

int getAverageLightLevel() {
  read();
  return (sensor_A0 + sensor_A1 + sensor_A2 + sensor_A3 + sensor_A4 + sensor_A5 + sensor_A6 + sensor_A7) / 8;
}

int getAverageBlackLimit(int* array) {
  int res = 0;
  for (int i = 0; i < 3; i++) {
    res += array[i];
  }
  return res / 3;
}

bool isAllSensors() {
  return isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1) && isOverBlackLimit(sensor_A2) &&
         isOverBlackLimit(sensor_A3) && isOverBlackLimit(sensor_A4) && isOverBlackLimit(sensor_A5) &&
         isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7);
}

bool isLeftSensors() {
  return (isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1) && isOverBlackLimit(sensor_A2)) ||
         (isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1)) || isOverBlackLimit(sensor_A0);
}

bool isRightSensors() {
  return (isOverBlackLimit(sensor_A5) && isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7)) ||
         (isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7));
}

bool isNoSensors() {
  return isBelowBlackLimit(sensor_A0) && isBelowBlackLimit(sensor_A1) && isBelowBlackLimit(sensor_A2) &&
         isBelowBlackLimit(sensor_A3) && isBelowBlackLimit(sensor_A4) && isBelowBlackLimit(sensor_A5) &&
         isBelowBlackLimit(sensor_A6) && isBelowBlackLimit(sensor_A7);
}

bool isCenterSensors() {
  return isOverBlackLimit(sensor_A3) || isOverBlackLimit(sensor_A4);
}

bool isCenterandOrSmallRightSensors() {
  return isOverBlackLimit(sensor_A3) || isOverBlackLimit(sensor_A4) || isOverBlackLimit(sensor_A5);
}

bool isOverBlackLimit(int sensor) {
  return sensor >= BLACK_LIMIT;
}

bool isBelowBlackLimit(int sensor) {
  return sensor <= BLACK_LIMIT;
}

void startMovementAdjustment() {
  while (!isAllSensors()) {
    read();
    if (isOverBlackLimit(sensor_A1)) {
      smallTurnRight(210);
    } else if (isOverBlackLimit(sensor_A6)) {
      smallTurnLeft(210);
    } else if (isOverBlackLimit(sensor_A0)) {
      smallTurnRight(160);
    } else if (isOverBlackLimit(sensor_A7)) {
      smallTurnLeft(160);
    } else if (isOverBlackLimit(sensor_A2)) {
      smallTurnRight();
    } else if (isOverBlackLimit(sensor_A5)) {
      smallTurnLeft();
    } else {
      goStraightSlow();
    }
  }
}

int culculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

int getMazeDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void grab() {
  Serial.println("grab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(19000);
  }
}

void ungrab() {
  Serial.println("ungrab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1500);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(18500);
  }
}

void goStraight() {
  analogWrite(MOT_A2, MOTOR_A_SPEED);
  analogWrite(MOT_B2, MOTOR_B_SPEED);
  analogWrite(MOT_A1, LOW);
  analogWrite(MOT_B1, LOW);
}

void goStraightSlow() {
  analogWrite(MOT_A2, MOTOR_A_SLOW_SPEED);
  analogWrite(MOT_B2, MOTOR_B_SLOW_SPEED);
  analogWrite(MOT_A1, LOW);
  analogWrite(MOT_B1, LOW);
}

void goStraightSlowStart() {
  analogWrite(MOT_A2, MOTOR_A_SLOW_SPEED );
  analogWrite(MOT_B2, MOTOR_B_SLOW_SPEED + 24);
  analogWrite(MOT_A1, LOW);
  analogWrite(MOT_B1, LOW);
}

void goStraight(int d) {
  countL = 0;
  countR = 0;

  while (countR < d) {
    analogWrite(MOT_A2, MOTOR_A_SPEED);
    analogWrite(MOT_B2, MOTOR_B_SPEED);
    analogWrite(MOT_A1, LOW);
    analogWrite(MOT_B1, LOW);
  }
  stop();
}

void goBack(int d) {
  countL = 0;
  countR = 0;

  while (countR < d) {
    analogWrite(MOT_A1, MOTOR_A_SPEED);
    analogWrite(MOT_B1, MOTOR_B_SPEED);
    analogWrite(MOT_A2, LOW);
    analogWrite(MOT_B2, LOW);
  }
  stop();
}

void stop() {
  analogWrite(MOT_A1, LOW);
  analogWrite(MOT_B1, LOW);
  analogWrite(MOT_A2, LOW);
  analogWrite(MOT_B2, LOW);
}

void smallTurnRight() {
  analogWrite(MOT_A2, MOTOR_TURN_SPEED);
  analogWrite(MOT_B2, MOTOR_B_SPEED);
}

void smallTurnLeft() {
  analogWrite(MOT_B2, MOTOR_TURN_SPEED);
  analogWrite(MOT_A2, MOTOR_A_SPEED);
}

void smallTurnRight(int speed) {
  analogWrite(MOT_A2, speed);
  analogWrite(MOT_B2, MOTOR_B_SPEED);
}

void smallTurnLeft(int speed) {
  analogWrite(MOT_B2, speed);
  analogWrite(MOT_A2, MOTOR_A_SPEED);
}

void turnRight(int d) {
  countL = 0;
  countR = 0;
  while (countL < d) {
    analogWrite(MOT_B2, MOTOR_B_SPEED);
    analogWrite(MOT_A1, LOW);
    analogWrite(MOT_A2, LOW);
    analogWrite(MOT_B1, LOW);
  }
  stop();
}

void turnLeft(int d) {
  countL = 0;
  countR = 0;
  while (countR < d) {
    analogWrite(MOT_A2, MOTOR_A_SPEED);
    analogWrite(MOT_A1, LOW);
    analogWrite(MOT_B1, LOW);
    analogWrite(MOT_B2, LOW);
  }
  stop();
}

void turnRightUntilClear() {
  fullTurnRightSlow();
  while (true) {
    read();
    if (isCenterSensors()) {
      stop();
      break;
    }
  }
  stop();
  fullTurnLeft();
  delay(100);
  stop();
}

void fullTurnLeft() {
  analogWrite(MOT_B1, MOTOR_B_SPEED);
  analogWrite(MOT_A2, MOTOR_A_SPEED);
  analogWrite(MOT_A1, LOW);
  analogWrite(MOT_B2, LOW);
}

void fullTurnRightSlow() {
  analogWrite(MOT_B2, MOTOR_B_SLOW_TURN_SPEED);
  analogWrite(MOT_A1, MOTOR_A_SLOW_TURN_SPEED);
  analogWrite(MOT_A2, LOW);
  analogWrite(MOT_B1, LOW);
}

void fullTurnRight() {
  analogWrite(MOT_B2, MOTOR_B_SPEED);
  analogWrite(MOT_A1, MOTOR_A_SPEED);
  analogWrite(MOT_B1, LOW);
  analogWrite(MOT_A2, LOW);
}

void activationWait() {
  delay(2000);
}
