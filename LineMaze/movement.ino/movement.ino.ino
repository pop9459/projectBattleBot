#include <Adafruit_NeoPixel.h>

#define TESTING_MODE false
//NeoPixel pin
#define PIN 5
//NeoPixel settings
#define NUMPIXELS 4
#define BRIGHTNES_LEVEL 20

//Motor pins
#define MOT_A1 10
#define MOT_A2 9
#define MOT_B1 6
#define MOT_B2 5
#define MOT_R1 2
#define MOT_R2 3

//Distance sersor pins
#define trigPin 12
#define echoPin 13

// Distance constants
#define CLOSE 20
#define NORMAL 30
#define FAR 100

//Gripper pin
#define GRIP 11

//Turns
//#define TURN_90_LEFT 34
//#define TURN_90_RIGHT 34
#define TURN_90_LEFT 40
#define TURN_90_RIGHT 40

//Movement
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
// Delay
//#define DELAYVAL 200
#define DELAYVAL 500

int BLACK_LIMIT = 775; // 500-550

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int sensorPins[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
int sensor_A0, sensor_A1, sensor_A2, sensor_A3, sensor_A4, sensor_A5, sensor_A6, sensor_A7;

// Define distance threshold (in cm)
const int _mazeDistanceThreshold = 10;

// Calibration values for the line sensors
int _minValues[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; // DARKEST VALUES
int _maxValues[] = {500, 500, 500, 500, 500, 500, 500, 500};         // LIGHTEST VALUES
int _lineTresholds[] = {800, 800, 800, 800, 800, 800, 800, 800};     // Threshold for the line sensors

//Bot state
bool started = false;
bool solved = false;
bool ended = false;

//Wheel rotation state
volatile int countL = 0;
volatile int countR = 0;

void ISR_L() {
  countL++;
}

void ISR_R() {
  countR++;
}

void setup() {
  Serial.begin(9600);
  pixels.begin();

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
  //  testSensors();
  //turnRight(TURN_90_RIGHT);
  //  turnLeft(TURN_90_LEFT);
  //  delay(5000);

  if (TESTING_MODE) {
    started = true;
  }



  //  //Game logic
  if (!started) {
    start();
  } else if (!solved) {
    maze();
  }
    else if (!ended) {
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

// //Running when obstacle apper and calibrate black limit
void start() {
  ungrab();
  //Check is obstacle appear
  int distance = culculateDistance();
  Serial.println(distance);
  // check if there is an object infront and not another batle bot
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

//   //Calibration for black limit
//   int blackLimit[3];
//   int currentIndex = 0;
//   int color;

//   for (int i = 0; i < 6; i++) {
//     stop();
//     delay(100);
//     int curentColor = getAverageLightLevel();
//     delay(100);
//     goStraightSlowStart();

//     color = curentColor;
//     while (color > curentColor - 300 && color < curentColor + 300) {
//       color = getAverageLightLevel();
//     }
//     if (i % 2 == 1) {
//       Serial.println(curentColor);
//       blackLimit[currentIndex] = curentColor;
//       currentIndex++;
//     }
//   }

  calibrateSensors();

  // BLACK_LIMIT = getAverageBlackLimit(blackLimit) - 100;

  // stop();
  // delay(1000);

  // startMovementAdjustment();

  grab();
  stop();
  delay(1000);

  goStraight(8);
  turnLeft(TURN_90_LEFT);
  delay(100);

  started = true;
}

// Function used for calculating the error of each of the line sensors
void calibrateSensors() {
  int startTime = millis(); // Mark the starting time
  int calibrationTime = 750; // Calibrate for 1 seconds
  goStraight(); // Start slowly creeping forward
  
  // Drive over a small distance and mark the brightest and dimmest values for each sensor
  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 8; i++) {
      int sensorValue = analogRead(sensorPins[i]);
      if(sensorValue > _maxValues[i]) _maxValues[i] = sensorValue;
      if(sensorValue < _minValues[i]) _minValues[i] = sensorValue;
    }
  }
  stop();

  // Calculate the thresholds (the average between the brightest and dimmest values)
  for (int i = 0; i < 8; i++) {
    _lineTresholds[i] = (_maxValues[i] + _minValues[i]) / 2;
  }

  int averageTreshold = 0;
  for (int i = 0; i < 8; i++) {
    averageTreshold += _lineTresholds[i];
  }
  averageTreshold /= 8;

  BLACK_LIMIT = averageTreshold;
}

//Solve the maze
void maze() {
  Serial.print("I am maze\n");
  read();

  // Read distance from ultrasonic sensor
  int mazeDistance = getMazeDistance();

  // Debugging: Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(mazeDistance);

  if (mazeDistance > _mazeDistanceThreshold || mazeDistance == 0) {
    // No obstacle detected: Continue
    if (isRightSensors()) {
      goStraight(CHECK_STRAIGT_LINE_MOVEMENT);
      read();

      if (isAllSensors()) {
        Serial.print("I am all sensors\n");
        solved = true;
      } else {
        Serial.print("I am right sensor\n");

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
      Serial.print("I am no sensors detected\n");
      //    goStraight(8);
      turnRightUltra();
      //    delay(DELAYVAL);
    } else if (sensor_A5 >= BLACK_LIMIT) {
      smallTurnRight();
    } else if (sensor_A2 >= BLACK_LIMIT) {
      smallTurnLeft();
    } else {
      Serial.print("going straight\n");
      goStraight();
    }
  } else {
    turnRight(TURN_90_RIGHT);
    turnRightUltra();
  }

  
}

//Finish maze solving and ungrab obstacle
void end() {
  delay(500);
  goBack(5);
  ungrab();
  goBack(30);
  while (true) {
    setPixlsRed();
    delay(200);
    setPixlsYellow();
    delay(200);
    setPixlsGreen();
    delay(200);
  }
  ended = true;
}

// Read values from sensor
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

// Get average value from all sensors
int getAverageLightLevel() {
  read();
  return (sensor_A0 + sensor_A1 + sensor_A2 + sensor_A3 + sensor_A4 + sensor_A5 + sensor_A6 + sensor_A7) / 8;
}

// Calculate black limit from array of light levels
int getAverageBlackLimit(int* array) {
  int res = 0;
  for (int i = 0; i < 3; i++) {
    res += array[i];
  }
  return res / 3;
}

// Check for sensors on line
bool isAllSensors() {
  return (isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1) && isOverBlackLimit(sensor_A2) && isOverBlackLimit(sensor_A3) && isOverBlackLimit(sensor_A4) && isOverBlackLimit(sensor_A5) && isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7));
}

bool isLeftSensors() {
  return (isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1) && isOverBlackLimit(sensor_A2)) || (isOverBlackLimit(sensor_A0) && isOverBlackLimit(sensor_A1)) || isOverBlackLimit(sensor_A0);
}

bool isRightSensors() {
  return (isOverBlackLimit(sensor_A5) && isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7)) || (isOverBlackLimit(sensor_A6) && isOverBlackLimit(sensor_A7));
}

bool isNoSensors() {
  return isBelowBlackLimit(sensor_A0) && isBelowBlackLimit(sensor_A1) && isBelowBlackLimit(sensor_A2) && isBelowBlackLimit(sensor_A3) && isBelowBlackLimit(sensor_A4) && isBelowBlackLimit(sensor_A5) && isBelowBlackLimit(sensor_A6) && isBelowBlackLimit(sensor_A7);
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
    if        (isOverBlackLimit(sensor_A1)) {
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


//Calculate distance from distance sensor
int culculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

//Calculate distance in the line maze
int getMazeDistance() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm
  int distance = duration * 0.034 / 2;  // Speed of sound: 0.034 cm/μs
  return distance;
}

// Grab with servo
void grab() {
  Serial.println("grab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(19000);
  }
}

// Ungrab with servo
void ungrab() {
  Serial.println("ungrab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1500);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(18500);
  }
}

//Movement
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

//Small Turning
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

void turnRightUltra() {
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

//Lights
void setPixlsRed() {
  pixels.setPixelColor(0, pixels.Color(0, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(1, pixels.Color(0, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(2, pixels.Color(0, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(3, pixels.Color(0, BRIGHTNES_LEVEL, 0));
  pixels.show();
}

void setPixlsGreen() {
  pixels.setPixelColor(0, pixels.Color(BRIGHTNES_LEVEL, 0, 0));
  pixels.setPixelColor(1, pixels.Color(BRIGHTNES_LEVEL, 0, 0));
  pixels.setPixelColor(2, pixels.Color(BRIGHTNES_LEVEL, 0, 0));
  pixels.setPixelColor(3, pixels.Color(BRIGHTNES_LEVEL, 0, 0));
  pixels.show();
}

void setPixlsYellow() {
  pixels.setPixelColor(0, pixels.Color(BRIGHTNES_LEVEL, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(1, pixels.Color(BRIGHTNES_LEVEL, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(2, pixels.Color(BRIGHTNES_LEVEL, BRIGHTNES_LEVEL, 0));
  pixels.setPixelColor(3, pixels.Color(BRIGHTNES_LEVEL, BRIGHTNES_LEVEL, 0));
  pixels.show();
}

void setPixlsBlue() {
  pixels.setPixelColor(0, pixels.Color(0, 0, BRIGHTNES_LEVEL));
  pixels.setPixelColor(1, pixels.Color(0, 0, BRIGHTNES_LEVEL));
  pixels.setPixelColor(2, pixels.Color(0, 0, BRIGHTNES_LEVEL));
  pixels.setPixelColor(3, pixels.Color(0, 0, BRIGHTNES_LEVEL));
  pixels.show();
}
