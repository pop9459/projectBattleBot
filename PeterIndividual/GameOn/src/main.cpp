#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

//function definitions

void leftSensorPulse();
void rightSensorPulse();

float getLeftDistance();
float getRightDistance();
float getFrontDistance();

void calibrateSensors();
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void followRightWall(int speed);
void followLeftWall(int speed);
void followLine(int slowSpeed, int fastSpeed);
void setGripper(int position);
void setLights(int startIndex, int endIndex, int r, int g, int b);

void updateMazeWithSensors();
void moveToNextCell();
void updateFloodFill();
void initializeMazeArray();

// Motor pins
#define L_FWD 5 // Left motor forward pin - A1
#define R_FWD 9 // Right motor forward pin - B1
#define L_BWD 6 // Left motor backward pin - A2
#define R_BWD 10 // Right motor backward pin - B2
#define L_ROT 2 // Left rotation sensor pin - R1
#define R_ROT 3 // Right rotation sensor pin -R2

// LINE SENSOR PINS
int _lineSensorPins[] = {A6, A7, A2, A3, A4, A5};

// Neopixel constants and definitions
#define NEOPIXEL_PIN 4 // Neopixel pin
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(4, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

// Ultrasonic sensor pins
#define FRONT_US_TRIG_PIN 12 // Ultrasonic sensor trigger pin
#define FRONT_US_ECHO_PIN A0 // Front ultrasonic sensor echo pin
#define LEFT_US_TRIG_PIN 13 // Ultrasonic sensor trigger pin
#define LEFT_US_ECHO_PIN 7 // Front ultrasonic sensor echo pin
#define RIGHT_US_TRIG_PIN 8 // Ultrasonic sensor trigger pin
#define RIGHT_US_ECHO_PIN A1 // Front ultrasonic sensor echo pin

// Gripper constants
#define GRIPPER_PIN 11 // Gripper servo pin
#define GRIPPER_OPEN 100 // Gripper open position
#define GRIPPER_CLOSE 40 // Gripper close position

// Driving constants
#define PULSES_PER_REVOLUTION 20 // Number of pulses the rotation sensor outputs per revolution 
#define STEERING_ADJUSTMENT 2 // Used to balance the speed

volatile unsigned int _leftPulses; //number of pulses counted by the left rotation sensor
volatile unsigned int _rightPulses; //number of pulses counted by the right rotation sensor

//calibration values for the line sensors
int _minValues[] = {1023, 1023, 1023, 1023, 1023, 1023}; //DARKEST VALUES
int _maxValues[] = {500, 500, 500, 500, 500, 500}; //LIGHTEST VALUES
int _lineTresholds[] = {800, 800, 800, 800, 800, 800}; //treshold for the line sensors
float _line_sensor_modifiers[] = {6, 3.5, 1.75, -1.75, -3.5, -6}; // weights for adjusting the intensity of steering based on the sensor position

int _currentLeftSpeed = 0; //current speed of the left motor
int _currentRightSpeed = 0; //current speed of the right motor

int _leftDistance; // Lats distance from the left ultrasonic sensor
int _frontDistance; // Lats distance from the front ultrasonic sensor
int _rightDistance; // Lats distance from the right ultrasonic sensor

// Maze dimensions
const int MAZE_WIDTH = 7;
const int MAZE_HEIGHT = 3;
int _mazeGoalX = 5; 
int _mazeGoalY = 0;

// Maze grid to store walls and distances
struct Cell {
  int posX;
  int posY;
  uint8_t floodFillDistance;
  bool northWall;
  bool eastWall;
  bool southWall;
  bool westWall;
};

Cell _mazeGrid[MAZE_HEIGHT][MAZE_WIDTH];

// Robot's current position and orientation
int _robotX = 1; // Starting X position
int _robotY = 0; // Starting Y position
int _robotDir = 0; // 0 = Up, 1 = Right, 2 = Down, 3 = Left

bool _printDebug = false;

void setup() {
  // Initialize the neopixels  
  pixels.begin();
  setLights(0, 3, 255, 0, 0); // Set the lights to blue

  // Initialize serial communication for debuging
  Serial.begin(9600); 

  // setGripper(GRIPPER_OPEN); // open the gripper

  //begin the interrupt functions to count the rotations
  attachInterrupt(digitalPinToInterrupt(L_ROT), leftSensorPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ROT), rightSensorPulse, RISING);

  //set the motor pins
  pinMode(L_FWD, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(L_BWD, OUTPUT);
  pinMode(R_BWD, OUTPUT);
  pinMode(L_ROT, INPUT);
  pinMode(R_ROT, INPUT);

  // Set the line sensor pins using a for loop
  for (int i = 0; i < 6; i++) {
    pinMode(_lineSensorPins[i], INPUT);
  }

  //reset pins
  digitalWrite(L_FWD, LOW);
  digitalWrite(R_FWD, LOW);
  digitalWrite(L_BWD, LOW);
  digitalWrite(R_BWD, LOW);

  //set the ultrasonic sensor pins
  pinMode(FRONT_US_TRIG_PIN, OUTPUT);
  pinMode(LEFT_US_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_US_TRIG_PIN, OUTPUT);
  pinMode(FRONT_US_ECHO_PIN, INPUT);
  pinMode(LEFT_US_ECHO_PIN, INPUT);
  pinMode(RIGHT_US_ECHO_PIN, INPUT);

  // // Indicate "override movement"
  // setLights(0, 3, 255, 165, 0); // Set the lights to orange

  // // Calibrate the line sensors
  // // This will allow the robot to get close egougth to the cone to pick it up
  // calibrateSensors();
  
  // setGripper(GRIPPER_CLOSE); // Open the gripper

  // delay(500); // Small delay to prevent cone bumping 

  // // manually steer into the maze
  // drive(60, 0, 0.25); // forward a bit
  // drive(60, -35, 1); // steer left

  // // follow the short line untill in a set possition
  // int startTime = millis();
  // while(millis() < startTime + 600)
  // {
  //   followLine(85, 100);
  // }

  initializeMazeArray();

  setLights(0, 3, 0, 255, 0); // Set the lights to green
}

void loop() {
  // Update the maze with sensor data
  updateMazeWithSensors();

  // Debug prints
  if(_printDebug) {
    // Print robot's current position and direction
    Serial.print("Robot Position: (");
    Serial.print(_robotX);
    Serial.print(", ");
    Serial.print(_robotY);
    Serial.print("), Direction: ");
    switch (_robotDir) {
        case 0: Serial.println("Up"); break;
        case 1: Serial.println("Right"); break;
        case 2: Serial.println("Down"); break;
        case 3: Serial.println("Left"); break;
    }

    // Print sensor readings
    Serial.print("Front Distance: ");
    Serial.print(getFrontDistance());
    Serial.print(" cm, Left Distance: ");
    Serial.print(getLeftDistance());
    Serial.print(" cm, Right Distance: ");
    Serial.print(getRightDistance());
    Serial.println(" cm");

    // Print the entire maze grid status
    Serial.println("Maze Grid Status:");
    for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        Serial.print("(");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print("): ");
        Serial.print(_mazeGrid[y][x].northWall ? "N" : "-");
        Serial.print(_mazeGrid[y][x].eastWall ? "E" : "-");
        Serial.print(_mazeGrid[y][x].southWall ? "S" : "-");
        Serial.print(_mazeGrid[y][x].westWall ? "W" : "-");
        Serial.print("  ");
      }
      Serial.println();
    }

    // Print the entire flood fill distances
    Serial.println("Flood Fill Distances:");
    for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
      for (int x = 0; x < MAZE_WIDTH; x++) {
        Serial.print("(");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print("): ");
        Serial.print(_mazeGrid[y][x].floodFillDistance);
        Serial.print("  ");
      }
      Serial.println();
    }

    // Wait for the user to press a key in the serial monitor
    Serial.println("Press a key to continue...");
    while (Serial.available() == 0) {
      // Do nothing, just wait for input
    }
    while (Serial.available() > 0) {
      Serial.read(); // Clear the input buffer
    }
  }
  
  // Move to the next cell
  moveToNextCell();

  // Check if the robot has reached the goal
  if (_robotX == _mazeGoalX && _robotY == _mazeGoalY) {
    Serial.println("Goal reached! Stopping robot.");
    drive(0); // Stop the robot
    setLights(0, 3, 255, 0, 0); // Set the lights to green
    while (true); // Stop execution
  }
}

void updateFloodFill() {
  // Initialize all cells with a high distance value
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      _mazeGrid[y][x].floodFillDistance = UINT8_MAX; // Max value (255) for uint8_t
    }
  }

  // BFS queue setup
  const int MAX_QUEUE_SIZE = MAZE_WIDTH * MAZE_HEIGHT;
  int queueX[MAX_QUEUE_SIZE];
  int queueY[MAX_QUEUE_SIZE];
  int front = 0, rear = 0;

  // Add the goal cell to the queue
  queueX[rear] = _mazeGoalX;
  queueY[rear] = _mazeGoalY;
  rear++;
  _mazeGrid[_mazeGoalY][_mazeGoalX].floodFillDistance = 0;

  // BFS loop
  while (front < rear) {
    int x = queueX[front];
    int y = queueY[front];
    int dist = _mazeGrid[y][x].floodFillDistance;
    front++;

    // Check all four directions
    if (!_mazeGrid[y][x].northWall && _mazeGrid[y + 1][x].floodFillDistance == UINT8_MAX) {
      _mazeGrid[y + 1][x].floodFillDistance = dist + 1;
      queueX[rear] = x;
      queueY[rear] = y + 1;
      rear++;
    }
    if (!_mazeGrid[y][x].eastWall && _mazeGrid[y][x + 1].floodFillDistance == UINT8_MAX) {
      _mazeGrid[y][x + 1].floodFillDistance = dist + 1;
      queueX[rear] = x + 1;
      queueY[rear] = y;
      rear++;
    }
    if (!_mazeGrid[y][x].southWall && _mazeGrid[y - 1][x].floodFillDistance == UINT8_MAX) {
      _mazeGrid[y - 1][x].floodFillDistance = dist + 1;
      queueX[rear] = x;
      queueY[rear] = y - 1;
      rear++;
    }
    if (!_mazeGrid[y][x].westWall && _mazeGrid[y][x - 1].floodFillDistance == UINT8_MAX) {
      _mazeGrid[y][x - 1].floodFillDistance = dist + 1;
      queueX[rear] = x - 1;
      queueY[rear] = y;
      rear++;
    }
  }
}


void updateMazeWithSensors() {
  // Check front wall
  if (getFrontDistance() < 12) {
    if (_robotDir == 0)
    {
      _mazeGrid[_robotY][_robotX].northWall = true; // Facing North
      _mazeGrid[_robotY+1][_robotX].southWall = true; 
    } 
    else if (_robotDir == 1)
    {
      _mazeGrid[_robotY][_robotX].eastWall = true; // Facing East
      _mazeGrid[_robotY][_robotX+1].westWall = true;
    } 
    else if (_robotDir == 2)
    {
      _mazeGrid[_robotY][_robotX].southWall = true; // Facing South
      _mazeGrid[_robotY-1][_robotX].northWall = true;
    } 
    else if (_robotDir == 3)
    {
      _mazeGrid[_robotY][_robotX].westWall = true; // Facing West
      _mazeGrid[_robotY][_robotX-1].eastWall = true;
    } 
  }

  // Check left wall
  if (getLeftDistance() < 15) {
    if (_robotDir == 0) {
      _mazeGrid[_robotY][_robotX].westWall = true; // Facing North
      _mazeGrid[_robotY][_robotX - 1].eastWall = true;
    } else if (_robotDir == 1) {
      _mazeGrid[_robotY][_robotX].northWall = true; // Facing East
      _mazeGrid[_robotY + 1][_robotX].southWall = true;
    } else if (_robotDir == 2) {
      _mazeGrid[_robotY][_robotX].eastWall = true; // Facing South
      _mazeGrid[_robotY][_robotX + 1].westWall = true;
    } else if (_robotDir == 3) {
      _mazeGrid[_robotY][_robotX].southWall = true; // Facing West
      _mazeGrid[_robotY - 1][_robotX].northWall = true;
    }
  }

  // Check right wall
  if (getRightDistance() < 15) {
    if (_robotDir == 0) {
      _mazeGrid[_robotY][_robotX].eastWall = true; // Facing North
      _mazeGrid[_robotY][_robotX + 1].westWall = true;
    } else if (_robotDir == 1) {
      _mazeGrid[_robotY][_robotX].southWall = true; // Facing East
      _mazeGrid[_robotY - 1][_robotX].northWall = true;
    } else if (_robotDir == 2) {
      _mazeGrid[_robotY][_robotX].westWall = true; // Facing South
      _mazeGrid[_robotY][_robotX - 1].eastWall = true;
    } else if (_robotDir == 3) {
      _mazeGrid[_robotY][_robotX].northWall = true; // Facing West
      _mazeGrid[_robotY + 1][_robotX].southWall = true;
    }
  }

  updateFloodFill();
}

void moveToNextCell() {
  int minDistance = INT8_MAX;
  int nextDir = _robotDir;

  // Check neighboring cells based on walls
  if (!_mazeGrid[_robotY][_robotX].northWall && _robotY < MAZE_HEIGHT - 1 && _mazeGrid[_robotY + 1][_robotX].floodFillDistance < minDistance) {
    minDistance = _mazeGrid[_robotY + 1][_robotX].floodFillDistance;
    nextDir = 0; // North
  }
  if (!_mazeGrid[_robotY][_robotX].eastWall && _robotX < MAZE_WIDTH - 1 && _mazeGrid[_robotY][_robotX + 1].floodFillDistance < minDistance) {
    minDistance = _mazeGrid[_robotY][_robotX + 1].floodFillDistance;
    nextDir = 1; // East
  }
  if (!_mazeGrid[_robotY][_robotX].southWall && _robotY > 0 && _mazeGrid[_robotY - 1][_robotX].floodFillDistance < minDistance) {
    minDistance = _mazeGrid[_robotY - 1][_robotX].floodFillDistance;
    nextDir = 2; // South
  }
  if (!_mazeGrid[_robotY][_robotX].westWall && _robotX > 0 && _mazeGrid[_robotY][_robotX - 1].floodFillDistance < minDistance) {
    minDistance = _mazeGrid[_robotY][_robotX - 1].floodFillDistance;
    nextDir = 3; // West
  }

  int turnAngle = (nextDir - _robotDir + 4) % 4;
  unsigned int startLeftPulses;
  unsigned int startRightPulses;
  int targetPulses;
  int motorSpeed = 70;

  Serial.print("Next Direction: ");
  Serial.println(nextDir);

  switch (turnAngle)
  {
    case 0:
      // GO STRAIGHT
      break;
    case 1:
      drive(motorSpeed, 100, 0.4);
      break;
    case 2:
      drive(-motorSpeed, 50, 0.9);
      drive(motorSpeed, -50, 0.9);
      drive(-motorSpeed, 0, 0.5);
      break;
    case 3:
      drive(motorSpeed, -100, 0.4);
      break;
    default:
      // HELP
      break;
  } 
  
  targetPulses = _leftPulses + 28;
  while (_leftPulses < targetPulses && getFrontDistance() > 7.5) 
  {
    if(_rightDistance < 15 && _rightDistance < _leftDistance)
    {
      followRightWall(motorSpeed);
    }
    else if(_leftDistance < 15)
    {
      followLeftWall(motorSpeed);
    }
    else
    {
      _leftDistance = getLeftDistance();
      _rightDistance = getRightDistance();
      drive(motorSpeed);
    }
  }

  drive(0);
  _robotDir = nextDir;  

  // Update robot position
  if (_robotDir == 0) _robotY++;
  else if (_robotDir == 1) _robotX++;
  else if (_robotDir == 2) _robotY--;
  else if (_robotDir == 3) _robotX--;
}

void initializeMazeArray()
{
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
      _mazeGrid[y][x].posX = x;
      _mazeGrid[y][x].posY = y;
      _mazeGrid[y][x].floodFillDistance = UINT8_MAX;
      
      if (x == 0)
      {
        _mazeGrid[y][x].westWall = true;
      } 
      else
      {
        _mazeGrid[y][x].westWall = false;
      }

      if (x == MAZE_WIDTH - 1)
      {
        _mazeGrid[y][x].eastWall = true;
      }
      else
      {
        _mazeGrid[y][x].eastWall = false;
      } 

      if (y == 0)
      {
        _mazeGrid[y][x].southWall = true;
      }
      else
      {
        _mazeGrid[y][x].southWall = false;
      } 

      if (y == MAZE_HEIGHT - 1)
      {
        _mazeGrid[y][x].northWall = true;
      }
      else
      {
        _mazeGrid[y][x].northWall = false;
      } 
    }
  } 
}

void setLights(int startIndex, int endIndex, int r, int g, int b)
{
  for(int i = startIndex; i <= endIndex; i++)
  {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
}

void setGripper(int position)
{
  // Constrain the position to prevent errors
  position = constrain(position, 0, 180);

  // Calculate the pulse width for the given position
  int pulseWidth = map(position, 0, 180, 544, 2400);

  // Send the pulse to the gripper servo
  digitalWrite(GRIPPER_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(GRIPPER_PIN, LOW);

  // Wait for the servo to reach the position
  delay(20);
}

void followLine(int slowSpeed, int fastSpeed)
{
  float steerVal = 0; // Value that specifies the steering direction and intensity
  int diff = 0; // Calc value to determine how far off the robot is from the line and in which direction
  float Kp = 0.02; // Proportional constant for fine tuning the controller
  int speed;

  for (int i = 0; i < 6; i++) {
    int sensorValue = analogRead(_lineSensorPins[i]); // Read the sensor value from each sensor
    int adjustedValue = (sensorValue - _minValues[i]) * _line_sensor_modifiers[i] ; // Subtract the sensor minimums to reach a common baseline and multiply based on sensor position
    diff += adjustedValue; // Add the adjusted value to the diff variable
  }

  steerVal = Kp * diff; // Apply the proportional constant for fine tuning
  if(abs(steerVal) > 30)
  {
    speed = slowSpeed;
  }
  else
  {
    speed = fastSpeed;
  }
  drive(speed, steerVal); // Apply the drive function with the calculated steering value
}

void followRightWall(int speed)
{
  float wallDistance = getRightDistance();
  float targetDistance = 8.5; // Target distance from the wall
  float error = targetDistance - wallDistance; // The difference between the target distance and the actual distance
  float Kp = -15.0; // Proportional gain - fine tunung value

  // Calculate the steering adjustment based on the proportional controller
  float steerPercent = Kp * error;

  // Drive the robot with the calculated steering adjustment
  steerPercent = constrain(steerPercent, -33, 33);
  drive(speed, steerPercent);
}

void followLeftWall(int speed)
{
  float wallDistance = getLeftDistance();
  float targetDistance = 8.5; // Target distance from the wall
  float error = targetDistance - wallDistance; // The difference between the target distance and the actual distance
  float Kp = 15.0; // Proportional gain - fine tunung value

  // Calculate the steering adjustment based on the proportional controller
  float steerPercent = Kp * error;

  // Drive the robot with the calculated steering adjustment
  steerPercent = constrain(steerPercent, -33, 33);
  drive(speed, steerPercent);
}

// Function used for calculating the error of each of the line sensors
// Use in the begining of the execution
void calibrateSensors()
{
  int startTime = millis(); //mark the starting time
  int calibrationTime = 1350; //calibrate for this amount of tiems
  drive(55); //start slowly creeping forward
  
  // Drive over a small distance and mark the brightest and dimmest values for each sensor
  while(millis() - startTime < calibrationTime)
  {
    for (int i = 0; i < 6; i++)
    {
      int sensorValue = analogRead(_lineSensorPins[i]);
      if(sensorValue > _maxValues[i]) _maxValues[i] = sensorValue;
      if(sensorValue < _minValues[i]) _minValues[i] = sensorValue;
    }
  }
  drive(0); //stop the robot

  //calculate the tresholds (the average between the brightest and dimmest values)
  for (int i = 0; i < 6; i++)
  {
    _lineTresholds[i] = (_maxValues[i] + _minValues[i]) / 2;
  }
}

// Function for retrieving the distance from the left ultrasonic sensor
// This function sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
// Based on the speed of sound the distance is calculated
float getLeftDistance()
{
  digitalWrite(LEFT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(LEFT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_US_TRIG_PIN, LOW);

  float duration = pulseIn(LEFT_US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  _leftDistance = distance;

  delay(25); // Delay to prevent interference between measurements

  return distance;
}

// Function for retrieving the distance from the right ultrasonic sensor
// This function sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
// Based on the speed of sound the distance is calculated
float getRightDistance()
{
  digitalWrite(RIGHT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(RIGHT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_US_TRIG_PIN, LOW);

  float duration = pulseIn(RIGHT_US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  _rightDistance = distance;

  delay(25); // Delay to prevent interference between measurements

  return distance;
}

// Function for retrieving the distance from the left ultrasonic sensor
// This function sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
// Based on the speed of sound the distance is calculated
float getFrontDistance()
{
  digitalWrite(FRONT_US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_US_TRIG_PIN, LOW);

  float duration = pulseIn(FRONT_US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  _frontDistance = distance;

  delay(25); // Delay to prevent interference between measurements

  return distance;
}

//function for easy controll of the robot movement 
//speedPercent: -100 to 100, steerPercent: -100 to 100, numRotations: 0 to inf
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0)
{
  // Constrain the values to prevent errrors
  speedPercent = constrain(speedPercent, -100, 100);
  steerPercent = constrain(steerPercent + STEERING_ADJUSTMENT, -100, 100);
  numRotations = abs(numRotations);

  // Adjust speed based on steerPercent
  float leftSteerAdjustment = steerPercent < 0 ? -0.02 * -steerPercent + 1 : 1;
  float rightSteerAdjustment = steerPercent > 0 ? -0.02 * steerPercent + 1 : 1;
  int speedLeft = constrain((speedPercent * leftSteerAdjustment) * 2.55, -255, 255);
  int speedRight = constrain((speedPercent * rightSteerAdjustment) * 2.55, -255, 255);

  // Apply the motor speed
  setMotorSpeed(speedLeft, speedRight);

  // If rotations are specified drive the robot for the specified number of rotations
  if(numRotations != 0)
  {
    unsigned int startPulses = (steerPercent < 0 ? _rightPulses : _leftPulses);
    unsigned int targetPulses = startPulses + numRotations * PULSES_PER_REVOLUTION;
    while((steerPercent < 0 ? _rightPulses : _leftPulses) <= targetPulses)
    {

    }
    // Stop if the rotations are reached
    setMotorSpeed(0, 0);
  }
}

// Function for easy setting the motor speed
// leftSpeed: -255 to 255, rightSpeed: -255 to 255
// Does apply an initial boost to alow the motor to start
void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  // Constrain the values to prevent errors
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Determine which pins to power based on the direction
  int leftToDisable;
  int rightToDisable;
  int leftToPower;
  int rightToPower;
  
  if(leftSpeed < 0)
  {
    leftToDisable = L_FWD;
    leftToPower = L_BWD;
  }
  else
  {
    leftToDisable = L_BWD;
    leftToPower = L_FWD;
  }

  if(rightSpeed < 0)
  {
    rightToDisable = R_FWD;
    rightToPower = R_BWD;
  }
  else
  {
    rightToDisable = R_BWD;
    rightToPower = R_FWD;
  }

  // Disable the power for the oposite direction pins
  analogWrite(leftToDisable, 0);
  analogWrite(rightToDisable, 0);

  // Set the speed for the forward pins
  analogWrite(leftToPower, abs(leftSpeed));
  analogWrite(rightToPower, abs(rightSpeed));

  // Store the current speed for future refference
  _currentLeftSpeed = leftSpeed;
  _currentRightSpeed = rightSpeed;
}

// Interrupt function for counting the left rotation sensor pulses
void leftSensorPulse() {
  _leftPulses++;
}

// Interrupt function for counting the right rotation sensor pulses
void rightSensorPulse() {
  _rightPulses++;
}