#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

//function definitions
// Interrupt functions
void leftSensorPulse();
void rightSensorPulse();

// Sensor functuions
float getLeftDistance();
float getRightDistance();
float getFrontDistance();
bool allSensorsBlack();
bool anySensorBlack();

// Driving and motor control
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void turnToAngle(int angleGoalDegrees, int speed, int turnAngle = 100);
void setGripper(int position);

// Maze-navigation functions
void followRightWall(int speed);
void followLeftWall(int speed);

// Line-following function
void followLine(int slowSpeed, int fastSpeed);

// Neopixel light control
void setLights(int startIndex, int endIndex, int r, int g, int b);
void RGBLights();

// Main flow control functions
void calibrateSensors();
void navigateMazeLeftHand();
void navigateMazeRightHand();
void driveIntoMaze();
void driveOutOfMaze();

// Motor pins
#define L_FWD 5 // Left motor forward pin - A1
#define R_FWD 9 // Right motor forward pin - B1
#define L_BWD 6 // Left motor backward pin - A2
#define R_BWD 10 // Right motor backward pin - B2
#define L_ROT 2 // Left rotation sensor pin - R1
#define R_ROT 3 // Right rotation sensor pin -R2

// LINE SENSOR PINS
int _lineSensorPins[] = {A6, A7, A2, A3, A4, A5};
int _numLineSensors = 6;

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
#define STEERING_ADJUSTMENT 0 // Used to balance steering

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

// Timing vars
int _frontWallCheckDelay = 50;
int _nextFrontWallCheck = 0;

// Fine tuning variables
float _wallTargetDistance = 8.5;

// Controll vars
bool _rightHand = true;

void setup() {
  // Initialize the neopixels  
  pixels.begin();

  // Initialize serial communication for debuging
  Serial.begin(9600); 

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

  setLights(0, 3, 255, 0, 0); // Set the lights to blue

  while (getFrontDistance() > 25)
  {
    setGripper(GRIPPER_OPEN); // Ensure the gripper is open
    delay(100);
    // Wait for previous robot
  }
  
  setGripper(GRIPPER_OPEN); // Ensure the gripper is open

  delay(1000); // Wait for the previus robot to leave

  driveIntoMaze();
}

void loop() {
  drive(0);
  while (false)
  {
    for (int i = 0; i < 6; i++) {
      int sensorValue = analogRead(_lineSensorPins[i]);
      Serial.print(" Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(sensorValue);
      Serial.print("/");
      Serial.print(_lineTresholds[i]);
    }

    Serial.println();
  }
  
  bool lineDetected = false;
  bool checkpointDetected = false;
  unsigned long lightUpdateDelay = 50;
  unsigned long nextLightUpdate = millis();
  unsigned long checkpointUpdateDelay = 100;
  unsigned long nextCheckpointUpdate = millis() + 5000;

  setLights(0, 3, 0, 0, 255); // Set the lights to green
  
  nextCheckpointUpdate = millis() + 500;
  
  // Drive out of the maze following the black line
  while (!checkpointDetected)
  {
    followLine(80, 100);

    if(millis() > nextCheckpointUpdate)
    {
      if(allSensorsBlack())
      {
        drive(100, 0, 0.2);
        checkpointDetected = allSensorsBlack();
      }
      nextCheckpointUpdate = millis() + checkpointUpdateDelay;
    }
  }

  drive(0);
  while (true)
  {
    setGripper(GRIPPER_OPEN);
    setLights(0, 3, 255, 0, 0); // Set the lights to red
    delay(1000);
    setLights(0, 3, 0, 0, 0); // Set the lights to off
    delay(1000);
  }
}

void RGBLights()
{
  float f = 0.3f;  // Frequency in Hz
  float t = millis() / 1000.0;  // Convert millis to seconds

  for(int i = 0; i < 4; i++)
  {
    byte r = int(255 * abs(2 * fmod(t * f + ((float)(i * 0.25f) + 1.0)/3, 1) - 1));  // Main wave
    byte g = int(255 * abs(2 * fmod(t * f + ((float)(i * 0.25f) + 2.0)/3, 1) - 1));  // 120° phase shift
    byte b = int(255 * abs(2 * fmod(t * f + ((float)(i * 0.25f) + 3.0)/3, 1) - 1));  // 240° phase shift

    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }

  pixels.show();
}

bool allSensorsBlack()
{
  for(int i = 0; i < 6; i++)
  {
    if(analogRead(_lineSensorPins[i]) < _lineTresholds[i])
    {
      return false;
    }
  }
  return true;
}

bool anySensorBlack()
{
  for(int i = 0; i < 6; i++)
  {
    if(analogRead(_lineSensorPins[i]) > _lineTresholds[i])
    {
      return true;
    }
  }
  return false;
}

void navigateMazeLeftHand()
{
  followLeftWall(100);
  
  if(millis() > _nextFrontWallCheck && getFrontDistance() < 12)
  {    
    setLights(0, 3, 0, 0, 255); // Set the lights to blue - indicate different movement
    
    //determine the type of the turn - different situations require different allignments
    if(getRightDistance() < 15)
    {
      //turn around 180 degrees
      drive(-100, 50, 0.9);
      drive(100, -50, 0.9);
    }
    else
    {
      //turn right 90 degrees
      drive(100, 85, 0.5);
    }
    
    _nextFrontWallCheck += _frontWallCheckDelay;
  }
}

void navigateMazeRightHand()
{
  followRightWall(100);
  
  if(millis() > _nextFrontWallCheck && getFrontDistance() < 12)
  {    
    setLights(0, 3, 0, 0, 255); // Set the lights to orange - indicate different movement
    
    //determine the type of the turn - different situations require different allignments
    if(getLeftDistance() < 15)
    {
      //turn around 180 degrees
      drive(-100, -50, 0.9);
      drive(100, 50, 0.9);
    }
    else
    {
      //turn left 90 degrees
      drive(100, -85, 0.5);
    }

    _nextFrontWallCheck += _frontWallCheckDelay;
  }
}

void driveIntoMaze()
{
  // Indicate "override movement"
  setLights(0, 3, 255, 165, 0); // Set the lights to orange

  // Calibrate the line sensors
  // This will allow the robot to get close egougth to the cone to pick it up
  drive(100);
  delay(10);
  calibrateSensors();
  
  setGripper(GRIPPER_CLOSE); // Open the gripper

  delay(500); // Small delay to prevent cone bumping 

  // manually steer into the maze
  drive(60, 0, 0.25); // forward a bit
  drive(60, -35, 0.25); // steer left
  while (!anySensorBlack())
  {
    drive(60, -35); // steer left
  }

  // follow the short line untill in a set possition
  int startTime = millis();
  while(millis() < startTime + 1200)
  {
    followLine(85, 100);
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
  float targetDistance = _wallTargetDistance; // Target distance from the wall
  float error = targetDistance - wallDistance; // The difference between the target distance and the actual distance
  float Kp = -14; // Proportional gain - fine tunung value

  // Calculate the steering adjustment based on the proportional controller
  float steerPercent = Kp * error;

  // Drive the robot with the calculated steering adjustment
  steerPercent = constrain(steerPercent, -40, 40);
  drive(speed, steerPercent);
}

void followLeftWall(int speed)
{
  float wallDistance = getLeftDistance();
  float targetDistance = _wallTargetDistance - 0.5; // Target distance from the wall
  float error = targetDistance - wallDistance; // The difference between the target distance and the actual distance
  float Kp = 15; // Proportional gain - fine tunung value

  // Calculate the steering adjustment based on the proportional controller
  float steerPercent = Kp * error;

  // Drive the robot with the calculated steering adjustment
  steerPercent = constrain(steerPercent, -40, 40);
  drive(speed, steerPercent);
}

// Function used for calculating the error of each of the line sensors
// Use in the begining of the execution
void calibrateSensors()
{
  unsigned long startTime = millis(); //mark the starting time
  unsigned long calibrationTime = 1350; //calibrate for this amount of tiems
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
    _lineTresholds[i] = ((_maxValues[i] + _minValues[i]) / 2) - 50;
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