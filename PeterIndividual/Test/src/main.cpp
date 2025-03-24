#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DS3TRC.h>

Adafruit_LSM6DS3TRC lsm6ds;
//function definitions

void leftSensorPulse();
void rightSensorPulse();

float getRpmLeft();
float getRpmRight();

float getDistance(int echoPin, int triggerPin);

void calibrateSensors();
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void followRightWall(int speed);
void followLine(int slowSpeed, int fastSpeed);
void setGripper(int position);
void setLights(int startIndex, int endIndex, int r, int g, int b);
void turnToAngle(int angleGoalDegrees, int speed);


// Motor pins
#define L_FWD 5 // Left motor forward pin
#define R_FWD 9 // Right motor forward pin
#define L_BWD 6 // Left motor backward pin
#define R_BWD 10 // Right motor backward pin
#define L_ROT 2 // Left rotation sensor pin
#define R_ROT 3 // Right rotation sensor pin

// LINE SENSOR PINS
int _lineSensorPins[] = {A6, A7, A2, A3};

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
#define GRIPPER_OPEN 110 // Gripper open position
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

void setup() {
  // Initialize the neopixels  
  pixels.begin();
  setLights(0, 3, 255, 0, 0); // Set the lights to blue

  // Initialize serial communication for debuging
  Serial.begin(9600); 

  setGripper(GRIPPER_OPEN); // open the gripper

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

  //set the line sensor pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);

  //gyroscope pins
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  if (!lsm6ds.begin_I2C()) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1);
  }
  Serial.println("LSM6DS3TR-C Found!");

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

  delay(1000);
}

void loop() {
  turnToAngle(90, 60);
  delay(500);

  turnToAngle(-90, 60);
  delay(500);
}

void turnToAngle(int angleGoalDegrees, int speed)
{
  float angleGoal = map(angleGoalDegrees, 0, 90, 0, -15);
  float currentAngle = 0;

  int angleScanDelay = 100;
  int nextScanTime = millis();

  if(angleGoalDegrees < 0)
  {
    drive(speed, -100);
  }
  else
  {
    drive(speed, 100);
  }
  
  Serial.println(angleGoal);
  while ((angleGoal > 0 && angleGoal > currentAngle) || (angleGoal < 0 && angleGoal < currentAngle))
  {
    if(millis() > nextScanTime)
    {
      sensors_event_t accel, gyro, temp;
      lsm6ds.getEvent(&accel, &gyro, &temp);

      if(abs(gyro.gyro.z) > 0.05)
      {
        currentAngle += gyro.gyro.z;
      }
      nextScanTime = millis() + angleScanDelay;
      Serial.println(currentAngle);
    }
  }

  drive(0);
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
  float distance = getDistance(RIGHT_US_ECHO_PIN, RIGHT_US_TRIG_PIN);
  float targetDistance = 8; // Target distance from the wall
  float error = targetDistance - distance; // The difference between the target distance and the actual distance
  float Kp = -10.0; // Proportional gain - fine tunung value

  // Calculate the steering adjustment based on the proportional controller
  float steerPercent = Kp * error;

  // Drive the robot with the calculated steering adjustment
  steerPercent = constrain(steerPercent, -35, 35);
  drive(speed, steerPercent);
  
  float frontDistance = getDistance(FRONT_US_ECHO_PIN, FRONT_US_TRIG_PIN);
  if(frontDistance < 12)
  {
    drive(0);
    delay(1000);
    Serial.println(getDistance(LEFT_US_ECHO_PIN, LEFT_US_TRIG_PIN));
    while (true)
    {
      /* code */
    }
    
    setLights(0, 3, 255, 165, 0); // Set the lights to orange - indicate different movement
    //determine the type of the turn - different situations require different allignments
    if(getDistance(LEFT_US_ECHO_PIN, LEFT_US_TRIG_PIN) < 15)
    {
      //turn around 180 degrees
      drive(-100, 50, 0.9);
      drive(100, -50, 0.9);
    }
    else
    {
      //turn right 90 degrees
      drive(-100, 85, 0.5);
    }
    setLights(0, 3, 0, 255, 0); // Set the lights to green - indicate normal movement
  }
}

// Function used for calculating the error of each of the line sensors
// Use in the begining of the execution
void calibrateSensors()
{
  int startTime = millis(); //mark the starting time
  int calibrationTime = 1350; //calibrate for 3 seconds
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

// Function for retrieving the distance from the ultrasonic sensor
// This function sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
// Based on the speed of sound the distance is calculated
float getDistance(int echoPin, int triggerPin)
{
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

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