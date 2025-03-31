#include <Arduino.h>

// Function definitions
void leftSensorPulse();
void rightSensorPulse();

float getRpmLeft();
float getRpmRight();

float getDistance();

void calibrateSensors();
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);

// Motor pins
#define L_FWD 9 // Left motor forward pin
#define R_FWD 5  // Right motor forward pin
#define L_BWD 10  // Left motor backward pin
#define R_BWD 6  // Right motor backward pin
#define L_ROT 2  // Left rotation sensor pin (encoder signal)
#define R_ROT 3  // Right rotation sensor pin (encoder signal)

// Line sensor pins
const int LINE_SENSOR_PINS[] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Ultrasonic sensor pins
#define US_TRIG_PIN 12 // Ultrasonic sensor trigger pin
#define US_ECHO_PIN 13 // Ultrasonic sensor echo pin

#define PULSES_PER_REVOLUTION 20 // Number of pulses the rotation sensor outputs per revolution 
#define STEERING_ADJUSTMENT -2 // Used to balance the speed

volatile unsigned int _leftPulses = 0; // Number of pulses counted by the left rotation sensor
volatile unsigned int _rightPulses = 0; // Number of pulses counted by the right rotation sensor

// Calibration values for the line sensors
int _minValues[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; // DARKEST VALUES
int _maxValues[] = {500, 500, 500, 500, 500, 500, 500, 500};         // LIGHTEST VALUES
int _lineTresholds[] = {800, 800, 800, 800, 800, 800, 800, 800};     // Threshold for the line sensors
float _line_sensor_modifiers[] = {6, 3.5, 1.75, 0, -1.75, -3.5, -6, -6}; // Weights for adjusting the steering intensity based on sensor position

int _currentLeftSpeed = 0;  // Current speed of the left motor
int _currentRightSpeed = 0; // Current speed of the right motor

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600); 

  // Begin the interrupt functions to count the rotations
  attachInterrupt(digitalPinToInterrupt(L_ROT), leftSensorPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ROT), rightSensorPulse, RISING);

  // Set motor pins
  pinMode(L_FWD, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(L_BWD, OUTPUT);
  pinMode(R_BWD, OUTPUT);
  pinMode(L_ROT, INPUT);
  pinMode(R_ROT, INPUT);

  // Set line sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }

  // Reset pins
  digitalWrite(L_FWD, LOW);
  digitalWrite(R_FWD, LOW);
  digitalWrite(L_BWD, LOW);
  digitalWrite(R_BWD, LOW);

  // Set ultrasonic sensor pins
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  // Calibrate the line sensors
  calibrateSensors();

  // Wait for user input from the ultrasonic sensor before starting the main loop
  while (getDistance() > 10) {
    // Wait for signal
  } 
}

void loop() {
  float steerVal = 0; // Value that specifies the steering direction and intensity
  int diff = 0;       // Calc value to determine how far off the robot is from the line and in which direction
  float Kp = 0.02;    // Proportional constant for fine-tuning the controller
  int speed;

  for (int i = 0; i < 8; i++) {
    int sensorValue = analogRead(LINE_SENSOR_PINS[i]); // Read the sensor value from each sensor
    int adjustedValue = (sensorValue - _minValues[i]) * _line_sensor_modifiers[i]; // Subtract the sensor minimums to reach a common baseline and multiply based on sensor position
    diff += adjustedValue; // Add the adjusted value to the diff variable
  }

  steerVal = Kp * diff; // Apply the proportional constant for fine-tuning
  if(abs(steerVal) > 30) {
    speed = 85;
  } else {
    speed = 100;
  }
  drive(speed, steerVal); // Apply the drive function with the calculated steering value
}

// Function used for calculating the error of each of the line sensors
void calibrateSensors() {
  int startTime = millis(); // Mark the starting time
  int calibrationTime = 1500; // Calibrate for 1.5 seconds
  drive(55); // Start slowly creeping forward
  
  // Drive over a small distance and mark the brightest and dimmest values for each sensor
  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 8; i++) {
      int sensorValue = analogRead(LINE_SENSOR_PINS[i]);
      if(sensorValue > _maxValues[i]) _maxValues[i] = sensorValue;
      if(sensorValue < _minValues[i]) _minValues[i] = sensorValue;
    }
  }
  drive(0); // Stop the robot

  // Calculate the thresholds (the average between the brightest and dimmest values)
  for (int i = 0; i < 8; i++) {
    _lineTresholds[i] = (_maxValues[i] + _minValues[i]) / 2;
  }
}

// Function for retrieving the distance from the ultrasonic sensor
float getDistance() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  float duration = pulseIn(US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}

// Function for easy control of robot movement
void drive(int speedPercent, int steerPercent = 0, float numRotations = 0) {
  // Constrain values to prevent errors
  speedPercent = constrain(speedPercent, -100, 100);
  steerPercent = constrain(steerPercent + STEERING_ADJUSTMENT, -100, 100);
  numRotations = abs(numRotations);

  // Adjust speed based on steerPercent
  float leftSteerAdjustment = steerPercent < 0 ? -0.02 * -steerPercent + 1 : 1;
  float rightSteerAdjustment = steerPercent > 0 ? -0.02 * steerPercent + 1 : 1;
  int speedLeft = constrain((speedPercent * leftSteerAdjustment) * 2.55, -255, 255);
  int speedRight = constrain((speedPercent * rightSteerAdjustment) * 2.55, -255, 255);

  // Apply motor speed
  setMotorSpeed(speedLeft, speedRight);

  // If rotations are specified, drive the robot for the specified number of rotations
  if(numRotations != 0) {
    unsigned int startPulses = (steerPercent < 0 ? _rightPulses : _leftPulses);
    unsigned int targetPulses = startPulses + numRotations * PULSES_PER_REVOLUTION;
    while ((steerPercent < 0 ? _rightPulses : _leftPulses) <= targetPulses) {
      // Continue until rotations are reached
    }
    setMotorSpeed(0, 0); // Stop the robot
  }
}

// Function for setting motor speeds
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
 
  // Disable opposite power
  analogWrite(leftSpeed < 0 ? L_FWD : L_BWD, 0);
  analogWrite(rightSpeed < 0 ? R_FWD : R_BWD, 0);

  // Set speed
  analogWrite(leftSpeed < 0 ? L_BWD : L_FWD, abs(leftSpeed));
  analogWrite(rightSpeed < 0 ? R_BWD : R_FWD, abs(rightSpeed));

  // Store current speed
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

