#include <Arduino.h>

//function definitions

void driveRot(float rotationsToDrive);
void leftSensorPulse();
void rightSensorPulse();

float getRpmLeft();
float getRpmRight();

float getDistance();

void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);

// Motor pins
#define L_FWD 5 // Left motor forward pin
#define R_FWD 9 // Right motor forward pin
#define L_BWD 6 // Left motor backward pin
#define R_BWD 10 // Right motor backward pin
#define L_ROT 2 // Left rotation sensor pin
#define R_ROT 3 // Right rotation sensor pin

// Ultrasonic sensor pins
#define US_TRIG_PIN 12 // Ultrasonic sensor trigger pin
#define US_ECHO_PIN 13 // Ultrasonic sensor echo pin

#define PULSES_PER_REVOLUTION 20 // Number of pulses the rotation sensor outputs per revolution 
#define STEERING_ADJUSTMENT -2 // Used to balance the speed

volatile unsigned int _leftPulses; //number of pulses counted by the left rotation sensor
volatile unsigned int _rightPulses; //number of pulses counted by the right rotation sensor

int _currentLeftSpeed = 0; //current speed of the left motor
int _currentRightSpeed = 0; //current speed of the right motor

void setup() {
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

  //set the ultrasonic sensor pins
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
}

void loop() {
  // Drive the robot forward until an object is detected
  if(getDistance() < 10)
  {
    // preprogramed sequence of moves for avoiding the object
    drive(50, 100, 0.45);
    drive(50, 0, 0.75);
    drive(50, -100, 0.45);
    drive(50, 0, 1.5);
    drive(50, -100, 0.45);
    drive(50, 0, 0.75);
    drive(50, 100, 0.45);
  }
  else
  {
    // If no object is detected drive forward
    drive(50);
  }
}

// Function for retrieving the distance from the ultrasonic sensor
float getDistance()
{
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  float duration = pulseIn(US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

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
 
  // Disable the power for the oposite 
  analogWrite(leftSpeed < 0 ? L_FWD : L_BWD, 0);
  analogWrite(rightSpeed < 0 ? R_FWD : R_BWD, 0);
  // Provide a speed boost for the motor to start
  if(leftSpeed != 0 && _currentLeftSpeed != leftSpeed) analogWrite(leftSpeed < 0 ? L_BWD : L_FWD, 255);
  if(rightSpeed != 0 && _currentRightSpeed != rightSpeed) analogWrite(rightSpeed < 0 ? R_BWD : R_FWD, 255);
  delay(40);

  // Set the speed
  analogWrite(leftSpeed < 0 ? L_BWD : L_FWD, abs(leftSpeed));
  analogWrite(rightSpeed < 0 ? R_BWD : R_FWD, abs(rightSpeed));

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