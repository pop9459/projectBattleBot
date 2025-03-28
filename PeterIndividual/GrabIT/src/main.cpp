#include <Arduino.h>

//function definitions

void leftSensorPulse();
void rightSensorPulse();

void drive(int speedPercent, int steerPercent = 0, float numRotations = 0);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void followRightWall(int speed);
void setGripper(int position);

// Motor pins
#define L_FWD 5 // Left motor forward pin
#define R_FWD 9 // Right motor forward pin
#define L_BWD 6 // Left motor backward pin
#define R_BWD 10 // Right motor backward pin
#define L_ROT 2 // Left rotation sensor pin
#define R_ROT 3 // Right rotation sensor pin

// LINE SENSOR PINS
int _lineSensorPins[] = {A6, A7, A2, A3, A4, A5};

// Gripper constants
#define GRIPPER_PIN 11 // Gripper servo pin
#define GRIPPER_OPEN 100 // Gripper open position
#define GRIPPER_CLOSE 40 // Gripper close position

// Driving constants
#define PULSES_PER_REVOLUTION 20 // Number of pulses the rotation sensor outputs per revolution 
#define STEERING_ADJUSTMENT -2 // Used to balance the speed

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

  //reset pins
  digitalWrite(L_FWD, LOW);
  digitalWrite(R_FWD, LOW);
  digitalWrite(L_BWD, LOW);
  digitalWrite(R_BWD, LOW);
}

void loop() {
  setGripper(GRIPPER_OPEN);
  
  delay(1000);

  setGripper(GRIPPER_CLOSE);
  
  delay(1000);

  setGripper(GRIPPER_OPEN);
  
  drive(100, 0, 1);

  _leftPulses = 0;
  _rightPulses = 0;
  drive(100);
  while (_leftPulses < 20 && _rightPulses < 20)
  {
    setGripper(GRIPPER_CLOSE);
  }
  drive(0);
  while (true)
  {
    if(millis() % 20 == 0)
    {
      setGripper(GRIPPER_CLOSE);
    }
  }
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