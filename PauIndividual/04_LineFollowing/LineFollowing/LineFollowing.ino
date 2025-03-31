#include <Adafruit_NeoPixel.h>

// Encoder counters (volatile because they are updated in interrupts)
volatile int _countL = 0; // Left encoder counter
volatile int _countR = 0; // Right encoder counter

// Pin Definitions
#define PIN 4              // NeoPixel data pin
#define NUMPIXELS 4        // Number of NeoPixels
#define BRIGHTNES_LEVEL 20 // NeoPixel brightness

// NeoPixel object
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Motor control pins
#define MOT_A1 10  // Motor A forward pin
#define MOT_A2 9   // Motor A backward pin
#define MOT_B1 6   // Motor B forward pin
#define MOT_B2 5   // Motor B backward pin
#define MOT_R1 2   // Motor A encoder signal
#define MOT_R2 3   // Motor B encoder signal

// Distance sensor pins
#define TRIG 12  // Trigger pin for ultrasonic sensor
#define ECHO 13  // Echo pin for ultrasonic sensor

// Line Sensor Pins
const int _lineSensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int _numLineSensors = 8; // Corrected: Uses all 8 sensors

// Calibration values for the line sensors
int _minValues[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; 
int _maxValues[] = {500, 500, 500, 500, 500, 500, 500, 500};        
int _lineTresholds[] = {800, 800, 800, 800, 800, 800, 800, 800};     
float _line_sensor_modifiers[] = {6, 3.5, 1.75, 0, -1.75, -3.5, -6, -6}; // Balanced modifiers for 8 sensors with 0 neutral

// Gripper servo pin
#define GRIP 11  // Pin controlling servo motor

// Constants for Path and Movement Behavior
#define CLOSE 20    // Close distance threshold in cm
#define NORMAL 30   // Normal distance threshold in cm
#define FAR 100     // Far distance threshold in cm

#define TURN_90_LEFT 40      // Encoder ticks to turn 90° left
#define TURN_90_RIGHT 40     // Encoder ticks to turn 90° right
#define MOTOR_TURN_SPEED 170 // Speed for small turns
#define MOTOR_A_SPEED 170    // Driving speed for Motor A
#define MOTOR_B_SPEED 170    // Driving speed for Motor B
#define CHECK_STRAIGHT_LINE_MOVEMENT 7 // Encoder ticks for straight motion
int BLACK_LIMIT = 775; // Detection threshold for black line

// Interrupt Service Routines (for encoders)
void ISR_L() {
    _countL++; // Left encoder tick
}

void ISR_R() {
    _countR++; // Right encoder tick
}

void setup() {
  /* Initialize serial communication, NeoPixels, pins, and motor encoders. */
  Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(BRIGHTNES_LEVEL);
  
  // Motor control pins and encoder interrupts
  pinMode(MOT_A1, OUTPUT);
  pinMode(MOT_A2, OUTPUT);
  pinMode(MOT_B1, OUTPUT);
  pinMode(MOT_B2, OUTPUT);
  
  pinMode(MOT_R1, INPUT_PULLUP);
  pinMode(MOT_R2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOT_R1), ISR_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT_R2), ISR_L, CHANGE);

  // Distance sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Gripper pin
  pinMode(GRIP, OUTPUT);
  digitalWrite(GRIP, LOW);

  // Set line sensor pins
  for (int i = 0; i < _numLineSensors; i++) {
    pinMode(_lineSensorPins[i], INPUT);
  }

  // Optional: Calibrate sensor when starting up
  calibrateSensors();
}

void loop() {
  // Continuously read sensor data to follow the black line
  followLine();
}

// Core Functions

void followLine() {
    float diff = 0; // Difference for alignment
    float Kp = 0.03; // Proportional constant (adjusted for slightly stronger corrections)

    // Calculate steering value based on line sensor data
    for (int i = 0; i < _numLineSensors; i++) {
        int sensorValue = analogRead(_lineSensorPins[i]); // Read sensor value
        int adjustedValue = (sensorValue - _minValues[i]) * _line_sensor_modifiers[i]; // Normalize and weight
        diff += adjustedValue;
    }

    float steerValue = Kp * diff; // Compute steering adjustment
    steerValue = constrain(steerValue, -100, 100); // Limit adjustment range

    // Set motor speeds based on steering adjustment
    if (steerValue > 0) {
        smallTurnRight(steerValue); // Turn right
    } else if (steerValue < 0) {
        smallTurnLeft(-steerValue); // Turn left
    } else {
        goStraight(); // Move straight
    }
}

// Helper Functions

// 1. Motor Control

void goStraight() {
    analogWrite(MOT_A2, MOTOR_A_SPEED);
    analogWrite(MOT_B2, MOTOR_B_SPEED);
    analogWrite(MOT_A1, LOW);
    analogWrite(MOT_B1, LOW);
}

void smallTurnLeft(float adjustment) {
    analogWrite(MOT_B2, MOTOR_B_SPEED - adjustment); // Reduce right motor speed
    analogWrite(MOT_A2, MOTOR_A_SPEED);
}

void smallTurnRight(float adjustment) {
    analogWrite(MOT_A2, MOTOR_A_SPEED - adjustment); // Reduce left motor speed
    analogWrite(MOT_B2, MOTOR_B_SPEED);
}

void stop() {
    analogWrite(MOT_A1, LOW);
    analogWrite(MOT_B1, LOW);
    analogWrite(MOT_A2, LOW);
    analogWrite(MOT_B2, LOW);
}

// 2. Calibration and Sensor Utility Functions

void calibrateSensors() {
    int startTime = millis();
    int calibrationTime = 1350; // Time for calibration in ms
    goStraight(); // Robot moves forward during calibration

    while (millis() - startTime < calibrationTime) {
        for (int i = 0; i < _numLineSensors; i++) {
            int sensorValue = analogRead(_lineSensorPins[i]);
            _maxValues[i] = max(_maxValues[i], sensorValue);
            _minValues[i] = min(_minValues[i], sensorValue);
        }
    }

    stop();

    // Calculate thresholds with a -100 offset for robustness
    for (int i = 0; i < _numLineSensors; i++) {
        _lineTresholds[i] = ((_maxValues[i] + _minValues[i]) / 2) - 100;
    }
}

bool allSensorsBlack() {
    for (int i = 0; i < _numLineSensors; i++) {
        if (analogRead(_lineSensorPins[i]) < _lineTresholds[i]) {
            return false;
        }
    }
    return true;
}

bool anySensorBlack() {
    for (int i = 0; i < _numLineSensors; i++) {
        if (analogRead(_lineSensorPins[i]) > _lineTresholds[i]) {
            return true;
        }
    }
    return false;
}


