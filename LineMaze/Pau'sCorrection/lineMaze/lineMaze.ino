#include <Adafruit_NeoPixel.h>

// Pin Definitions

#define PIN 4           // NeoPixel data pin
#define NUMPIXELS 4     // Number of NeoPixels
#define BRIGHTNES_LEVEL 20  // NeoPixel brightness

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

// Gripper servo pin
#define GRIP 11  // Pin controlling servo motor

// Constants for Path and Movement Behavior

#define CLOSE 20    // Close distance threshold in cm
#define NORMAL 30   // Normal distance threshold in cm
#define FAR 100     // Far distance threshold in cm

#define TURN_90_LEFT 40      // Encoder ticks to turn 90° left
#define TURN_90_RIGHT 40     // Encoder ticks to turn 90° right
#define MOTOR_TURN_SPEED 170 // Speed for small turns
#define MOTOR_A_SPEED 245    // Driving speed for Motor A
#define MOTOR_B_SPEED 245    // Driving speed for Motor B
#define CHECK_STRAIGT_LINE_MOVEMENT 7 // Encoder ticks for straight motion
int BLACK_LIMIT = 775; // Detection threshold for black line


void setup() {
  /* Initialize serial communication, NeoPixels, pins, and motor encoders. */
  Serial.begin(9600);
  pixels.begin();
  
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

  // Light sensors (analog)
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  // Testing mode enables bypassing maze logic
  if (TESTING_MODE) started = true;

  // Game phases
  if (!started) {
    start();   // Perform startup and calibration
  } else if (!solved) {
    maze();    // Maze-solving logic
  } else if (!ended) {
    end();     // Post-solution logic and finishing behavior
  }
}

//Core Functions
void start() {
  // Check for an obstacle within `NORMAL` distance to start:
  int distance = culculateDistance();  
  while (distance >= NORMAL) {
    distance = culculateDistance();
  }

  // Calibrate color sensors to determine the `BLACK_LIMIT`.
  int blackLimit[3];
  for (int i = 0; i < 6; i++) {
    int currentColor = getAverageLightLevel();
    // Save calibration results and calculate black threshold
    if (i % 2 == 1) blackLimit[currentIndex++] = currentColor;
  }
  BLACK_LIMIT = getAverageBlackLimit(blackLimit) - 100;

  // Initial positioning adjustments
  startMovementAdjustment();
  grab();       // Close gripper on object
  turnLeft(TURN_90_LEFT); 

  started = true; // Start maze-solving phase
}

// Maze Navigation
void maze() {
  // Read sensor data
  read();

  // Conditions to decide movement:
  if (isLeftSensors()) {
    // Left sensors activated -> Adjust position by turning left
    turnLeft(TURN_90_LEFT);
  } else if (isRightSensors()) {
    // Right sensors activated -> Move straight and turn right later
    goStraight(CHECK_STRAIGT_LINE_MOVEMENT);
    turnRight(TURN_90_RIGHT);
  } else if (isNoSensors()) {
    // Lost the line, perform a bigger right turn
    turnRightUltra();
  } else if (sensor_A5 >= BLACK_LIMIT) {
    smallTurnRight(); // Small right turn for adjustment
  } else if (sensor_A2 >= BLACK_LIMIT) {
    smallTurnLeft();  // Small left turn for adjustment
  } else {
    goStraight();     // Default case: move straight
  }
}

// End Phase
void end() {
  goBack(5);  // Back off slightly
  ungrab();   // Release object
  goBack(30);
  
  // Signal completion with flashing NeoPixels
  while (true) {
    setPixlsRed(); delay(200);
    setPixlsYellow(); delay(200);
    setPixlsGreen(); delay(200);
  }
}

// Helper Functions:

//1.Distance Measurement
int culculateDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to centimeters
}

//2.Light Sensor Reading
void read() {
  sensor_A0 = analogRead(A0); sensor_A1 = analogRead(A1);
  sensor_A2 = analogRead(A2); sensor_A3 = analogRead(A3);
  sensor_A4 = analogRead(A4); sensor_A5 = analogRead(A5);
  sensor_A6 = analogRead(A6); sensor_A7 = analogRead(A7);
}

int getAverageLightLevel() {
  read(); // Call `read()` first
  return (sensor_A0 + sensor_A1 + ... + sensor_A7) / 8;
}
