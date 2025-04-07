#include <Adafruit_NeoPixel.h> //libary for the neoLEDs
// Motor A (Left)
const int MOTOR_A_1 = 5;  // Motor A control pin 1 (left forward)
const int MOTOR_A_2 = 6;  // Motor A control pin 2  (reverse for the left wheel)
// LED pins 
#define NeoLed 7
#define NUM_PIXELS 4
// Motor B (Right)
const int MOTOR_B_1 = 9;  // Motor B control pin 1 (backwards for the right wheel)
const int MOTOR_B_2 = 10; // Motor B control pin 2 (forward for the right wheel)


// Rotation detector pins
int rotationDetector1 = 2;
int rotationDetector2 = 3;

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLed, NEO_GRB + NEO_KHZ800);
void setup() {
  // Configure motor control pins as OUTPUT
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);

  strip.begin();
  setLEDColor(0,0,0);
  strip.show();
  // Configure rotation detector pins as INPUT
  pinMode(rotationDetector1, INPUT);
  pinMode(rotationDetector2, INPUT);
}
void setLEDColor(int g, int r, int b){
  for(int i = 0; i <NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(g, r, b));
  }
  strip.show();
}

// Move forward
void moveForward() {
  digitalWrite(MOTOR_A_1, HIGH);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, HIGH);
   setLEDColor(255, 0, 0); // green
}

// Move backward
void moveBackward() {
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, HIGH);
  digitalWrite(MOTOR_B_1, HIGH);
  digitalWrite(MOTOR_B_2, LOW);
  setLEDColor(50, 255, 0); // s orange
}

// Turn left
void turnLeft() {
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, HIGH);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, HIGH);
  setLEDColor(0, 240, 200); //purple

}

// Turn right
void turnRight() {
  digitalWrite(MOTOR_A_1, HIGH);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, HIGH);
  digitalWrite(MOTOR_B_2, LOW);
  setLEDColor(0, 0, 250);
}

// Stop motors
void stopMotors() {
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
  setLEDColor(0, 255, 0); // Red
}


void loop() {
  moveForward();
  delay(2000);

  stopMotors();
  delay(500);

  turnLeft();
  delay(800);

  stopMotors();
  delay(500);

  moveForward();
  delay(3000);

  stopMotors();
  delay(500);

  turnRight();
  delay(800);

  stopMotors();
  delay(500);

  moveBackward();
  delay(3000);

}

