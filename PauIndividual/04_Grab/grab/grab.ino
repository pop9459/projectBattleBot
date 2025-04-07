// Motor A (pins 5 and 6)
int motorA1 = 5; // Motor A control pin 1
int motorA2 = 6; // Motor A control pin 2

// Motor B (pins 9 and 10)
int motorB1 = 9;  // Motor B control pin 1
int motorB2 = 10; // Motor B control pin 2
const int speedValue = 90;

// Gripper pin
#define GRIP 11

// Rotation detector pins
int rotationDetector1 = 2;
int rotationDetector2 = 3;

void setup() {
  // Configure motor control pins as OUTPUT
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Configure rotation detector pins as INPUT
  pinMode(rotationDetector1, INPUT);
  pinMode(rotationDetector2, INPUT);

  pinMode(GRIP, OUTPUT); // Gripper pin as OUTPUT

  analogWrite(motorA1, speedValue); // Motor A speed: PWM value (0-255)
  analogWrite(motorA2, speedValue); 
  analogWrite(motorB1, speedValue);
  analogWrite(motorB2, speedValue); 
}

void loop() {
  grab(); // Close the gripper
  delay(1000);
  ungrab(); // Open the gripper
  delay(1000);
  grab();
  delay(1000);
  ungrab();
  moveForward();
  delay(1000);
  stopMotors();
  grab();
  delay(1000);
  moveForward();
  delay(1000);
  stopMotors();
  ungrab();
  delay(1000);
}

void gripper(int newPulse) {
  static unsigned long timer;
  static int pulse;
  if (millis() > timer) {
    if (newPulse > 0) {
      pulse = newPulse;
    }
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIP, LOW);
    timer = millis() + 20; // Update every 20 milliseconds
  }
}

// Grab with servo
void grab() {
  Serial.println("grab");
  unsigned long startTime = millis();
  while (millis() - startTime < 300) { // Perform grab action for 300 milliseconds
    gripper(1500); // Typical pulse for "grab" action
  }
}

// Ungrab with servo
void ungrab() {
  Serial.println("ungrab");
  unsigned long startTime = millis();
  while (millis() - startTime < 300) { // Perform ungrab action for 300 milliseconds
    gripper(1000); // Typical pulse for "ungrab" action
  }
}

// Move forward
void moveForward() {
  digitalWrite(motorA1, HIGH); // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
}

// Move backward
void moveBackward() {
  digitalWrite(motorA1, LOW); // Motor A backward
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW); // Motor B backward
  digitalWrite(motorB2, HIGH);
}

// Turn left
void turnLeft() {
  digitalWrite(motorA1, LOW); // Motor A backward
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
  delay(400);
}

// Turn right
void turnRight() {
  digitalWrite(motorA1, HIGH); // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); // Motor B backward
  digitalWrite(motorB2, HIGH);
  delay(400);
}

// Stop all motors
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
