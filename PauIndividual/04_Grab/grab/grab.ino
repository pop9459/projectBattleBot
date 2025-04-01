// Motor A (pins 5 and 6)
int motorA1 = 5; // Motor A control pin 1
int motorA2 = 6; // Motor A control pin 2

// Motor B (pins 9 and 10)
int motorB1 = 9;  // Motor B control pin 1
int motorB2 = 10; // Motor B control pin 2
const int speedValue = 90;
// Rotation detector pins
int rotationDetector1 = 2;
int rotationDetector2 = 3;

//Gripper pin
#define GRIP 11

void setup() {
  // Configure motor control pins as OUTPUT
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Configure rotation detector pins as INPUT
  pinMode(rotationDetector1, INPUT);
  pinMode(rotationDetector2, INPUT);

  analogWrite(motorA1, speedValue); // Motor A speed: PWM value (0-255)
  analogWrite(motorA2, speedValue); 
  analogWrite(motorB1, speedValue);
  analogWrite(motorB2, speedValue); 
}

void loop() {
  grab();
  delay(1000)
  ungrab();
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

// Grab with servo
void grab() {
  Serial.println("grab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(19000);
  }
}

// Ungrab with servo
void ungrab() {
  Serial.println("ungrab");
  for (int i = 0; i < 15; i++) {
    digitalWrite(GRIP, HIGH);
    delayMicroseconds(1500);
    digitalWrite(GRIP, LOW);
    delayMicroseconds(18500);
  }
}

void moveForward() {
  digitalWrite(motorA1, HIGH); // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
}

void moveBackward() {
  digitalWrite(motorA1, LOW); // Motor A backward
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW); // Motor B backward
  digitalWrite(motorB2, HIGH);
}

void turnLeft() {
  digitalWrite(motorA1, LOW); // Motor A backward
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
  delay(400);
}

void turnRight() {
  digitalWrite(motorA1, HIGH); // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW); // Motor B backward
  digitalWrite(motorB2, HIGH);
  delay(400);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
