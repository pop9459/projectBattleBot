// Motor A (pins 5 and 6)
int motorA1 = 5; // Motor A control pin 1
int motorA2 = 6; // Motor A control pin 2

// Motor B (pins 9 and 10)
int motorB1 = 9;  // Motor B control pin 1
int motorB2 = 10; // Motor B control pin 2

// Ultrasonic Sensor (pins 12 and 13)
int trigPin = 12;
int echoPin = 13;

// Define distance threshold (in cm)
const int _mazeDistanceThreshold = 10;

// Speed Value
const int speedValue = 90;

void setup() {
  // Initialize serial monitor for debugging
  Serial.begin(9600);

  // Configure motor control pins as OUTPUT
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Read distance from ultrasonic sensor
  int mazeDistance = getMazeDistance();

  // Debugging: Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(mazeDistance);

  if (mazeDistance > _mazeDistanceThreshold || mazeDistance == 0) {
    // No obstacle detected: Move forward
    moveForward();
  } else {
    // Obstacle detected: Avoid it
    stopMotors();
    delay(500);  // Pause briefly
    turnRight();
    moveForward();
    delay(500);
    stopMotors();
    delay(500);
    turnLeft();
    moveForward();
    delay(1500);
    stopMotors();
    delay(500);
    turnLeft();
    moveForward();
    delay(500);
    stopMotors();
    delay(500);
    turnRight();
    moveForward();
    delay(1000);
  }
}

void moveForward() {
  digitalWrite(motorA1, HIGH);  // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void turnRight() {
  digitalWrite(motorA1, HIGH);  // Motor A forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);  // Motor B backward
  digitalWrite(motorB2, HIGH);
  delay(400);  // Adjust this delay for an accurate 90-degree turn
  stopMotors();
}

void turnLeft() {
  digitalWrite(motorA1, LOW);  // Motor A backward
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, HIGH); // Motor B forward
  digitalWrite(motorB2, LOW);
  delay(400);  // Adjust this delay for an accurate 90-degree turn
  stopMotors();
}

int getMazeDistance() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo pulse duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance in cm
  int mazeDistance = duration * 0.034 / 2;  // Speed of sound: 0.034 cm/μs
  return mazeDistance;
}

void avoidObstacle() {
  // Turn to the right (90°)
  Serial.println("Turning right to go around obstacle");
  turnRight();

  // Move forward to bypass the obstacle
  Serial.println("Bypassing the obstacle");
  moveForward();
  delay(1000);  // Move forward for 1 second to clear the obstacle

  // Turn back to the left (90°) to return to the original direction
  Serial.println("Turning back to original direction");
  turnLeft();

  // Continue moving forward along the original straight line
  moveForward();
}
