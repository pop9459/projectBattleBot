#include <Adafruit_NeoPixel.h>

#define NeoLed 7
#define NUM_PIXELS 4

#define NUM_SENSORS 8  // Number of sensors

int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Sensor pin mapping
int sensorValues[NUM_SENSORS];  // Array to store sensor readings
int sensorThreshold[NUM_SENSORS]; // Calibration thresholds for each sensor

// Motor control pins (adapted for two connections per motor)
const int MOTOR_A_1 = 5;  // Left Motor Forward
const int MOTOR_A_2 = 6;  // Left Motor Reverse
const int MOTOR_B_1 = 10;  // Right Motor Forward
const int MOTOR_B_2 = 9; // Right Motor Reverse

// HC-SR04 sensor pins
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;

// Speed settings
int leftSpeed = 245;  // Adjust base speed (0-255)
int baseSpeed = 170;  // Adjust base speed (0-255)
int rightSpeed = 245;
int turnSpeed = 80;



Adafruit_NeoPixel strip(NUM_PIXELS, NeoLed, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
    
  pinMode(MOTOR_A_1, OUTPUT);
  pinMode(MOTOR_A_2, OUTPUT);
  pinMode(MOTOR_B_1, OUTPUT);
  pinMode(MOTOR_B_2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();

  calibrateSensors();

  strip.begin();
  setLEDColor(0,0,0);
  strip.show();
}
// Move Forward
void moveForward() {
    analogWrite(MOTOR_A_1, baseSpeed);
    analogWrite(MOTOR_A_2, 0);  

    analogWrite(MOTOR_B_1, baseSpeed);
    analogWrite(MOTOR_B_2, 0);

    setLEDColor(255, 0, 0); // green
}

// Turn Left (Reduce left motor speed)
void turnLeft() {
    analogWrite(MOTOR_A_1, turnSpeed);  // Slow left motor
    analogWrite(MOTOR_A_2, 0);

    analogWrite(MOTOR_B_1, leftSpeed);
    analogWrite(MOTOR_B_2, 0);
  
    setLEDColor(0, 240, 200); //purple
}

// Turn Right (Reduce right motor speed)
void turnRight() {
    analogWrite(MOTOR_A_1, rightSpeed);
    analogWrite(MOTOR_A_2, 0);

    analogWrite(MOTOR_B_1, turnSpeed);  // Slow right motor
    analogWrite(MOTOR_B_2, 0);

    setLEDColor(0, 0, 250); // blue
}

// Move backward
void moveBackward() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, baseSpeed);

    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, baseSpeed);

    setLEDColor(50, 255, 0); // Orange
}

// Stop Motors
void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);

    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);

    setLEDColor(0, 255, 0); // Red
}


void setLEDColor(int g, int r, int b){
  for(int i = 0; i <NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(g, r, b));
  }
  strip.show();
}
void loop() {
    readSensors();
    int position = calculateLinePosition();

    Serial.print("Line Position: ");
    Serial.println(position);

    followLine(position);

    delay(100);
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = analogRead(sensorPins[i]);
  }
  Serial.println("Calibration complete");
}

// Function to read sensor values
void readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }
    Serial.println();
}

// Function to calculate line position based on sensor readings
int calculateLinePosition() {
  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    weightedSum += (long)value * i * 1000;
    sum += value;
  }

  if (sum < 3200 || sum > 8000) {
    return -1;  // No line detected
  }

  return weightedSum / sum;  // Normalized position
}

// Function to follow the line
void followLine(int position) {
  if (position == -1) {
        Serial.println("Line lost - Searching...");

        for (int i = 1; i <= 3; i++) {  // Increasing turn duration
            turnRight(); // Turn right slightly
            delay(400 * i);
            readSensors();
            if (calculateLinePosition() != -1) return;

            turnLeft(); // Turn left slightly
            delay(400 * i);
            readSensors();
            if (calculateLinePosition() != -1) return;

            moveBackward(); // move backward in search of the line
            delay(500 * i);
            readSensors();
            if (calculateLinePosition() != -1) return;
  
        }

        moveBackward();  // Move slightly backward if still lost
        delay(500);
        stopMotors();
        return;
    }

    int center = (NUM_SENSORS - 1) * 1000 / 2;  // Midpoint of sensor array
    int error = position - center;  // Deviation from center

    if (error > 240) {  // Line is to the left
        turnLeft();
        Serial.print("Error: ");
        Serial.print(error);
        Serial.println(" turning left");
    } else if (error < -250) {  // Line is to the right
        turnRight();
        Serial.print("Error:  ");
        Serial.print(error);
        Serial.println(" turning right");
        
    } else {
        moveForward();
    }
}
