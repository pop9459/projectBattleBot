// Include the Adafruit NeoPixel library to control  LEDs
#include <Adafruit_NeoPixel.h>

// -------------------- NeoPixel LED Setup --------------------
#define NeoLed 7               // Pin connected to the NeoPixel strip
#define NUM_PIXELS 4           // Number of NeoPixels in the strip

// -------------------- Servo Setup --------------------
#define SERVO 11               // Servo control pin
#define GRIPPER_OPEN 1900      // Pulse width to open the gripper
#define GRIPPER_CLOSE 1100     // Pulse width to close the gripper

// -------------------- Line Sensors --------------------
#define NUM_SENSORS 8                          // Number of line sensors
int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog pins for sensors
int sensorValues[NUM_SENSORS];                // To store sensor readings
int sensorThreshold[NUM_SENSORS];             // Calibration values per sensor

// -------------------- Motor Pins --------------------
const int MOTOR_A_1 = 5;     // Left motor forward
const int MOTOR_A_2 = 6;     // Left motor reverse
const int MOTOR_B_1 = 10;    // Right motor forward
const int MOTOR_B_2 = 9;     // Right motor reverse
const int baseSpeed = 170;   // Base speed for both motors

// -------------------- Ultrasonic Sensor (HC-SR04) --------------------
const int TRIG_PIN = 13;     // Trigger pin for ultrasonic sensor
const int ECHO_PIN = 12;     // Echo pin for ultrasonic sensor
const int FLAG_THRESHOLD = 25;        // Distance (cm) to detect flag
const int OBSTACLE_THRESHOLD = 10;    // Distance (cm) to detect obstacles

// -------------------- PID Controller Settings --------------------
float Kp = 0.10;     // Proportional constant
float Ki = 0.0005;   // Integral constant
float Kd = 0.05;     // Derivative constant
int error = 0, lastError = 0; // PID error values
float integral = 0, derivative = 0;
int correction;       // Value to adjust motor speed

// -------------------- State Flags --------------------
bool startDone = false;      // Whether the robot has started
bool flagDone = false;       // Whether the flag sequence has finished
bool flagDetected = false;   // If the flag was seen at least once

// -------------------- Other Variables --------------------
Adafruit_NeoPixel strip(NUM_PIXELS, NeoLed, NEO_GRB + NEO_KHZ800);
unsigned long lastGoAroundTime = 0;            // Track last time goAround() ran
const unsigned long goAroundCooldown = 3000;   // Cooldown for goAround in ms
int distance;                                  // Distance from ultrasonic sensor

// -------------------- Setup Function --------------------
void setup() {
    Serial.begin(9600); // Start serial monitor

    // Set motor pins as outputs
    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_B_1, OUTPUT);
    pinMode(MOTOR_B_2, OUTPUT);

    // Set ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Set servo pin
    pinMode(SERVO, OUTPUT);
    stopMotors();         // Stop motors at start
    calibrateSensors();   // Read base values from line sensors

    digitalWrite(SERVO, LOW);  // Servo idle

    strip.begin();             // Initialize NeoPixel strip
    setLEDColor(0, 0, 0);      // Turn off LEDs
    strip.show();
}

// -------------------- Main Loop --------------------
void loop() {
    distance = getDistance();  // Measure distance from sensor
    Serial.print("Distance: ");
    Serial.println(distance);

    // Check if flag is close (robot should wait)
    if (distance > 0 && distance <= FLAG_THRESHOLD && !startDone) {
        gripper(GRIPPER_OPEN); // Open gripper
        stopMotors();          // Pause movement
        Serial.println("Flag detected! Waiting...");
        flagDetected = true;
    }

    // If flag has been detected and moved away, begin movement
    if (flagDetected && distance > FLAG_THRESHOLD && !startDone) {
        start();          // Perform starting actions
        startDone = true; // Mark that robot has started
        flagDone = true;
    }

    // Begin main behavior after start
    if (startDone && flagDone) {
        // If obstacle is close, avoid it
        if (distance > 0 && distance <= OBSTACLE_THRESHOLD) {
            stopMotors();
            delay(200);
            Serial.println("Object Detected! Avoiding now!");
            goAround();  // Obstacle avoidance routine
        } else {
            // Normal operation: read sensors and follow line
            readSensors();
            int position = calculateLinePosition();
            Serial.print("Line Position: ");
            Serial.println(position);
            followLine(position);
        }
    }

    delay(15);  // Small delay for sensor stability
}

// -------------------- Distance Measuring Function --------------------
int getDistance() {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH); // Time echo received
    return duration * 0.034 / 2;             // Convert time to cm
}

// -------------------- LED Control Function --------------------
void setLEDColor(int g, int r, int b) {
    for (int i = 0; i < NUM_PIXELS; i++) {
        strip.setPixelColor(i, strip.Color(g, r, b)); // Set color for each LED
    }
    strip.show(); // Update the LEDs
}

// -------------------- Sensor Calibration --------------------
void calibrateSensors() {
    Serial.println("Calibrating sensors...");
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorThreshold[i] = analogRead(sensorPins[i]); // Store baseline readings
    }
    Serial.println("Calibration complete");
}

// -------------------- Start Behavior --------------------
void start() {
    moveForward(240, 240); // Move forward to approach object
    delay(1150);

    stopMotors();
    delay(250);

    gripper(GRIPPER_CLOSE); // Close gripper to grab item
    delay(100);

    goLeft();               // Turn left to rejoin path
    delay(770);

    stopMotors();
    delay(200);
}

// -------------------- Sensor Reading --------------------
void readSensors() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]); // Read line sensor
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }
    Serial.println();
}

// -------------------- Line Position Calculation --------------------
int calculateLinePosition() {
    long weightedSum = 0, sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = sensorValues[i];
        weightedSum += (long)value * i * 1000;
        sum += value;
    }

    // If no clear line, assume it's the finish line
    if (sum < 3200 || sum > 8000) return -1;
    return weightedSum / sum;
}

// -------------------- Line Following with PID --------------------
void followLine(int position) {
    if (position == -1) {
        Serial.println("Finish line reached...");
        finishingLine(); // Drop object and stop
        return;
    }

    int center = (NUM_SENSORS - 1) * 1000 / 2; // Midpoint of sensors
    error = position - center;
    integral += error;
    derivative = error - lastError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Adjust motor speeds based on PID
    int _leftSpeed = constrain(baseSpeed - correction, 0, 255);
    int _rightSpeed = constrain(baseSpeed + correction, 0, 255);
    integral = constrain(integral, -10000, 10000);

    moveForward(_leftSpeed, _rightSpeed);
    lastError = error;
}

// -------------------- Gripper Pulse Control --------------------
void gripper(int pulse) {
    static unsigned long timer;
    static int lastPulse;

    if (millis() > timer) {
        if (pulse > 0) lastPulse = pulse;
        else pulse = lastPulse;

        digitalWrite(SERVO, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(SERVO, LOW);
        timer = millis() + 20;
    }
}

// -------------------- Movement Functions --------------------
void moveForward(int _leftSpeed, int _rightSpeed) {
    analogWrite(MOTOR_A_1, _leftSpeed);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, _rightSpeed);
    analogWrite(MOTOR_B_2, 0);
    setLEDColor(255, 0, 0); // Red for forward
}

void moveBackward() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, baseSpeed);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, baseSpeed);
    setLEDColor(50, 255, 0); // Orange for reverse
}

void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
    setLEDColor(0, 255, 0); // Green for stop
}

void goLeft() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 80);
    analogWrite(MOTOR_B_1, 245);
    analogWrite(MOTOR_B_2, 0);
    setLEDColor(0, 240, 200); // Cyan for left
}

void goRight() {
    analogWrite(MOTOR_A_1, 245);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 80);
    analogWrite(MOTOR_B_2, 0);
    setLEDColor(0, 0, 250); // Blue for right
}

// -------------------- Obstacle Avoidance --------------------
void goAround() {
    goLeft();                // Step 1: Turn to avoid
    delay(650);
    moveForward(190, 190);   // Step 2: Move forward
    delay(700);
    goRight();               // Step 3: Rejoin the path
    delay(1000);
    moveForward(190, 190);   // Step 4: Continue forward
}

// -------------------- Final Behavior at Finish Line --------------------
void finishingLine() {
    stopMotors();
    delay(100);
    gripper(GRIPPER_OPEN);  // Drop object
    delay(10);
    moveBackward();         // Move back from drop point
    delay(1000);
    stopEverything();       // Stop completely
}

void stopEverything() {
    stopMotors();           // Stop all motion
    setLEDColor(0, 0, 0);   // Turn off LEDs
    Serial.println("System Halted!");
    while (true);           // Freeze program
}
