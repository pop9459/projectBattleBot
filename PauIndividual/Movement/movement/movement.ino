// Motor Pins
int motorA_PWM = 5; // Motor A control pin (PWM for speed)
int motorA_DIR = 6; // Motor A control pin (Direction)

int motorB_PWM = 9; // Motor B control pin (PWM for speed)
int motorB_DIR = 10; // Motor B control pin (Direction)

// Encoder Pins
int encoderA = 2; // Encoder pin for motor A
int encoderB = 3; // Encoder pin for motor B

volatile int countA = 0; // Pulse count for motor A
volatile int countB = 0; // Pulse count for motor B

// Desired speed (setpoint) - Adjust as needed
int desiredSpeedA = 50; // Target speed for motor A (in terms of encoder pulses or a unit like RPM)
int desiredSpeedB = 50; // Target speed for motor B

// Speed control variables
int currentSpeedA = 0;   // Current speed of motor A
int currentSpeedB = 0;   // Current speed of motor B
int motorAPWM = 0;       // PWM value for motor A
int motorBPWM = 0;       // PWM value for motor B

// Timer for loop execution
unsigned long previousMillis = 0;
const long interval = 100; // Speed measurement interval (milliseconds)

void setup() {
  // Configure motor pins as OUTPUT
  pinMode(motorA_PWM, OUTPUT);
  pinMode(motorA_DIR, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(motorB_DIR, OUTPUT);

  // Configure encoder pins as INPUT
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderA), countPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), countPulseB, RISING);

  // Initialize serial monitor (optional, for debugging)
  Serial.begin(9600);
}

void loop() {
  // Update speed calculations periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calculate current speeds (pulses per interval)
    currentSpeedA = countA;
    currentSpeedB = countB;

    // Reset pulse counters
    countA = 0;
    countB = 0;

    // Adjust motor speeds using a proportional controller
    motorAPWM = calculatePWM(desiredSpeedA, currentSpeedA, motorAPWM);
    motorBPWM = calculatePWM(desiredSpeedB, currentSpeedB, motorBPWM);

    // Apply speed and direction for Motor A
    if (motorAPWM > 0) { // Forward
      digitalWrite(motorA_DIR, LOW); // Direction
      analogWrite(motorA_PWM, motorAPWM); // Speed (PWM)
    } else { // Backward
      digitalWrite(motorA_DIR, HIGH); // Reverse direction
      analogWrite(motorA_PWM, -motorAPWM); // Convert to positive PWM value
    }

    // Apply speed and direction for Motor B
    if (motorBPWM > 0) { // Forward
      digitalWrite(motorB_DIR, LOW); // Direction
      analogWrite(motorB_PWM, motorBPWM); // Speed (PWM)
    } else { // Backward
      digitalWrite(motorB_DIR, HIGH); // Reverse direction
      analogWrite(motorB_PWM, -motorBPWM); // Convert to positive PWM value
    }

    // Optional: Print speed info to Serial Monitor for debugging
    Serial.print("Motor A Current Speed: ");
    Serial.println(currentSpeedA);
    Serial.print("Motor B Current Speed: ");
    Serial.println(currentSpeedB);
  }
}

// Interrupt service routine for encoder A
void countPulseA() {
  countA++;
}

// Interrupt service routine for encoder B
void countPulseB() {
  countB++;
}

// Function to calculate the new PWM value
int calculatePWM(int desiredSpeed, int currentSpeed, int currentPWM) {
  int error = desiredSpeed - currentSpeed;       // Calculate the error
  int kP = 2;                                   // Proportional gain (adjust as needed)
  int adjustment = kP * error;                  // Proportional control
  int newPWM = currentPWM + adjustment;         // Calculate the new PWM value
  
  // Constrain PWM to valid range for analogWrite (0 - 255)
  newPWM = constrain(newPWM, -255, 255);

  return newPWM; // Return the adjusted PWM value
}
