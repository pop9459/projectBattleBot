const int MOTOR_A_1 = 5; //Left Forward
const int MOTOR_A_2 = 6; //Left Backward 
const int MOTOR_B_1 = 9; //Right backward
const int MOTOR_B_2 = 10; //Right forward

// Ultrasonic Sensor Pins
const int trigPin = 13;
const int echoPin = 12;


void setup() {
  // put your setup code here, to run once:
  pinMode (MOTOR_A_1, OUTPUT);
  pinMode (MOTOR_A_2, OUTPUT);
  pinMode (MOTOR_B_1, OUTPUT);
  pinMode (MOTOR_B_2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);

}


unsigned long lastGoAroundTime = 0;  // Stores the last time goAround() was executed
const unsigned long goAroundCooldown = 3000; // Cooldown time in milliseconds (3 seconds)

void loop() {
    int distance = getDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    unsigned long currentTime = millis();  // Get the current time

    if (distance > 0 && distance <= 17) {  // Object detected within 20 cm
        if (currentTime - lastGoAroundTime > goAroundCooldown) {  // Check cooldown
        Serial.println("Object detected! Executing goAround()");
            lastGoAroundTime = millis();  // Update BEFORE calling goAround()
            stopMotors();
            delay(100);
            goAround();

        }
    } else {
        goForward();
    }

    delay(20);
}


// Function to measure distance with ultrasonic sensor
int getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;  // Convert time to distance (cm)
    return distance;
}

// Function to move forward
void goForward() {
    analogWrite(MOTOR_A_1, 235);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 245);
}

// Function to stop motors
void stopMotors() {
    analogWrite(MOTOR_A_1, 0);
    analogWrite(MOTOR_A_2, 0);
    analogWrite(MOTOR_B_1, 0);
    analogWrite(MOTOR_B_2, 0);
}

void turnAround () {
      analogWrite(MOTOR_A_1, 0);
      analogWrite(MOTOR_A_2, 0);      //speed decrease to turn left (reversed)
      analogWrite(MOTOR_B_1, 255);
      analogWrite(MOTOR_B_2, 0);
}

void goLeft () {               //move forward and turns left - both motors moving
      analogWrite(MOTOR_A_1, 0);      //speed decreased to turn left
      analogWrite(MOTOR_A_2, 0);
      analogWrite(MOTOR_B_1, 0);      //speed increased to turn left
      analogWrite(MOTOR_B_2, 255);
  }

  void goRight () {              //move forward and turn right - both motors moving
      analogWrite(MOTOR_A_1, 230);      //speed increased to turn right
      analogWrite(MOTOR_A_2, 0);
      analogWrite(MOTOR_B_1, 0);      //speed decreased to turn right
      analogWrite(MOTOR_B_2, 0);
  };

  void goBackward () {                  //move back for both motors
      analogWrite(MOTOR_A_1, 0);
      analogWrite(MOTOR_A_2, 255);      //reverse 
      analogWrite(MOTOR_B_1, 255);
      analogWrite(MOTOR_B_2, 0);
  }

  void goBackwardLeft () {              //move back and turn left - both motors moving in reverse
      analogWrite(MOTOR_A_1, 0);
      analogWrite(MOTOR_A_2, 130);      //speed decrease to turn left (reversed)
      analogWrite(MOTOR_B_1, 255);
      analogWrite(MOTOR_B_2, 0);      //speed increased to turn left (reversed)
  }

  void goBackwardRight () {             //move back and turn right - both motors moving in reverse
      analogWrite(MOTOR_A_1, 0);
      analogWrite(MOTOR_A_2, 255);      //speed increased to turn right (reversed)

      analogWrite(MOTOR_B_1, 130);
      analogWrite(MOTOR_B_2, 0);      //speed decreased to turn right (reversed)
  }

void goAround() {
    goRight();
    delay(700);
    goForward();
    delay(800);
    goLeft();
    delay(700);
    goForward();
}

