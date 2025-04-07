#include <Adafruit_NeoPixel.h>

#define SERVO           11  // servo pin is on pin 11
#define GRIPPER_OPEN   1900 // PULSE LENGTH IN THE OPEN STATE 
#define GRIPPER_CLOSE  1100// PULSE LENGTH IN THE CLOSE STATE 
//#define OBSTACLE_THRESHOLD 10;

#define NeoLed 7
#define NUM_PIXELS 4

 //Motot pins
const int MOTOR_A_1 = 5; //Left Forward
const int MOTOR_A_2 = 6; //Left Backward 
const int MOTOR_B_1 = 9; //Right backward
const int MOTOR_B_2 = 10; //Right forward


// Ultrasonic Sensor Pins
const int trigPin = 13;
const int echoPin = 12;

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLed, NEO_GRB + NEO_KHZ800);
void setup() {
  // put your setup code here, to run once:
    pinMode (MOTOR_A_1, OUTPUT);
  pinMode (MOTOR_A_2, OUTPUT);
  pinMode (MOTOR_B_1, OUTPUT);
  pinMode (MOTOR_B_2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600);
  pinMode(SERVO, OUTPUT);
  digitalWrite(SERVO, LOW);
//  pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);

strip.begin();
  setLEDColor(0,0,0);
  strip.show();
}

void loop() {
  // put your main code here, to run repeatedly:
  gripper_open();
  delay(100);

  moveForward();
  delay(1000);

  stopMotors();
  delay(500);

  gripper_close();
  delay(100);

  turnRight();
  delay(1000);
  
  moveForward();
  delay(1500);

  gripper_open();

  moveBackward();
  delay(1000);

  stopMotors();
  delay(5000);

}


void gripper(int pulse){
  static unsigned long timer;
  static int lastPulse;
  if (millis() > timer){
    if (pulse > 0) {
      lastPulse = pulse;
    } else {
      pulse = lastPulse;
    }
    digitalWrite(SERVO, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO, LOW);
    timer = millis() + 20;    //time interval

    // Grab with servo

 }
}


void gripper_open(){
  unsigned long startTime = millis();
  while (millis() - startTime < 250) { // Perform OPEN action for 300 milliseconds
    gripper(GRIPPER_OPEN); // Typical pulse for "OPEN" action
  }
 }

void gripper_close(){
  unsigned long startTime = millis();
  while (millis() - startTime < 250) { // Perform CLOSE action for 300 milliseconds
    gripper(GRIPPER_CLOSE); // Typical pulse for "CLOSE" action
  }
 }

// void gripper(int pulse) {
//   for (int i = 0; i < 10; i++) {  // Send 50 pulses
//     digitalWrite(SERVO, HIGH);
//     delayMicroseconds(pulse);
//     digitalWrite(SERVO, LOW);
//     delay(20); // Pause between pulses
//   }
// }


void setLEDColor(int g, int r, int b){
  for(int i = 0; i <NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(g, r, b));
  }
  strip.show();
}

// Move forward
void moveForward() {
  digitalWrite(MOTOR_A_1, 240);
  digitalWrite(MOTOR_A_2, 0);
  digitalWrite(MOTOR_B_1, 0);
  digitalWrite(MOTOR_B_2, 240);
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
  setLEDColor(0, 0, 250); // blue 
}

// Stop motors
void stopMotors() {
  digitalWrite(MOTOR_A_1, LOW);
  digitalWrite(MOTOR_A_2, LOW);
  digitalWrite(MOTOR_B_1, LOW);
  digitalWrite(MOTOR_B_2, LOW);
  setLEDColor(0, 255, 0); // Red
}


