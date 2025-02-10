#include <Arduino.h>
#include <Servo.h>

float getDistance();

#define US_TRIG_PIN 12
#define US_ECHO_PIN 13

#define GRIPPER_PIN 11

Servo gripper;

void setup() {
  Serial.begin(9600);
  
  gripper.attach(GRIPPER_PIN);

  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

  gripper.write(45);
}

void loop() {
  if (getDistance() < 3)
  {
    gripper.write(45);
  }
  else
  {
    gripper.write(90);
  }
  
  delay(1000);
}

float getDistance()
{
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  float duration = pulseIn(US_ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  Serial.print(distance);
  return distance;
}