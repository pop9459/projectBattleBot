#include <Arduino.h>

#define RED_LED_PIN 4
#define ORANGE_LED_PIN 5
#define GREEN_LED_PIN 6

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ORANGE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(ORANGE_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(3000);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(ORANGE_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(4000);

  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(ORANGE_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH);
  delay(1000);
}