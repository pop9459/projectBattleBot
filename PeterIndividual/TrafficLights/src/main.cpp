#include <Arduino.h>

void runLights();

#define RED_LED_PIN 4
#define ORANGE_LED_PIN 5
#define GREEN_LED_PIN 6

#define BUTTON_PIN 8

void setup() {
  Serial.begin(9600);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ORANGE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  pinMode(BUTTON_PIN, INPUT);
}

void loop() {
  // TURN OFF ALL THE LIGHTS
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(ORANGE_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
  
  while(digitalRead(BUTTON_PIN) == HIGH) {
    // DO NOTHING WHILE THE BUTTON IS NOT PRESSED
  }
  Serial.println("Button pressed!");

  // RUN THE LIGHT SEQUENCE
  runLights();
}

// bla bla bla 2
void runLights()
{
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
