int ledRed = 4;
int ledYellow = 5;
int ledGreen = 6;
int buttonGreen = 8;   // Start button

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(buttonGreen, INPUT_PULLUP);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledYellow, HIGH);
}

void loop() {
  
  int buttonPlay = digitalRead(buttonGreen); // Read Green Button (Play)

  // Run the traffic light sequence when isRunning is true
  if (buttonPlay  == LOW ) {
    digitalWrite(ledRed, HIGH);   // Red OFF

    digitalWrite(ledGreen, LOW);  // Green ON
    delay(4000);                  // Wait 4 seconds
    digitalWrite(ledGreen, HIGH); // Green OFF
    delay(100);

    digitalWrite(ledYellow, LOW); // Yellow ON
    delay(1000);                  // Wait 1 second
    digitalWrite(ledYellow, HIGH);// Yellow OFF
    delay(100);
  } else {
    digitalWrite(ledRed, LOW);    // Red ON
  }
}

