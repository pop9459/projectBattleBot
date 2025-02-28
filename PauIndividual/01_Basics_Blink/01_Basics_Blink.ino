int buttonGreen = 8;   // Start button
int buttonBlue = 9;    // Stop button
bool isRunning = false;

void setup() {
  // put your setup code here, to run once:
pinMode(buttonGreen, INPUT_PULLUP);
pinMode(buttonBlue, INPUT_PULLUP);
pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int buttonPlay = digitalRead(buttonGreen); // Read Green Button (Play)
  int buttonStop = digitalRead(buttonBlue);  // Read Blue Button (Stop)
  if (buttonPlay == LOW && !isRunning) {
    isRunning = true;  // Start the traffic light sequence
  }

 // If the Stop Button is pressed
  if (buttonStop == LOW && isRunning) {
    isRunning = false;  // Stop the traffic light sequence
    
  }
  // put your main code here, to run repeatedly:
  if(isRunning) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}
