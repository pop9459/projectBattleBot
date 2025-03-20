int ledRed = 4;
int ledYellow = 5;
int ledGreen = 6;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ledRed, LOW);
  delay(3000);
  digitalWrite(ledRed, HIGH);
  delay(100);
  digitalWrite(ledGreen, LOW);
  delay(4000);
  digitalWrite(ledGreen, HIGH);
  delay(100);
  digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledYellow, HIGH);
  delay(100);
}
