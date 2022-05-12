#define boardLed 4

void setup() {
  pinMode(boardLed,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(boardLed, 1-digitalRead(boardLed));
  Serial.println("Hola Mundo");
  delay(1000);
}
