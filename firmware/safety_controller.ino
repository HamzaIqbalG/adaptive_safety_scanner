const int ledPin = 13; 
unsigned long lastSignalTime = 0;
const long timeout = 1000; // 1 second timeout

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Communication speed

  // Start ON (Safe) to show we have power
  digitalWrite(ledPin, HIGH); 
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'S') { // SAFE -> LED ON
      digitalWrite(ledPin, HIGH);
      lastSignalTime = millis();
    } 
    else if (command == 'C') { // CRITICAL -> LED OFF
      digitalWrite(ledPin, LOW);
      lastSignalTime = millis();
    }
  }

  // Watchdog: If laptop crashes/disconnects, kill the light
  if (millis() - lastSignalTime > timeout) {
    digitalWrite(ledPin, LOW);
  }
}