#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char x = Serial.read();  // Read one character
    if (x == '0') {          // If the character is '0', turn off the LED
      digitalWrite(LED_PIN, LOW);
    } 
    else if (x == '1') {     // If the character is '1', turn on the LED
      digitalWrite(LED_PIN, HIGH);
    }
  }
}

