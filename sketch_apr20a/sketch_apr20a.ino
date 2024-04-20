void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32C6 Dev board");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int n = Serial.parseInt();

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      Serial.println(n);

    }
  }
}
