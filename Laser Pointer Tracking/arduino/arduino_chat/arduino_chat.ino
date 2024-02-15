void setup() {
  Serial.begin(115200); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) {
    float speed = Serial.parseFloat(); // Read speed from serial
    Serial.print("Received speed: ");
    Serial.println(speed);
  }
}
