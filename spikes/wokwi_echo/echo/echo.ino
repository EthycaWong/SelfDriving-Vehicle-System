// Link test: echoes each serial line back and sends a heartbeat once a second.
void setup() {
  Serial.begin(115200);
  Serial.println(F("READY"));
}

unsigned long lastBeat = 0;
unsigned long beats = 0;

void loop() {
  while (Serial.available()) {
    Serial.write(Serial.read());
  }
  if (millis() - lastBeat >= 1000) {
    lastBeat = millis();
    Serial.print(F("BEAT "));
    Serial.println(beats++);
  }
}
