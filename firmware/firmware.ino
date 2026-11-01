// ESP32-S2 firmware for the SelfDrive demo. Uses the wiring from the Wokwi circuit in diagram.json.
//
// Sends the four ultrasonic distances to the simulation and receives its commands. The circuit has
// no outputs, so the board reports what it would do through the serial monitor (lines starting #).

const int sensorHR1T = 0;  // Front (ultrasonic4)
const int sensorHR2T = 1;  // Rear (ultrasonic1)
const int sensorHR3T = 2;  // Left (ultrasonic2)
const int sensorHR4T = 3;  // Right (ultrasonic3)

const int sensorHR1E = 18;
const int sensorHR2E = 19;
const int sensorHR3E = 20;
const int sensorHR4E = 21;

const unsigned long ECHO_TIMEOUT_US = 25000;
const int NO_ECHO_CM = 400;
const unsigned long WATCHDOG_MS = 500;
const unsigned long NOTE_PERIOD_MS = 1000;

const char* const STATE_NAMES[] = {"CRUISE", "SLOW", "STOP", "ESTOP", "FAILSAFE"};
const char* const MANEUVER_NAMES[] = {"LANE KEEP", "LANE CHANGE", "OVERTAKE", "GO AROUND", "SWERVE",
                                      "TURN", "U-TURN", "REVERSE", "PULL OVER"};
const char* const CLASS_NAMES[] = {"person", "animal", "vehicle", "traffic_control", "tree", "building", "obstruction"};

int distances[4] = {NO_ECHO_CM, NO_ECHO_CM, NO_ECHO_CM, NO_ECHO_CM};
char line[80];
size_t lineLength = 0;

unsigned long lastCommandMs = 0;
unsigned long lastNoteMs = 0;
bool everCommanded = false;
bool watchdogTripped = false;

int steerDegrees = 0, throttlePct = 0, brakePct = 0, driveState = 0, maneuver = 0;
unsigned long classMask = 0;
int nearestClass = -1, nearestDm = 0;

byte checksumOf(const char* text, size_t length) {
  byte sum = 0;
  for (size_t i = 0; i < length; i++) sum ^= (byte)text[i];
  return sum;
}

void sendFramed(const char* body) {
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", checksumOf(body, strlen(body)));
  Serial.print('$');
  Serial.print(body);
  Serial.println(tail);
}

// One sensor at a time, because firing all four together lets them hear each other's echoes.
int readDistance(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  const long totaltime = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (totaltime == 0) return NO_ECHO_CM;
  return (int)(totaltime * 0.034 / 2);
}

void handleBody(char* body) {
  if (strncmp(body, "CMD,", 4) == 0) {
    unsigned long seq = 0;
    if (sscanf(body + 4, "%lu,%d,%d,%d,%d,%d", &seq, &steerDegrees, &throttlePct, &brakePct, &driveState, &maneuver) != 6) return;
    lastCommandMs = millis();
    everCommanded = true;
    if (watchdogTripped) {
      watchdogTripped = false;
      Serial.println(F("# Link back: commands resumed"));
    }
    char ack[20];
    snprintf(ack, sizeof(ack), "ACK,%lu", seq);
    sendFramed(ack);
  } else if (strncmp(body, "DET,", 4) == 0) {
    unsigned long seq = 0;
    sscanf(body + 4, "%lu,%lu,%d,%d", &seq, &classMask, &nearestClass, &nearestDm);
  }
}

void handleLine() {
  line[lineLength] = '\0';
  const char* star = strrchr(line, '*');
  if (line[0] != '$' || star == nullptr) return;
  const size_t bodyLength = star - line - 1;
  if (strtol(star + 1, nullptr, 16) != checksumOf(line + 1, bodyLength)) {
    Serial.println(F("# Dropped a damaged line"));
    return;
  }
  line[bodyLength + 1] = '\0';
  handleBody(line + 1);
}

void readSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n') {
      handleLine();
      lineLength = 0;
    } else if (c != '\r' && lineLength < sizeof(line) - 1) {
      line[lineLength++] = c;
    }
  }
}

void printNote() {
  if (!everCommanded) {
    Serial.println(F("# Waiting for the simulation..."));
    return;
  }
  char note[110];
  const bool validState = driveState >= 0 && driveState < 5;
  const bool validManeuver = maneuver >= 0 && maneuver < 9;
  snprintf(note, sizeof(note), "# %s | %s | steer %+d deg | throttle %d%% | brake %d%%",
           validState ? STATE_NAMES[driveState] : "?", validManeuver ? MANEUVER_NAMES[maneuver] : "?",
           steerDegrees, throttlePct, brakePct);
  Serial.println(note);
  if (nearestClass >= 0 && nearestClass < 7) {
    snprintf(note, sizeof(note), "#   nearest: %s at %d.%d m", CLASS_NAMES[nearestClass], nearestDm / 10, nearestDm % 10);
    Serial.println(note);
  }
  snprintf(note, sizeof(note), "#   sensors cm: front %d, rear %d, left %d, right %d", distances[0], distances[1], distances[2], distances[3]);
  Serial.println(note);
}

void setup() {
  Serial.begin(115200);

  pinMode(sensorHR1T, OUTPUT);
  pinMode(sensorHR2T, OUTPUT);
  pinMode(sensorHR3T, OUTPUT);
  pinMode(sensorHR4T, OUTPUT);
  pinMode(sensorHR1E, INPUT);
  pinMode(sensorHR2E, INPUT);
  pinMode(sensorHR3E, INPUT);
  pinMode(sensorHR4E, INPUT);

  Serial.println(F("# SelfDrive board ready"));
}

void loop() {
  distances[0] = readDistance(sensorHR1T, sensorHR1E);
  readSerial();
  distances[1] = readDistance(sensorHR2T, sensorHR2E);
  readSerial();
  distances[2] = readDistance(sensorHR3T, sensorHR3E);
  readSerial();
  distances[3] = readDistance(sensorHR4T, sensorHR4E);
  readSerial();

  // With no commands, the board treats the link as lost and reports that it's braking.
  if (everCommanded && !watchdogTripped && millis() - lastCommandMs > WATCHDOG_MS) {
    watchdogTripped = true;
    throttlePct = 0;
    brakePct = 100;
    Serial.println(F("# WATCHDOG: no commands for 500 ms, braking"));
  }

  char sensors[48];
  snprintf(sensors, sizeof(sensors), "SEN,%d,%d,%d,%d,%d", distances[0], distances[1], distances[2], distances[3], watchdogTripped ? 1 : 0);
  sendFramed(sensors);

  if (millis() - lastNoteMs >= NOTE_PERIOD_MS) {
    lastNoteMs = millis();
    printNote();
  }
}
