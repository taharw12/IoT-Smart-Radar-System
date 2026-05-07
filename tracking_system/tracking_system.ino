// Camera Tracking + Radar + Buzzer
// Hardware: Arduino UNO R4 Minima

// ---- PIN DEFINITIONS ----
#define PAN_PIN    9
#define TILT_PIN   10
#define LASER_PIN  7
#define BUZZER_PIN 6
#define TRIG_PIN   4
#define ECHO_PIN   5

// ---- SERVO CONFIG ----
#define PULSE_MIN_US  544
#define PULSE_MAX_US  2400
#define PULSE_PERIOD  20000

#define PAN_MIN   0
#define PAN_MAX   180
#define TILT_MIN  10
#define TILT_MAX  170

#define SERIAL_BAUD 115200

// ---- STATE ----
int targetPan  = 90;
int targetTilt = 90;

unsigned long lastPulseTime = 0;
unsigned long lastRadarTime = 0;

String serialBuffer = "";

// ---- SERVO PULSE ----
int angleToPulse(int angle) {
    return map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
}

void sendServoPulse(int pin, int angle) {
    int us = angleToPulse(constrain(angle, 0, 180));
    digitalWrite(pin, HIGH);
    delayMicroseconds(us);
    digitalWrite(pin, LOW);
}

void updateServos() {
    unsigned long now = micros();
    if (now - lastPulseTime >= (unsigned long)PULSE_PERIOD) {
        lastPulseTime = now;
        sendServoPulse(PAN_PIN,  targetPan);
        sendServoPulse(TILT_PIN, targetTilt);
    }
}

// ---- LASER & BUZZER ----
void setLaser(bool on) {
    digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

void setBuzzer(bool on) {
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

// ---- RADAR HC-SR04 ----
void readRadar() {
    if (millis() - lastRadarTime >= 100) {
        lastRadarTime = millis();

        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        long duration = pulseIn(ECHO_PIN, HIGH, 18000);
        if (duration > 0) {
            int distance = (int)(duration * 0.034 / 2);
            Serial.print("RADAR:");
            Serial.print(targetPan);
            Serial.print(",");
            Serial.println(distance);
        }
    }
}

// ---- SERIAL PARSER ----
void parseCommand(const String& cmd) {

    if (cmd.startsWith("X:")) {
        int commaIdx = cmd.indexOf(",Y:");
        if (commaIdx == -1) return;
        int pan  = constrain(cmd.substring(2, commaIdx).toInt(), PAN_MIN,  PAN_MAX);
        int tilt = constrain(cmd.substring(commaIdx + 3).toInt(), TILT_MIN, TILT_MAX);
        targetPan  = pan;
        targetTilt = tilt;
        Serial.print("ACK:X:"); Serial.print(pan);
        Serial.print(",Y:");    Serial.println(tilt);
        return;
    }

    if (cmd == "LASER:ON")   { setLaser(true);   Serial.println("ACK:LASER:ON");   return; }
    if (cmd == "LASER:OFF")  { setLaser(false);  Serial.println("ACK:LASER:OFF");  return; }
    if (cmd == "BUZZER:ON")  { setBuzzer(true);  Serial.println("ACK:BUZZER:ON");  return; }
    if (cmd == "BUZZER:OFF") { setBuzzer(false); Serial.println("ACK:BUZZER:OFF"); return; }

    if (cmd == "CENTER") {
        targetPan  = 90;
        targetTilt = 90;
        Serial.println("ACK:CENTER");
        return;
    }
}

void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) parseCommand(serialBuffer);
            serialBuffer = "";
        } else if (c != '\r' && serialBuffer.length() < 32) {
            serialBuffer += c;
        }
    }
}

// ---- SETUP ----
void setup() {
    Serial.begin(SERIAL_BAUD);

    pinMode(PAN_PIN,    OUTPUT);
    pinMode(TILT_PIN,   OUTPUT);
    pinMode(LASER_PIN,  OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TRIG_PIN,   OUTPUT);
    pinMode(ECHO_PIN,   INPUT);

    digitalWrite(PAN_PIN,  LOW);
    digitalWrite(TILT_PIN, LOW);
    setLaser(false);
    setBuzzer(false);

    Serial.println("TEST:START");

    for (int i = 0; i < 40; i++) { sendServoPulse(PAN_PIN, 90);  sendServoPulse(TILT_PIN, 90); delay(20); }
    for (int i = 0; i < 35; i++) { sendServoPulse(PAN_PIN, 45);  sendServoPulse(TILT_PIN, 90); delay(20); }
    for (int i = 0; i < 35; i++) { sendServoPulse(PAN_PIN, 135); sendServoPulse(TILT_PIN, 90); delay(20); }
    for (int i = 0; i < 30; i++) { sendServoPulse(PAN_PIN, 90);  sendServoPulse(TILT_PIN, 90); delay(20); }

    targetPan  = 90;
    targetTilt = 90;

    Serial.println("TEST:DONE");
    setLaser(true);
    lastPulseTime = micros();
    Serial.println("READY");
}
// ---- LOOP ----
void loop() {
    readSerial();
    updateServos();
    readRadar();
}
