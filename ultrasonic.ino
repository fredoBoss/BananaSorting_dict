// ultrasonic.ino

int inches = 0;
int cm1    = 0;
int cm2    = 0;

int limitSwState1 = 1;
int limitSwState2 = 1;

bool rotateOnly = true; // set to true so conveyor rotates even if weight
                        // sensor already settled after sort

// ─── Ultrasonic ──────────────────────────────────────────────────────
long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

// ─── Rotation helpers (unchanged) ────────────────────────────────────
void startRotate() {
  runRotate(true);
}

void rotateNext1() {
  int a = 0.01723 * readUltrasonicDistance(ut2, ue2);
  delay(50);
  int b = 0.01723 * readUltrasonicDistance(ut1, ue1);
  delay(50);

  digitalWrite(motorCtrlPin, LOW);
  delay(300);
  if (a <= 5 && b <= 5) {
    Serial.println("att");
    delay(800);
    digitalWrite(motorCtrlPin, HIGH);
    scale.tare(20);
    currentPlate = (currentPlate % 5) + 1;
    delay(4000);
    digitalWrite(motorCtrlPin, LOW);
  }
  Serial.println("start rotate");
  boolean isRunning = true;
  while (isRunning) {
    Serial.println("rotating");
    int cm11 = 0.01723 * readUltrasonicDistance(ut1, ue1);
    delay(50);
    int cm21 = 0.01723 * readUltrasonicDistance(ut2, ue2);
    delay(50);
    if (cm21 <= 3) {
      if (cm11 <= 3) {
        digitalWrite(motorCtrlPin, HIGH);
        isRunning = false;
        scale.tare(20);
        currentPlate = (currentPlate % 5) + 1;
      } else {
        digitalWrite(motorCtrlPin, LOW);
      }
    } else {
      digitalWrite(motorCtrlPin, LOW);
    }
  }
  Serial.println("end while");
}

void rotateNext() {
  Serial.println("start rotate");
  digitalWrite(motorCtrlPin, LOW);
  delay(700);
  digitalWrite(motorCtrlPin, HIGH);
  delay(3000);
  digitalWrite(motorCtrlPin, LOW);
  boolean isRunning = true;
  int cm11 = 10;
  int cm21 = 10;
  while (isRunning) {
    Serial.println("rotating");
    cm21 = 0.01723 * readUltrasonicDistance(ut2, ue2);
    delay(50);
    if (cm21 <= 10) {
      digitalWrite(motorCtrlPin, HIGH);
      cm11 = 0.01723 * readUltrasonicDistance(ut1, ue1);
      delay(50);
      digitalWrite(motorCtrlPin, LOW);
      delay(200);
      if (cm11 <= 10) {
        digitalWrite(motorCtrlPin, HIGH);
        isRunning = false;
        scale.tare(20);
        currentPlate = (currentPlate % 5) + 1;
      } else {
        digitalWrite(motorCtrlPin, LOW);
      }
    } else {
      digitalWrite(motorCtrlPin, LOW);
    }
  }
  Serial.println("end while");
}

void translateDistance() {
  cm1 = 0.01723 * readUltrasonicDistance(ut1, ue1);
  delay(100);
  cm2 = 0.01723 * readUltrasonicDistance(ut2, ue2);
  if (cm2 >= 25) {
    Serial.println("stop");
    Serial.println("near camera");
    digitalWrite(motorCtrlPin, HIGH);
  } else {
    Serial.println("run");
    digitalWrite(motorCtrlPin, LOW);
  }
  delay(100);
}

void runRotate(boolean en) {
  if (en != motorLastState) {
    if (en) {
      Serial.println("run");
      digitalWrite(motorCtrlPin, LOW);
    } else {
      Serial.println("stop");
      digitalWrite(motorCtrlPin, HIGH);
    }
    motorLastState = en;
  }
}

// ─── Limit switch init ────────────────────────────────────────────────
// limitSw1 (23) and limitSw2 (25) = camera/rotation positioning (unchanged)
// limitSw3–8 (35–45)              = NEW servo position switches
void initLimitSwitch() {
  // existing camera/rotation switches
  pinMode(limitSw1, INPUT_PULLUP);
  pinMode(limitSw2, INPUT_PULLUP);

  // new servo position switches
  pinMode(limitSw3, INPUT_PULLUP);
  pinMode(limitSw4, INPUT_PULLUP);
  pinMode(limitSw5, INPUT_PULLUP);
  pinMode(limitSw6, INPUT_PULLUP);
  pinMode(limitSw7, INPUT_PULLUP);
  pinMode(limitSw8, INPUT_PULLUP);
}

// ─── Limit switch test ────────────────────────────────────────────────
// Updated to also report the 6 new servo switches.
// Send "checkLimitSw:" from PC to run this.
void testLimitSwitch() {
  Serial.println("Limit Switch 1 (cam): "  + String(digitalRead(limitSw1)));
  Serial.println("Limit Switch 2 (cam): "  + String(digitalRead(limitSw2)));
  Serial.println("Limit Switch 3 (bin1): " + String(digitalRead(limitSw3)));
  Serial.println("Limit Switch 4 (bin2): " + String(digitalRead(limitSw4)));
  Serial.println("Limit Switch 5 (bin3): " + String(digitalRead(limitSw5)));
  Serial.println("Limit Switch 6 (bin4): " + String(digitalRead(limitSw6)));
  Serial.println("Limit Switch 7 (bin5): " + String(digitalRead(limitSw7)));
  Serial.println("Limit Switch 8 (bin6): " + String(digitalRead(limitSw8)));
}

// ─── rotateNextSwitchTrig (unchanged) ────────────────────────────────
// This handles moving tray to camera position. Uses limitSw1 only.
// Completely separate from rotateAndSort() which uses limitSw3–8.
void rotateNextSwitchTrig() {
  if (testWeight) {
    return;
  }
  limitSwState1 = digitalRead(limitSw1);
  limitSwState2 = digitalRead(limitSw2);
  Serial.println("start rotate");
  Serial.println("DEBUG: Initial Limit Switch States - SW1: " + String(limitSwState1) + " SW2: " + String(limitSwState2));

  if (rotateOnly) {
    digitalWrite(motorCtrlPin, LOW);
    Serial.println("DEBUG: Motor started (rotateOnly mode)");
    delay(1000);
    digitalWrite(motorCtrlPin, HIGH);
    Serial.println("DEBUG: Motor stopped after delay");
  } else {
    if (limitSwState1 == LOW) {
      Serial.println("DEBUG: Limit Switch 1 already pressed at start!");
      Serial.println("pass rotation");
      Serial.println("MOTOR_STOP");
      return;
    }
  }
  delay(2000);

  limitSwState1 = 1;
  limitSwState2 = 1;
  digitalWrite(motorCtrlPin, LOW);
  Serial.println("DEBUG: Motor started - entering rotation loop");
  boolean isRunning = true;

  unsigned long startTime = millis();
  unsigned long timeout   = 30000; // 30 s safety timeout

  while (isRunning) {
    if (millis() - startTime > timeout) {
      digitalWrite(motorCtrlPin, HIGH);
      Serial.println("DEBUG: TIMEOUT REACHED!");
      Serial.println("MOTOR_STOP");
      isRunning = false;
      break;
    }

    limitSwState1 = digitalRead(limitSw1);
    limitSwState2 = digitalRead(limitSw2);

    if (limitSwState1 == LOW) {
      delay(600);
      digitalWrite(motorCtrlPin, HIGH);

      Serial.println("DEBUG: Plate at camera position");
      Serial.println("DEBUG: Pausing for camera capture...");
      delay(4000); // camera capture pause
      Serial.println("DEBUG: Camera pause complete");

      Serial.println("MOTOR_STOP");
      isRunning = false;
    }
  }
  Serial.println("end while");
}
