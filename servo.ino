// servo.ino

// ─── Double-click detection globals ──────────────────────────────────
// Track how many times each bin's limit switch has been triggered
int clickCount[6] = {2, 0, 0, 0, 0, 0};  // index 0 = bin 1, etc.
unsigned long lastClickTime[6] = {0, 0, 0, 0, 0, 0};

const unsigned long CLICK_TIMEOUT = 8000;    // reset if no 2nd click within 3 sec
const unsigned long DEBOUNCE_TIME = 3000;     // ignore triggers within 200ms

void setupServo() {
  servoY1.attach(5);
  servoY2.attach(4);
  servoY3.attach(6);
  servoY4.attach(8);
  servoY5.attach(9);
  servoY6.attach(7);
  delay(15);
  stopallServo();
  delay(15);
}

void stopallServo() {
  servoY1.write(90);
  servoY2.write(90);
  servoY3.write(90);
  servoY4.write(90);
  servoY5.write(90);
  servoY6.write(90);
}

void initPosServo() {
  servoY1.write(servoValRot1);
  delay(rotateDelay);
  servoY1.write(90);
  servoY2.write(servoValRot1);
  delay(rotateDelay);
  servoY2.write(90);
  servoY3.write(90);
  servoY4.write(servoValRot1);
  delay(rotateDelay);
  servoY4.write(90);
  servoY5.write(servoValRot1);
  delay(rotateDelay);
  servoY5.write(90);
  servoY6.write(servoValRot1);
  delay(rotateDelay);
  servoY6.write(90);
}

// ─── OLD trayPos (kept for reference / rollback) ─────────────────────
// This used fixed delays which were unreliable due to inconsistent
// motor speed. Replaced by rotateAndSort() + fireServo() below.
//
// void trayPos(int sortResult) {
//   Serial.println("trayPos:" + String(sortResult));
//   switch (sortResult) {
//     case 1:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(8600);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY1.write(servoValRot2);
//       delay(rotateDelay);
//       servoY1.write(servoValRot1);
//       delay(rotateDelay);
//       servoY1.write(95);
//       break;
//     case 2:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(9300);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY2.write(servoValRot2);
//       delay(rotateDelay);
//       servoY2.write(servoValRot1);
//       delay(rotateDelay);
//       servoY2.write(95);
//       break;
//     case 3:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(11400);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY3.write(servoValRot2);
//       delay(rotateDelay);
//       servoY3.write(servoValRot1);
//       delay(rotateDelay);
//       servoY3.write(95);
//       break;
//     case 4:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(14500);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY4.write(servoValRot2);
//       delay(rotateDelay);
//       servoY4.write(servoValRot1);
//       delay(rotateDelay);
//       servoY4.write(95);
//       break;
//     case 5:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(17900);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY5.write(servoValRot2);
//       delay(rotateDelay);
//       servoY5.write(servoValRot1);
//       delay(rotateDelay);
//       servoY5.write(95);
//       break;
//     case 6:
//       digitalWrite(motorCtrlPin, LOW);
//       delay(20700);
//       digitalWrite(motorCtrlPin, HIGH);
//       delay(1000);
//       servoY6.write(servoValRot2);
//       delay(rotateDelay);
//       servoY6.write(servoValRot1);
//       delay(rotateDelay);
//       servoY6.write(95);
//       break;
//   }
//   stopallServo();
//   delay(rotateDelay);
//   Serial.println("Rotate done");
//   Serial.println("MOTOR_STOP");
// }

// ─── NEW: fireServo ───────────────────────────────────────────────────
// Fires the servo for a given bin number then resets it to neutral.
// Called by rotateAndSort() once the matching limit switch is triggered.
// Keeps same servo movement pattern as original trayPos().
void fireServo(int bin) {
  switch (bin) {
    case 1:
      servoY1.write(servoValRot2);
      delay(rotateDelay);
      servoY1.write(servoValRot1);
      delay(rotateDelay);
      servoY1.write(95);
      break;
    case 2:
      servoY2.write(servoValRot2);
      delay(rotateDelay);
      servoY2.write(servoValRot1);
      delay(rotateDelay);
      servoY2.write(95);
      break;
    case 3:
      servoY3.write(servoValRot2);
      delay(rotateDelay);
      servoY3.write(servoValRot1);
      delay(rotateDelay);
      servoY3.write(95);
      break;
    case 4:
      servoY4.write(servoValRot2);
      delay(rotateDelay);
      servoY4.write(servoValRot1);
      delay(rotateDelay);
      servoY4.write(95);
      break;
    case 5:
      servoY5.write(servoValRot2);
      delay(rotateDelay);
      servoY5.write(servoValRot1);
      delay(rotateDelay);
      servoY5.write(95);
      break;
    case 6:
      servoY6.write(servoValRot2);
      delay(rotateDelay);
      servoY6.write(servoValRot1);
      delay(rotateDelay);
      servoY6.write(95);
      break;
  }
  stopallServo();
  delay(rotateDelay);
}

// ─── NEW: rotateAndSort (with double-click detection) ─────────────────
// Requires each limit switch to be triggered TWICE before firing the servo.
// This ensures the tray has fully settled under the servo.
//
// Logic:
//   - 1st click: increment clickCount, record timestamp
//   - 2nd click (within CLICK_TIMEOUT): fire servo
//   - If timeout expires: reset counter and wait for fresh 1st click
//   - If wrong bin triggers: reset counter for target bin (tray passed)
void rotateAndSort(int targetBin) {
  if (targetBin < 1 || targetBin > 6) {
    Serial.println("Error: invalid bin " + String(targetBin));
    Serial.println("MOTOR_STOP");
    return;
  }

  // Servo switch pins in order — index 0 = bin 1 … index 5 = bin 6
  // limitSw1 (23) and limitSw2 (25) are NOT in this array on purpose
  const int switchPins[6] = {35, 37, 39, 41, 43, 45};

  Serial.println("trayPos:" + String(targetBin));
  Serial.println("Sorting to bin " + String(targetBin) + " (requires 2 clicks)");

  // Initialize click tracking for this sort
  int targetIndex = targetBin - 1;  // 0-indexed
  clickCount[targetIndex] = 0;
  lastClickTime[targetIndex] = 0;

  // Start conveyor immediately
  motorRotateFunc(0);  // 0 = LOW = motor ON (active low)

  boolean isRunning = true;
  unsigned long startTime = millis();
  const unsigned long timeout = 45000; // 45 s safety timeout

  while (isRunning) {

    // Safety timeout — stops motor and notifies PC
    if (millis() - startTime > timeout) {
      motorRotateFunc(1); // motor OFF
      clickCount[targetIndex] = 0;  // reset for next sort
      Serial.println("TIMEOUT: sort aborted for bin " + String(targetBin));
      Serial.println("MOTOR_STOP");
      return;
    }

    // Poll only the 6 servo switches — ignore limitSw1/limitSw2
    for (int i = 0; i < 6; i++) {
      if (digitalRead(switchPins[i]) == LOW) {
        int triggeredBin = i + 1;

        if (triggeredBin == targetBin) {
          // ── Correct bin switch triggered ─────────────────────────

          unsigned long now = millis();
          int binIndex = triggeredBin - 1;

          // Check if this is a timeout (too long since first click)
          if (clickCount[binIndex] > 0 && 
              (now - lastClickTime[binIndex]) > CLICK_TIMEOUT) {
            Serial.println("Bin " + String(targetBin) + " click timeout, resetting...");
            clickCount[binIndex] = 0;
          }

          // Check if this is debounce (too soon after last click)
          if (clickCount[binIndex] > 0 && 
              (now - lastClickTime[binIndex]) < DEBOUNCE_TIME) {
            Serial.println("Debounce: ignoring rapid re-trigger");
            break;  // ignore this trigger, it's probably a bounce
          }

          // Valid click — increment counter
          clickCount[binIndex]++;
          lastClickTime[binIndex] = now;

          if (clickCount[binIndex] == 1) {
            Serial.println("Bin " + String(targetBin) + " CLICK 1/2 - waiting for confirmation...");
          } 
          else if (clickCount[binIndex] == 2) {
            // ── Second click confirmed ──────────────────────────────
            Serial.println("Bin " + String(targetBin) + " CLICK 2/2 - CONFIRMED!");
            
            // alignDelay lets the tray coast a little further so it sits
            // perfectly under the servo before the motor stops.
            delay(alignDelay[targetBin - 1]);
            motorRotateFunc(1); // motor OFF

            Serial.println("Bin " + String(targetBin) + " reached - firing servo");
            fireServo(targetBin);
            clickCount[binIndex] = 0;  // reset for next sort

            Serial.println("Rotate done");
            Serial.println("MOTOR_STOP");
            isRunning = false;
          }
        } 
        else {
          // ── Wrong bin triggered ──────────────────────────────────
          // Tray passed the target bin, so reset the click counter
          Serial.println("Wrong bin " + String(triggeredBin) + " triggered (target was " 
                         + String(targetBin) + ") - tray passed, resetting...");
          clickCount[targetIndex] = 0;
          lastClickTime[targetIndex] = 0;
        }
        
        break; // only process one switch per polling cycle
      }
    }
  }
}

// ─── testServo (unchanged) ────────────────────────────────────────────
void testServo() {
  delay(rotateDelay);
  for (pos = 0; pos <= degF; pos += 1) {
    servoY1.write(pos);
    delay(15);
  }
  delay(rotateDelay);
  for (pos = degR; pos >= 0; pos -= 1) {
    servoY1.write(pos);
    delay(15);
  }
}
