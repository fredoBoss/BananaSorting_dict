// servo.ino

// ─── Double-click detection globals ──────────────────────────────────
// Track how many times each bin's limit switch has been triggered
int clickCount[6] = {0, 0, 0, 0, 0, 0};  // index 0 = bin 1, etc.  // FIX: start all at 0
unsigned long lastClickTime[6] = {0, 0, 0, 0, 0, 0};

// ─── State machine for edge detection ───────────────────────────────
// Tracks previous switch state to detect RELEASE edges (LOW→HIGH transitions)
// FIX: was used for press detection (HIGH→LOW); now used for release detection (LOW→HIGH)
int prevSwitchState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};  // index 0 = bin 1, etc.

const unsigned long CLICK_TIMEOUT = 8000;    // reset if no next click within 8 sec
const unsigned long DEBOUNCE_TIME = 1500;    // (reserved for future debounce use)

// ─── Required clicks per bin ────────────────────────────────────────
int requiredClicks[6] = {2, 3, 3, 4, 4, 5};
//                       bin1 bin2 bin3 bin4 bin5 bin6

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

// ─── fireServo ────────────────────────────────────────────────────────
// Fires the servo for a given bin number then resets it to neutral.
// Called by rotateAndSort() once the required release count is confirmed.
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

// ─── rotateAndSort (with release-based click detection) ───────────────
//
// Counts a "click" only when the limit switch is RELEASED (LOW → HIGH),
// not when it is pressed. This prevents a held/bouncing switch from
// counting multiple times on the same physical pass.
//
// Logic:
//   - Switch goes LOW  (press)   → just note it in prevSwitchState, do nothing yet
//   - Switch goes HIGH (release) → count as ONE click
//   - When clickCount reaches requiredClicks → stop motor, fire servo
//   - If CLICK_TIMEOUT elapses between releases → reset counter
void rotateAndSort(int targetBin) {
  curMotorState = 1;             // force state reset
  digitalWrite(motorCtrlPin, HIGH);  // ensure motor is OFF before we start

  if (targetBin < 1 || targetBin > 6) {
    Serial.println("Error: invalid bin " + String(targetBin));
    Serial.println("MOTOR_STOP");
    return;
  }

  // Servo switch pins in order — index 0 = bin 1 … index 5 = bin 6
  const int switchPins[6] = {35, 37, 39, 41, 43, 45};

  // ── Use ONE unified index variable throughout ─────────────────────
  // FIX: original code mixed targetIndex and binIndex (both = targetBin-1),
  //      which was redundant and error-prone. Now only targetIndex is used.
  int targetIndex = targetBin - 1;  // 0-based index for all arrays

  Serial.println("trayPos:" + String(targetBin));
  Serial.println("Sorting to bin " + String(targetBin) +
                 " (requires " + String(requiredClicks[targetIndex]) + " releases)");

  // Reset click tracking for this sort run
  clickCount[targetIndex]    = 0;
  lastClickTime[targetIndex] = 0;

  // Also reset the edge-detection state so we don't carry over a stale LOW
  // from a previous run (prevents a phantom first count on start)
  prevSwitchState[targetIndex] = HIGH;

  // Start conveyor
  motorRotateFunc(0);  // 0 = LOW = motor ON (active low)

  boolean isRunning = true;
  unsigned long startTime = millis();
  const unsigned long timeout = 45000;  // 45 s safety timeout

  while (isRunning) {

    // ── Safety timeout ───────────────────────────────────────────────
    if (millis() - startTime > timeout) {
      motorRotateFunc(1);               // motor OFF
      clickCount[targetIndex] = 0;      // reset for next sort
      prevSwitchState[targetIndex] = HIGH;
      Serial.println("TIMEOUT: sort aborted for bin " + String(targetBin));
      Serial.println("MOTOR_STOP");
      return;
    }

    int targetSwitchPin = switchPins[targetIndex];
    int currentState    = digitalRead(targetSwitchPin);
    unsigned long now   = millis();

    // ── STATE MACHINE: detect RELEASE edge (LOW → HIGH) ─────────────
    // FIX: original detected PRESS (HIGH→LOW). We now detect RELEASE
    //      (LOW→HIGH) so the count only increments after the switch
    //      has fully returned to its open/released state.
        if (prevSwitchState[targetIndex] == LOW && currentState == HIGH) {
      // Switch just released — check debounce before counting

      // ── DEBOUNCE GUARD ────────────────────────────────────────────
      // If this release happened too soon after the last valid one,
      // it is switch bounce from the SAME tray pass — ignore it.
      // Real trays passing the switch are always several seconds apart,
      // so DEBOUNCE_TIME (500ms) is a safe minimum gap between valid releases.
      bool tooSoon = (lastClickTime[targetIndex] > 0) &&
                     ((now - lastClickTime[targetIndex]) < DEBOUNCE_TIME);

      if (tooSoon) {
        // Bounce — log it and skip counting
        Serial.println("Bin " + String(targetBin) +
                       " BOUNCE ignored (" +
                       String(now - lastClickTime[targetIndex]) +
                       "ms since last release)");
      } else {
        // ── Valid release ─────────────────────────────────────────

        // Check for timeout between clicks (tray may have passed the bin)
        if (clickCount[targetIndex] > 0 &&
            (now - lastClickTime[targetIndex]) > CLICK_TIMEOUT) {
          Serial.println("Bin " + String(targetBin) +
                         " click timeout — resetting counter");
          clickCount[targetIndex] = 0;
        }

        // Increment and timestamp
        clickCount[targetIndex]++;
        lastClickTime[targetIndex] = now;

        Serial.println("Bin " + String(targetBin) +
                       " RELEASE " + String(clickCount[targetIndex]) +
                       "/" + String(requiredClicks[targetIndex]));

        if (clickCount[targetIndex] < requiredClicks[targetIndex]) {
          // Not enough releases yet — keep waiting
          Serial.println("  → waiting for more...");
        } else {
          // ── Required release count reached ──────────────────────
          Serial.println("  → CONFIRMED! Firing servo for bin " + String(targetBin));

          // alignDelay lets tray coast slightly so it sits perfectly under servo
          delay(alignDelay[targetBin - 1]);
          motorRotateFunc(1);  // motor OFF

          fireServo(targetBin);

          // Clean up state
          clickCount[targetIndex]      = 0;
          prevSwitchState[targetIndex] = HIGH;

          Serial.println("Rotate done");
          Serial.println("MOTOR_STOP");
          isRunning = false;
        }
      }
    }

    // ── Always update previous state ─────────────────────────────────
    // This must stay OUTSIDE the if-block so both press and release
    // transitions are tracked correctly on every loop iteration.
    prevSwitchState[targetIndex] = currentState;
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



// // servo.ino

// // ─── Double-click detection globals ──────────────────────────────────
// // Track how many times each bin's limit switch has been triggered
// int clickCount[6] = {2, 3, 3, 4, 0, 5};  // index 0 = bin 1, etc.
// unsigned long lastClickTime[6] = {0, 0, 0, 0, 0, 0};

// // ─── State machine for edge detection ───────────────────────────────
// // Tracks previous switch state to detect press edges (HIGH→LOW transitions)
// int prevSwitchState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};  // index 0 = bin 1, etc.

// const unsigned long CLICK_TIMEOUT = 8000;    // reset if no 2nd click within 3 sec
// const unsigned long DEBOUNCE_TIME = 3000;     // ignore triggers within 200ms

// // ─── Required clicks per bin ────────────────────────────────────────
// int requiredClicks[6] = {2, 3, 4, 4, 4, 5};
// //                       bin1 bin2 bin3 bin4 bin5 bin6

// void setupServo() {
//   servoY1.attach(5);
//   servoY2.attach(4);
//   servoY3.attach(6);
//   servoY4.attach(8);
//   servoY5.attach(9);
//   servoY6.attach(7);
//   delay(15);
//   stopallServo();
//   delay(15);
// }

// void stopallServo() {
//   servoY1.write(90);
//   servoY2.write(90);
//   servoY3.write(90);
//   servoY4.write(90);
//   servoY5.write(90);
//   servoY6.write(90);
// }

// void initPosServo() {
//   servoY1.write(servoValRot1);
//   delay(rotateDelay);
//   servoY1.write(90);
//   servoY2.write(servoValRot1);
//   delay(rotateDelay);
//   servoY2.write(90);
//   servoY3.write(90);
//   servoY4.write(servoValRot1);
//   delay(rotateDelay);
//   servoY4.write(90);
//   servoY5.write(servoValRot1);
//   delay(rotateDelay);
//   servoY5.write(90);
//   servoY6.write(servoValRot1);
//   delay(rotateDelay);
//   servoY6.write(90);
// }

// // ─── OLD trayPos (kept for reference / rollback) ─────────────────────
// // This used fixed delays which were unreliable due to inconsistent
// // motor speed. Replaced by rotateAndSort() + fireServo() below.
// //
// // void trayPos(int sortResult) {
// //   Serial.println("trayPos:" + String(sortResult));
// //   switch (sortResult) {
// //     case 1:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(8600);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY1.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY1.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY1.write(95);
// //       break;
// //     case 2:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(9300);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY2.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY2.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY2.write(95);
// //       break;
// //     case 3:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(11400);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY3.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY3.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY3.write(95);
// //       break;
// //     case 4:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(14500);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY4.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY4.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY4.write(95);
// //       break;
// //     case 5:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(17900);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY5.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY5.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY5.write(95);
// //       break;
// //     case 6:
// //       digitalWrite(motorCtrlPin, LOW);
// //       delay(20700);
// //       digitalWrite(motorCtrlPin, HIGH);
// //       delay(1000);
// //       servoY6.write(servoValRot2);
// //       delay(rotateDelay);
// //       servoY6.write(servoValRot1);
// //       delay(rotateDelay);
// //       servoY6.write(95);
// //       break;
// //   }
// //   stopallServo();
// //   delay(rotateDelay);
// //   Serial.println("Rotate done");
// //   Serial.println("MOTOR_STOP");
// // }

// // ─── NEW: fireServo ───────────────────────────────────────────────────
// // Fires the servo for a given bin number then resets it to neutral.
// // Called by rotateAndSort() once the matching limit switch is triggered.
// // Keeps same servo movement pattern as original trayPos().
// void fireServo(int bin) {
//   switch (bin) {
//     case 1:
//       servoY1.write(servoValRot2);
//       delay(rotateDelay);
//       servoY1.write(servoValRot1);
//       delay(rotateDelay);
//       servoY1.write(95);
//       break;
//     case 2:
//       servoY2.write(servoValRot2);
//       delay(rotateDelay);
//       servoY2.write(servoValRot1);
//       delay(rotateDelay);
//       servoY2.write(95);
//       break;
//     case 3:
//       servoY3.write(servoValRot2);
//       delay(rotateDelay);
//       servoY3.write(servoValRot1);
//       delay(rotateDelay);
//       servoY3.write(95);
//       break;
//     case 4:
//       servoY4.write(servoValRot2);
//       delay(rotateDelay);
//       servoY4.write(servoValRot1);
//       delay(rotateDelay);
//       servoY4.write(95);
//       break;
//     case 5:
//       servoY5.write(servoValRot2);
//       delay(rotateDelay);
//       servoY5.write(servoValRot1);
//       delay(rotateDelay);
//       servoY5.write(95);
//       break;
//     case 6:
//       servoY6.write(servoValRot2);
//       delay(rotateDelay);
//       servoY6.write(servoValRot1);
//       delay(rotateDelay);
//       servoY6.write(95);
//       break;
//   }
//   stopallServo();
//   delay(rotateDelay);
// }

// // ─── NEW: rotateAndSort (with double-click detection) ─────────────────
// // Requires each limit switch to be triggered TWICE before firing the servo.
// // This ensures the tray has fully settled under the servo.
// //
// // Logic:
// //   - 1st click: increment clickCount, record timestamp
// //   - 2nd click (within CLICK_TIMEOUT): fire servo
// //   - If timeout expires: reset counter and wait for fresh 1st click
// //   - If wrong bin triggers: reset counter for target bin (tray passed)
// void rotateAndSort(int targetBin) {
//    curMotorState = 1;  // Force state reset
//   digitalWrite(motorCtrlPin, HIGH);  // Ensure motor is OFF
  
//   if (targetBin < 1 || targetBin > 6) {
//     Serial.println("Error: invalid bin " + String(targetBin));
//     Serial.println("MOTOR_STOP");
//     return;
//   }

//   // Servo switch pins in order — index 0 = bin 1 … index 5 = bin 6
//   // limitSw1 (23) and limitSw2 (25) are NOT in this array on purpose
//   const int switchPins[6] = {35, 37, 39, 41, 43, 45};

//   Serial.println("trayPos:" + String(targetBin));
//   Serial.println("Sorting to bin " + String(targetBin) + " (requires " + String(requiredClicks[targetBin - 1]) + " clicks)");

//   // Initialize click tracking for this sort
//   int targetIndex = targetBin - 1;  // 0-indexed
//   clickCount[targetIndex] = 0;
//   lastClickTime[targetIndex] = 0;

//   // Start conveyor immediately
//   motorRotateFunc(0);  // 0 = LOW = motor ON (active low)

//   boolean isRunning = true;
//   unsigned long startTime = millis();
//   const unsigned long timeout = 45000; // 45 s safety timeout

//   while (isRunning) {

//     // Safety timeout — stops motor and notifies PC
//     if (millis() - startTime > timeout) {
//       motorRotateFunc(1); // motor OFF
//       clickCount[targetIndex] = 0;  // reset for next sort
//       Serial.println("TIMEOUT: sort aborted for bin " + String(targetBin));
//       Serial.println("MOTOR_STOP");
//       return;
//     }

//     // Poll ONLY the target bin's switch — ignore all other switches and limitSw1/limitSw2
//     int targetSwitchPin = switchPins[targetIndex];
//     int currentState = digitalRead(targetSwitchPin);
//     unsigned long now = millis();
//     int binIndex = targetBin - 1;

//     // ── STATE MACHINE: Detect press edge (HIGH → LOW transition) ────
//     if (prevSwitchState[targetIndex] == HIGH && currentState == LOW) {
//       // Switch just pressed — count this as ONE click

//       // Check if this is a timeout (too long since first click)
//       if (clickCount[binIndex] > 0 && 
//           (now - lastClickTime[binIndex]) > CLICK_TIMEOUT) {
//         Serial.println("Bin " + String(targetBin) + " click timeout, resetting...");
//         clickCount[binIndex] = 0;
//       }

//       // Increment click counter (debounce already handled by edge detection)
//       clickCount[binIndex]++;
//       lastClickTime[binIndex] = now;

//       if (clickCount[binIndex] == 1) {
//         Serial.println("Bin " + String(targetBin) + " CLICK " + String(clickCount[binIndex]) + "/" + String(requiredClicks[binIndex]) + " - waiting for confirmation...");
//       } 
//       else if (clickCount[binIndex] == requiredClicks[binIndex]) {
//         // ── Required clicks confirmed ────────────────────────────────
//         Serial.println("Bin " + String(targetBin) + " CLICK " + String(clickCount[binIndex]) + "/" + String(requiredClicks[binIndex]) + " - CONFIRMED!");
        
//         // alignDelay lets the tray coast a little further so it sits
//         // perfectly under the servo before the motor stops.
//         delay(alignDelay[targetBin - 1]);
//         motorRotateFunc(1); // motor OFF

//         Serial.println("Bin " + String(targetBin) + " reached - firing servo");
//         fireServo(targetBin);
//         clickCount[binIndex] = 0;  // reset for next sort
//         prevSwitchState[targetIndex] = HIGH;  // reset state

//         Serial.println("Rotate done");
//         Serial.println("MOTOR_STOP");
//         isRunning = false;
//       }
//     }

//     // Update previous state for next iteration
//     prevSwitchState[targetIndex] = currentState;
//   }
// }

// // ─── testServo (unchanged) ────────────────────────────────────────────
// void testServo() {
//   delay(rotateDelay);
//   for (pos = 0; pos <= degF; pos += 1) {
//     servoY1.write(pos);
//     delay(15);
//   }
//   delay(rotateDelay);
//   for (pos = degR; pos >= 0; pos -= 1) {
//     servoY1.write(pos);
//     delay(15);
//   }
// }
