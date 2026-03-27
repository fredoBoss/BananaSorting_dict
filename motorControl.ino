// motorControl.ino

// ─── motorRotateFunc ──────────────────────────────────────────────────
// Controls the motor via motorCtrlPin (active LOW).
//   en = 0 → motor ON  (digitalWrite LOW)
//   en = 1 → motor OFF (digitalWrite HIGH)
//
// Used by:
//   - rotateAndSort()        in servo.ino   (new sort-by-switch logic)
//   - rotateNextSwitchTrig() in ultrasonic.ino (camera positioning)
//
// curMotorState tracks last state to avoid redundant writes.
void motorRotateFunc(int en) {
  if (en != curMotorState) {
    if (en == 0) {
      digitalWrite(motorCtrlPin, LOW);  // motor ON
      curMotorState = en;
    } else {
      digitalWrite(motorCtrlPin, HIGH); // motor OFF
      curMotorState = en;
    }
  }
  Serial.println("motor state:" + String(curMotorState));
}
