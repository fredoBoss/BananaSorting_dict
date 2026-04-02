#include <HX711.h>
#include <Servo.h>
#include <EEPROM.h>

// ─── Pin Definitions ────────────────────────────────────────────────
#define loadcellDout 2
#define loadcellSck  3
#define motorCtrlPin 33

#define ue1 11
#define ut1 12
#define ut2 13
#define ue2 46

// Camera / rotation positioning switches (unchanged)
#define limitSw1 23
#define limitSw2 25

// NEW — 6 servo position switches (one beside each servo/bin)
#define limitSw3 35   // bin 1 → servoY1
#define limitSw4 37   // bin 2 → servoY2
#define limitSw5 39   // bin 3 → servoY3
#define limitSw6 41   // bin 4 → servoY4
#define limitSw7 43   // bin 5 → servoY5
#define limitSw8 45   // bin 6 → servoY6

// ─── Load Cell / Calibration ────────────────────────────────────────
float balancePt[5] = {0, 0, 0, 0, 0};
int currentPlate = 2; // Default to plate 1 (1–5)
bool testWeight = false;

float ZERO_OFFSET = 0.0;
float SCALE_FACTOR = 1.0;
const float BALANCE_PT_OFFSET[5] = {862.63, 862.63, 862.63, 862.63, 862.63};

// ─── Servos ─────────────────────────────────────────────────────────
Servo servoY1;
Servo servoY2;
Servo servoY3;
Servo servoY4;
Servo servoY5;
Servo servoY6;

// ─── HX711 ──────────────────────────────────────────────────────────
HX711 scale;

// ─── Globals ────────────────────────────────────────────────────────
String readStr       = "";
bool   calibrated    = false;
int    curMotorState = 0;

int pos          = 0;
int degF         = 90;
int degR         = 90;
int rotateDelay  = 1500;
int servoValRot2 = 180;
int servoValRot1 = 0;
int servoValStop = 90;
boolean motorLastState = 0;

bool isBusy = false;

// ─── NEW: sort assignment + per-bin alignment delay ─────────────────
int pendingSortResult = 0;

// Tune these per bin (ms) — increase if tray stops short of servo,
// decrease if tray overshoots. Start at 300 and test bin by bin.
int alignDelay[6] = {400, 100, 300, 100, 300, 300};
//                  bin1  bin2  bin3  bin4  bin5  bin6

// ─── Setup ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  setupServo();
  pinMode(motorCtrlPin, OUTPUT);
  digitalWrite(motorCtrlPin, HIGH); // Active low — motor OFF at start
  initLimitSwitch();
}

// ─── Loop ───────────────────────────────────────────────────────────
void loop() {
  mainLoop();
}

// ─── Main command handler ────────────────────────────────────────────
void mainLoop() {
  while (Serial.available()) {
    readStr = Serial.readStringUntil('\n');
    readStr.trim();

    // ── Read weight ──────────────────────────────────────────────────
    if (readStr.indexOf("readWt:") >= 0) {
      if (!calibrated) {
        Serial.println("Error: Run 'calibrate:<1-5>' for plate " + String(currentPlate) + " first!");
      } else {
        float rawWeight = weightValAvg();
        Serial.println("readWt:" + String(rawWeight - balancePt[currentPlate - 1]));
      }
    }

    // ── Tray position / sort ─────────────────────────────────────────
    // CHANGED: now calls rotateAndSort() instead of old trayPos()
    // rotateAndSort() starts the conveyor immediately and waits for the
    // matching servo limit switch to fire — no fixed delays for motor position.
    else if (readStr.indexOf("trayPos:") >= 0) {
      if (isBusy) {
        Serial.println("BUSY: trayPos ignored");
        return;
      }
      isBusy = true;
      String trayPosVal = readStr.substring(8);
      pendingSortResult = trayPosVal.toInt();
      rotateAndSort(pendingSortResult);   // ← NEW function (servo.ino)
      isBusy = false;
    }

    // ── Tare ─────────────────────────────────────────────────────────
    else if (readStr.indexOf("tare:") >= 0) {
      String plateStr = readStr.substring(5);
      int plate = plateStr.toInt();
      if (plate < 1 || plate > 5) {
        Serial.println("Error: Plate must be 1-5");
        return;
      }
      digitalWrite(motorCtrlPin, LOW);
      delay(400);
      scale.tare(25);
      currentPlate = plate;
      delay(100);
      digitalWrite(motorCtrlPin, HIGH);
      Serial.println("Tare done for plate " + String(plate));
    }

    else if (readStr.indexOf("tare1:") >= 0) {
      scale.tare(20);
      delay(1000);
      Serial.println("done tare");
    }

    // ── Calibrate ────────────────────────────────────────────────────
    else if (readStr.indexOf("calibrate:") >= 0) {
      String plateStr = readStr.substring(10);
      int plate = plateStr.toInt();
      if (plate < 1 || plate > 5) {
        Serial.println("Error: Plate must be 1-5");
        return;
      }
      calibrateWeight1(plate);
      Serial.println("Calibration complete for plate " + String(plate));
    }

    // ── Set rotate state ─────────────────────────────────────────────
    else if (readStr.indexOf("setRotateState:") >= 0) {
      String plateStateStr = readStr.substring(15);
      int plateState = plateStateStr.toInt();
      if (plateState == 0) {
        Serial.println("disable motor rotation");
        testWeight = true;
        return;
      } else if (plateState == 1) {
        Serial.println("enable motor rotation");
        testWeight = false;
        return;
      } else {
        Serial.println("(motor rotation) invalid value");
        return;
      }
    }

    // ── Next tray to camera ──────────────────────────────────────────
    else if (readStr.indexOf("next:") >= 0) {
      rotateNextSwitchTrig();
    }

    else if (readStr.indexOf("next1:") >= 0) {
      rotateNext1();
    }

    // ── Diagnostics ──────────────────────────────────────────────────
    else if (readStr.indexOf("checkLimitSw:") >= 0) {
      testLimitSwitch();
    }

    else if (readStr.indexOf("printCal:") >= 0) {
      printCalibration();
    }

    else if (readStr.indexOf("clearEEPROM:") >= 0) {
      clearEEPROM();
    }

    else if (readStr.indexOf("clearBalancePt:") >= 0) {
      int colonIndex = readStr.indexOf(":");
      if (colonIndex == -1 || colonIndex >= readStr.length() - 1) {
        Serial.println("Error: Invalid clearBalancePt command format");
        return;
      }
      String plateStr = readStr.substring(colonIndex + 1);
      Serial.println("Parsed plateStr: '" + plateStr + "'");
      int plate = plateStr.toInt();
      if (plate < 1 || plate > 5) {
        Serial.println("Error: Plate must be 1-5");
        return;
      }
      clearBalancePt(plate);
      Serial.println("Cleared balancePt for plate " + String(plate));
    }

    else return;
  }
  readStr = "";
}
