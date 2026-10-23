#include <ArduinoBLE.h>

// ============================================================
// DHRUVBRAIN — JETSON ORIN NANO + UNO R4 CABLE INTERFACE
// Serial USB (Primary Control) + BLE Telemetry & Mode Arbitrator
//
// 4-Wheel Mecanum H-Drive + 12V LED Mood Controller
//
// ARCHITECTURE OVERVIEW:
// 1. JETSON ORIN NANO (Cable Link):
//    - Connected via USB-A to USB-C cable.
//    - Sends commands over hardware USB Serial (115200 baud).
//    - Arduino immediately acknowledges receipt by replying "ACK\n".
//
// 2. BLE TELEMETRY (ArduinoBLE):
//    - Advertises Service:        19b10000-e8f2-537e-4f6c-d104768a1214
//    - Command / Log Char:       19b10001-e8f2-537e-4f6c-d104768a1214
//    - Device Local Name:        "Dhruv_Arduino"
//    - Any command executed or received from Jetson is mirrored
//      over BLE notifications to a phone app or remote dashboard!
//
// 3. AUTONOMOUS VS MANUAL ARBITRATOR (is_autonomous):
//    - is_autonomous = true (DEFAULT):
//        * ONLY Jetson cable commands are permitted to move motors.
//        * Dashboard BLE movement commands are blocked (safety lock).
//    - is_autonomous = false (MANUAL MODE):
//        * Dashboard BLE commands are permitted to move motors.
//        * Jetson cable commands are ignored (except STOP).
//    - Emergency STOP (<STOP>, STOP, HALT, Q, X) is ALWAYS honored
//      from ANY source regardless of mode!
//    - Mode toggle commands: "MODE:AUTO", "MODE:MANUAL", "AUTO", "MANUAL".
//
// ============================================================

// ============================================================
// BLE DEFINITIONS (Matching hal_jetson.py / DhruvBrain)
// ============================================================

BLEService dhruvService("19b10000-e8f2-537e-4f6c-d104768a1214");

BLEStringCharacteristic commandChar(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,
  128
);

bool bleCentralConnected = false;

// ============================================================
// ARBITRATION STATE
// ============================================================

bool is_autonomous = false; // Default: Manual / UI Controller active (Jetson turns it on via MODE:AUTO)

// ============================================================
// MOTOR PINS (IBT-2 / BTS7960 Drivers on UNO R4)
// ============================================================

#define M1_RPWM 2
#define M1_LPWM 3

#define M2_RPWM 4
#define M2_LPWM 5

#define M3_RPWM 6
#define M3_LPWM 7

#define M4_RPWM 8
#define M4_LPWM 9

// ============================================================
// MOTOR SETTINGS & CALIBRATION
// ============================================================

const int SPEED_LIMIT = 180;
const bool DEBUG = true;

// Motor direction inversion flags
// Layout: M1=Front-Left, M2=Back-Left, M3=Back-Right, M4=Front-Right
// Confirmed working with physical tests: M1=F, M2=F, M3=T, M4=T
bool M1_INVERT = false;
bool M2_INVERT = false;
bool M3_INVERT = true;
bool M4_INVERT = true;


bool testModeActive = false;

// ============================================================
// TIMED MOTOR EXECUTION & DURATION SAFEGUARDS
// ============================================================

unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
bool motorTimedActive = false;

// Sustained durations for step commands received from Jetson (avoids jerking/stalling)
unsigned long defaultTranslationDurationMs = 3000; // Default sustained translation: 3000ms (3.0s)
unsigned long defaultPivotDurationMs       = 1800; // Default sustained pivot: 1800ms (1.8s)
unsigned long minTimedDurationMs           = 2500; // Threshold: timed translations < 2500ms will be updated to default
bool durationOverrideActive                = false; // Raw pass-through by default; sustained mode enabled only when requested

// Continuous mode safety watchdog: if Jetson sends duration 0 and dies/hangs, auto-stop after 1500ms
unsigned long lastSerialCmdTime            = 0;
const unsigned long SERIAL_COMMS_TIMEOUT_MS = 1500;
bool continuousActive                      = false;

// ============================================================
// STATUS LED / MOSFET (PIN 13) — JETSON MOOD CONTROLLER
// ============================================================

const int LED_PIN = 13; // MOSFET triggering 12V Blue Light

enum LedMood
{
  MOOD_STARTUP,
  MOOD_IDLE_WHITE,       // Solid ON (Passive waiting)
  MOOD_CURIOSITY_GREEN,  // Slow pulsing fade (Analyzing)
  MOOD_FLIRT_PINK,       // Double blink pattern (Approach/Witty)
  MOOD_ALERT_RED,        // Fast aggressive strobe (Obstacle/Alert)
  MOOD_THINKING_BLUE,    // Rapid random flickering (LLM inference)
  MOOD_SPEAKING,         // Fast 200ms speech rhythm blink
  MOOD_OFF               // Solid OFF
};

LedMood currentLedMood = MOOD_STARTUP;

const int blinkInterval = 50;
const unsigned long startupBlinkDuration = 2000;

unsigned long ledPreviousMillis = 0;
unsigned long ledPhaseStartMillis = 0;
int ledBrightness = 0;
bool ledState = LOW;
bool ledFadeDirection = true;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void logOut(String msg);
void bleBroadcast(String msg);
void handleIncomingSerial(String line);
void handleIncomingBle(String line);
bool handleModeSwitch(String line, String source);
bool handleDurationConfig(String line, String source);
void parseAndExecuteCommand(String line, String source);

void parseMotorPart(String str, String source);
void parseLedPart(String str);
void parseColonMotor(String str, String source);
void parseColonLed(String str);
void executeMotorAction(String dir, int speed, unsigned long durationMs, String source);
void setLedMood(LedMood mood);
void setLedMoodByName(String moodName);

void handleLegacyCommand(String command, String source);
void checkMotorTimer();
void updateStatusLED();

void bleBroadcast(String msg);
void mixDrive(int Vx, int Vy, int W);
void moveForward(int speed = SPEED_LIMIT);
void moveBackward(int speed = SPEED_LIMIT);
void strafeLeft(int speed = SPEED_LIMIT);
void strafeRight(int speed = SPEED_LIMIT);
void moveForwardRight(int speed = SPEED_LIMIT);
void moveForwardLeft(int speed = SPEED_LIMIT);
void moveBackwardRight(int speed = SPEED_LIMIT);
void moveBackwardLeft(int speed = SPEED_LIMIT);
void rotateCW(int speed = SPEED_LIMIT);
void rotateCCW(int speed = SPEED_LIMIT);
void stopAll();

void setMotor(int motor, int speed);
void setRawMotor(int motor, int speed);
void runMotorTokens(String line);
bool parseMotorToken(String token, int &motor, int &speed, bool &reverse);

// ============================================================
// SETUP
// ============================================================

// Non-blocking serial line accumulator
String _serialBuf = "";

void setup()
{
  // 1. Initialize USB CDC Serial (Primary Jetson Link)
  Serial.begin(115200);
  // Do NOT use setTimeout — non-blocking char reads only

  // 2. Initialize Motor PWM Pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);
  pinMode(M4_RPWM, OUTPUT);
  pinMode(M4_LPWM, OUTPUT);

  stopAll();

  // 3. Initialize Status MOSFET / LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledPhaseStartMillis = millis();

  // 4. Initialize BLE (for Wireless Monitoring & Dashboard Override)
  if (!BLE.begin())
  {
    Serial.println("WARN: BLE initialization failed! Running in USB-only mode.");
  }
  else
  {
    BLE.setLocalName("Dhruv_Arduino");
    BLE.setAdvertisedService(dhruvService);

    dhruvService.addCharacteristic(commandChar);
    BLE.addService(dhruvService);

    BLE.advertise();
    Serial.println("SYS: BLE Peripheral Advertising as 'Dhruv_Arduino'...");
  }

  Serial.println("==================================================");
  Serial.println("   DHRUVBRAIN — JETSON CABLE + BLE ARBITRATOR");
  Serial.println("   Mode: MANUAL (BLE/UI controller active by default)");
  Serial.println("   Duration Safeguard: OFF (raw pass-through)");
  Serial.println("   Listening on Serial (115200 baud) for Jetson/Nav2...");
  Serial.println("==================================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  updateStatusLED();
  checkMotorTimer();

  // ----------------------------------------------------------
  // 1. RECEIVE FROM JETSON / Nav2 (USB SERIAL) — Non-blocking char accumulator
  // ----------------------------------------------------------
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\n')
    {
      _serialBuf.trim();
      if (_serialBuf.length() > 0)
      {
        handleIncomingSerial(_serialBuf);
      }
      _serialBuf = "";
    }
    else if (c != '\r')
    {
      if (_serialBuf.length() < 128)
      {
        _serialBuf += c;
      }
      else
      {
        _serialBuf = ""; // Reset on buffer overflow
      }
    }
  }

  // ----------------------------------------------------------
  // 2. RECEIVE FROM DASHBOARD / MOBILE (BLUETOOTH LE)
  // ----------------------------------------------------------
  BLE.poll(); // Non-blocking BLE event pump — must be called every loop

  BLEDevice central = BLE.central();

  if (central)
  {
    if (!bleCentralConnected)
    {
      bleCentralConnected = true;
      String connMsg = "BLE: Connected (" + central.address() + ")";
      Serial.println(connMsg);
      bleBroadcast(connMsg);
    }

    if (central.connected())
    {
      // Check for incoming BLE commands from phone/dashboard
      if (commandChar.written())
      {
        String bleVal = commandChar.value();
        bleVal.trim();
        if (bleVal.length() > 0)
        {
          handleIncomingBle(bleVal);
        }
      }
    }
    else
    {
      // BLE Disconnected
      bleCentralConnected = false;
      Serial.println("BLE: Disconnected from central.");
    }
  }
}

// ============================================================
// INCOMING SERIAL HANDLER (JETSON CABLE)
// ============================================================

void handleIncomingSerial(String line)
{
  lastSerialCmdTime = millis();

  bool isMixCmd = line.startsWith("<MIX,") || line.startsWith("<mix,") ||
                  line.startsWith("MIX,") || line.startsWith("mix,");

  // 1. Send ACK for configuration / non-streaming commands so Jetson handshake works.
  //    Suppress ACK for high-frequency streaming MIX commands to prevent USB CDC buffer overflow!
  if (!isMixCmd)
  {
    Serial.println("ACK");
  }

  // 2. Check if this is a mode switch command
  if (handleModeSwitch(line, "CABLE"))
  {
    return;
  }

  // 3. Mirror the received command over BLE only for non-streaming commands (rate-limited for MIX)
  if (!isMixCmd)
  {
    logOut("RX[CABLE]: " + line);
  }

  // 4. Always execute — USB cable commands from Jetson/Nav2 are always trusted.
  //    No arbitration blocking here. BLE and cable can coexist safely.
  parseAndExecuteCommand(line, "CABLE");
}

// ============================================================
// INCOMING BLE HANDLER (DASHBOARD / MOBILE)
// ============================================================

void handleIncomingBle(String line)
{
  lastSerialCmdTime = millis(); // Keep watchdog alive during continuous manual streaming

  // 1. Check if this is a mode switch command (Always allowed over BLE)
  if (handleModeSwitch(line, "BLE"))
  {
    return;
  }

  // 2. Log received BLE message to Serial and BLE Terminal
  logOut("RX[BLE]: " + line);

  // 3. Parse and execute the dashboard command (UI manual control always responsive)
  parseAndExecuteCommand(line, "BLE");
}

// ============================================================
// MODE SWITCH HANDLER (is_autonomous Arbitrator)
// ============================================================

bool handleModeSwitch(String line, String source)
{
  String clean = line;
  clean.toUpperCase();
  clean.trim();

  // Mode: Manual (Dashboard control allowed)
  if (clean == "MODE:MANUAL" || clean == "MANUAL" || clean == "AUTO_OFF" || clean == "MODE_MANUAL")
  {
    is_autonomous = false;
    stopAll();
    motorTimedActive = false;
    motorDuration = 0;
    // Instant FIFO purge: flush any pending serial bytes so zero buffer lag occurs!
    while (Serial.available()) { Serial.read(); }
    logOut("MODE: Switched to MANUAL. Dashboard BLE control ACTIVE. Jetson locked.");
    return true;
  }

  // Mode: Autonomous (Jetson cable control allowed)
  if (clean == "MODE:AUTO" || clean == "AUTO" || clean == "AUTO_ON" || clean == "MODE_AUTO" || clean == "AUTONOMOUS")
  {
    is_autonomous = true;
    stopAll();
    motorTimedActive = false;
    motorDuration = 0;
    // Instant FIFO purge: flush any pending serial bytes so zero buffer lag occurs!
    while (Serial.available()) { Serial.read(); }
    logOut("MODE: Switched to AUTONOMOUS. Jetson Cable control ACTIVE. Dashboard locked.");
    return true;
  }

  // Query mode status
  if (clean == "MODE?" || clean == "STATUS?")
  {
    String status = String("STATUS: Mode=") + (is_autonomous ? "AUTONOMOUS" : "MANUAL") + " | Motors=" + (motorTimedActive ? "RUNNING" : "STOPPED");
    logOut(status);
    return true;
  }

  return false;
}

// ============================================================
// DURATION CONFIGURATION HANDLER
// ============================================================

bool handleDurationConfig(String line, String source)
{
  String clean = line;
  clean.trim();
  clean.toUpperCase();

  if (clean == "DURATION?" || clean == "GET_DURATION" || clean == "DURATION:STATUS")
  {
    String status = "CFG DURATION: Translation=" + String(defaultTranslationDurationMs) +
                    "ms | Pivot=" + String(defaultPivotDurationMs) +
                    "ms | MinThreshold=" + String(minTimedDurationMs) +
                    "ms | Override=" + (durationOverrideActive ? "ACTIVE" : "OFF (RAW)");
    logOut(status);
    return true;
  }

  if (clean == "DURATION:RAW" || clean == "DURATION:OFF" || clean == "DURATION:PASSTHROUGH")
  {
    durationOverrideActive = false;
    logOut("CFG DURATION: Duration override DISABLED (Raw pass-through active).");
    return true;
  }

  if (clean == "DURATION:ON" || clean == "DURATION:ENABLE")
  {
    durationOverrideActive = true;
    logOut("CFG DURATION: Duration override ENABLED (Sustained motion active).");
    return true;
  }

  if (clean.startsWith("DURATION:") || clean.startsWith("SET_DURATION:"))
  {
    int colonIdx = clean.indexOf(':');
    String valStr = clean.substring(colonIdx + 1);
    valStr.trim();

    if (valStr.startsWith("PIVOT:"))
    {
      unsigned long val = valStr.substring(6).toInt();
      if (val > 0)
      {
        defaultPivotDurationMs = val;
        logOut("CFG DURATION: Set pivot duration to " + String(defaultPivotDurationMs) + "ms");
        return true;
      }
    }
    else if (valStr.startsWith("MIN:"))
    {
      unsigned long val = valStr.substring(4).toInt();
      if (val > 0)
      {
        minTimedDurationMs = val;
        logOut("CFG DURATION: Set min threshold to " + String(minTimedDurationMs) + "ms");
        return true;
      }
    }
    else
    {
      unsigned long val = valStr.toInt();
      if (val > 0)
      {
        defaultTranslationDurationMs = val;
        if (minTimedDurationMs > val) minTimedDurationMs = val;
        durationOverrideActive = true;
        logOut("CFG DURATION: Set default translation duration to " + String(defaultTranslationDurationMs) + "ms");
        return true;
      }
    }
  }

  return false;
}

// ============================================================
// COMMAND PARSER DISPATCHER
// ============================================================

void parseAndExecuteCommand(String line, String source)
{
  testModeActive = false;

  // ----------------------------------------------------------
  // Duration Configuration Commands (e.g. DURATION:3500, DURATION?)
  // ----------------------------------------------------------
  if (handleDurationConfig(line, source))
  {
    return;
  }

  // ----------------------------------------------------------
  // Motor Calibration Token (T1_150, t2_120)
  // ----------------------------------------------------------
  char first = line.charAt(0);
  if ((first == 'T' || first == 't') && line.indexOf('_') != -1 && line.indexOf('<') == -1)
  {
    testModeActive = true;
    runMotorTokens(line);
    return;
  }

  // ----------------------------------------------------------
  // 1. COMBINED PIPE FORMAT: "<MOTOR_CMD>|<LED_CMD>"
  // ----------------------------------------------------------
  int pipeIndex = line.indexOf('|');
  if (pipeIndex != -1)
  {
    String motorPart = line.substring(0, pipeIndex);
    String ledPart = line.substring(pipeIndex + 1);

    motorPart.trim();
    ledPart.trim();

    if (motorPart.length() > 0)
      parseMotorPart(motorPart, source);

    if (ledPart.length() > 0)
      parseLedPart(ledPart);

    return;
  }

  // ----------------------------------------------------------
  // 2. COLON FORMAT ("MOTOR:..." or "LED:...")
  // ----------------------------------------------------------
  if (line.startsWith("MOTOR:") || line.startsWith("motor:"))
  {
    parseColonMotor(line, source);
    return;
  }

  if (line.startsWith("LED:") || line.startsWith("led:"))
  {
    parseColonLed(line);
    return;
  }

  // ----------------------------------------------------------
  // 3. STANDALONE BRACKETED COMMAND ("<...>")
  // ----------------------------------------------------------
  if (line.startsWith("<") && line.endsWith(">"))
  {
    String inner = line.substring(1, line.length() - 1);
    inner.trim();

    if (inner.startsWith("LED,") || inner.startsWith("led,") || inner.startsWith("LED:") || inner.startsWith("led:"))
    {
      parseLedPart(inner);
    }
    else
    {
      parseMotorPart(line, source);
    }
    return;
  }

  // ----------------------------------------------------------
  // 4. LEGACY SINGLE-KEY / DASHBOARD COMMANDS (N, S, W, E, etc.)
  // ----------------------------------------------------------
  handleLegacyCommand(line, source);
}

// ============================================================
// PARSE MOTOR PART: "<DIR,SPEED,TIME_MS>" or "<STOP>"
// ============================================================

void parseMotorPart(String str, String source)
{
  str.trim();

  // Strip '<' and '>'
  if (str.startsWith("<"))
    str = str.substring(1);
  if (str.endsWith(">"))
    str = str.substring(0, str.length() - 1);
  str.trim();

  if (str.length() == 0)
    return;

  // Check for MIX command: <MIX,vx,vy,w> or <MIX,vx,vy,w,duration>
  if (str.startsWith("MIX,") || str.startsWith("mix,"))
  {
    String rest = str.substring(4);
    int p1 = rest.indexOf(',');
    int p2 = (p1 != -1) ? rest.indexOf(',', p1 + 1) : -1;
    int p3 = (p2 != -1) ? rest.indexOf(',', p2 + 1) : -1;
    if (p1 != -1 && p2 != -1)
    {
      int vx = rest.substring(0, p1).toInt();
      int vy = rest.substring(p1 + 1, p2).toInt();
      int w  = (p3 != -1) ? rest.substring(p2 + 1, p3).toInt() : rest.substring(p2 + 1).toInt();
      unsigned long dur = (p3 != -1) ? (unsigned long)rest.substring(p3 + 1).toInt() : 0;
      mixDrive(vx, vy, w);
      if (dur > 0)
      {
        motorStartTime = millis();
        motorDuration = dur;
        motorTimedActive = true;
        continuousActive = false;
      }
      else
      {
        motorTimedActive = false;
        motorDuration = 0;
        continuousActive = (vx != 0 || vy != 0 || w != 0);
      }
      static unsigned long lastMixLogTime = 0;
      if (millis() - lastMixLogTime > 1000)
      {
        logOut("MOTOR: MIX Vx=" + String(vx) + " Vy=" + String(vy) + " W=" + String(w) + " [" + source + "]");
        lastMixLogTime = millis();
      }
      return;
    }
  }

  // Split by comma
  int firstComma = str.indexOf(',');
  if (firstComma == -1)
  {
    // Single token e.g. "STOP", "FWD", "Q"
    executeMotorAction(str, SPEED_LIMIT, 0, source);
    return;
  }

  String dirStr = str.substring(0, firstComma);
  dirStr.trim();

  int speed = SPEED_LIMIT;
  unsigned long duration = 0;

  int secondComma = str.indexOf(',', firstComma + 1);
  if (secondComma == -1)
  {
    String speedStr = str.substring(firstComma + 1);
    speedStr.trim();
    if (speedStr.length() > 0)
      speed = speedStr.toInt();
  }
  else
  {
    String speedStr = str.substring(firstComma + 1, secondComma);
    String timeStr = str.substring(secondComma + 1);
    speedStr.trim();
    timeStr.trim();

    if (speedStr.length() > 0)
      speed = speedStr.toInt();
    if (timeStr.length() > 0)
      duration = (unsigned long)timeStr.toInt();
  }

  executeMotorAction(dirStr, speed, duration, source);
}

// ============================================================
// PARSE LED PART: "<LED,MOOD>" or "<MOOD>"
// ============================================================

void parseLedPart(String str)
{
  str.trim();

  if (str.startsWith("<"))
    str = str.substring(1);
  if (str.endsWith(">"))
    str = str.substring(0, str.length() - 1);
  str.trim();

  if (str.startsWith("LED,") || str.startsWith("led,"))
  {
    str = str.substring(4);
    str.trim();
  }
  else if (str.startsWith("LED:") || str.startsWith("led:"))
  {
    str = str.substring(4);
    str.trim();
  }

  setLedMoodByName(str);
}

// ============================================================
// PARSE COLON MOTOR: "MOTOR:FWD:255:500"
// ============================================================

void parseColonMotor(String str, String source)
{
  str = str.substring(6);
  str.trim();

  int firstColon = str.indexOf(':');
  if (firstColon == -1)
  {
    executeMotorAction(str, SPEED_LIMIT, 0, source);
    return;
  }

  String dirStr = str.substring(0, firstColon);
  dirStr.trim();

  int speed = SPEED_LIMIT;
  unsigned long duration = 0;

  int secondColon = str.indexOf(':', firstColon + 1);
  if (secondColon == -1)
  {
    String speedStr = str.substring(firstColon + 1);
    speedStr.trim();
    if (speedStr.length() > 0)
      speed = speedStr.toInt();
  }
  else
  {
    String speedStr = str.substring(firstColon + 1, secondColon);
    String timeStr = str.substring(secondColon + 1);
    speedStr.trim();
    timeStr.trim();

    if (speedStr.length() > 0)
      speed = speedStr.toInt();
    if (timeStr.length() > 0)
      duration = (unsigned long)timeStr.toInt();
  }

  executeMotorAction(dirStr, speed, duration, source);
}

void parseColonLed(String str)
{
  str = str.substring(4);
  str.trim();
  setLedMoodByName(str);
}

// ============================================================
// EXECUTE MOTOR ACTION
// ============================================================

void executeMotorAction(String dir, int speed, unsigned long durationMs, String source)
{
  dir.toUpperCase();
  dir.trim();

  if (speed <= 0)
    speed = SPEED_LIMIT;
  speed = constrain(speed, 0, 255);

  // Stop Actions
  if (dir == "STOP" || dir == "HALT" || dir == "Q" || dir == "X")
  {
    stopAll();
    motorTimedActive = false;
    continuousActive = false;
    motorDuration = 0;
    logOut("CMD: STOP [" + source + "]");
    return;
  }

  // Duration Update & Safeguard:
  // - durationMs == 0: CONTINUOUS mode (runs indefinitely until STOP). Preserved as 0!
  // - durationMs > 0: If short (< minTimedDurationMs), update to sustained duration so the chassis doesn't jerk and stall.
  unsigned long effectiveDurationMs = durationMs;

  if (effectiveDurationMs > 0 && durationOverrideActive)
  {
    bool isPivot = (dir.indexOf("PIVOT") != -1 || dir == "CW" || dir == "CCW");
    if (isPivot)
    {
      if (effectiveDurationMs < 1200)
      {
        effectiveDurationMs = defaultPivotDurationMs;
        logOut("INFO: Updated pivot duration from " + String(durationMs) + "ms -> " + String(effectiveDurationMs) + "ms");
      }
    }
    else
    {
      if (effectiveDurationMs < minTimedDurationMs)
      {
        effectiveDurationMs = defaultTranslationDurationMs;
        logOut("INFO: Updated translation duration from " + String(durationMs) + "ms -> " + String(effectiveDurationMs) + "ms");
      }
    }
  }

  // Setup Non-Blocking Duration Timer
  if (effectiveDurationMs > 0)
  {
    motorStartTime = millis();
    motorDuration = effectiveDurationMs;
    motorTimedActive = true;
    continuousActive = false;
  }
  else
  {
    motorTimedActive = false;
    motorDuration = 0;
    continuousActive = true;
  }

  // Mecanum Direction Execution
  if (dir == "FWD" || dir == "FORWARD" || dir == "N")
  {
    moveForward(speed);
  }
  else if (dir == "REV" || dir == "BACK" || dir == "BACKWARD" || dir == "BWD" || dir == "S")
  {
    moveBackward(speed);
  }
  else if (dir == "STRAFE_L" || dir == "LEFT" || dir == "W")
  {
    strafeLeft(speed);
  }
  else if (dir == "STRAFE_R" || dir == "RIGHT" || dir == "E")
  {
    strafeRight(speed);
  }
  else if (dir == "DIAG_FL" || dir == "FWD_L" || dir == "FORWARD-LEFT" || dir == "NW")
  {
    moveForwardLeft(speed);
  }
  else if (dir == "DIAG_FR" || dir == "FWD_R" || dir == "FORWARD-RIGHT" || dir == "NE")
  {
    moveForwardRight(speed);
  }
  else if (dir == "DIAG_BL" || dir == "BWD_L" || dir == "BACKWARD-LEFT" || dir == "SW")
  {
    moveBackwardLeft(speed);
  }
  else if (dir == "DIAG_BR" || dir == "BWD_R" || dir == "BACKWARD-RIGHT" || dir == "SE")
  {
    moveBackwardRight(speed);
  }
  else if (dir == "PIVOT_L" || dir == "CCW" || dir == "ROTATE_CCW" || dir == "ROTATE_L")
  {
    rotateCCW(speed);
  }
  else if (dir == "PIVOT_R" || dir == "CW" || dir == "ROTATE_CW" || dir == "ROTATE_R")
  {
    rotateCW(speed);
  }
  else if (dir == "WANDER")
  {
    mixDrive(speed, 0, speed / 3);
  }
  else
  {
    logOut("ERR: Unknown direction: " + dir);
    return;
  }

  logOut("MOTOR: " + dir + " (Spd=" + String(speed) + ", Time=" + String(effectiveDurationMs) + "ms) [" + source + "]");
}

// ============================================================
// TIMED MOTOR CHECK (NON-BLOCKING)
// ============================================================

void checkMotorTimer()
{
  if (motorTimedActive && motorDuration > 0)
  {
    if (millis() - motorStartTime >= motorDuration)
    {
      stopAll();
      motorTimedActive = false;
      motorDuration = 0;
      logOut("MOTOR: Auto-stopped (Duration reached)");
    }
  }

  // Comms watchdog for continuous streaming commands (duration=0)
  if (continuousActive)
  {
    if (millis() - lastSerialCmdTime > SERIAL_COMMS_TIMEOUT_MS)
    {
      stopAll();
      continuousActive = false;
      logOut("MOTOR: Safety Watchdog triggered (Jetson comms timeout). Stopped.");
    }
  }
}

// ============================================================
// SET LED MOOD
// ============================================================

void setLedMood(LedMood mood)
{
  currentLedMood = mood;
  ledPhaseStartMillis = millis();
  ledPreviousMillis = millis();
  ledBrightness = 0;
}

void setLedMoodByName(String moodName)
{
  moodName.toUpperCase();
  moodName.trim();

  if (moodName == "IDLE_WHITE" || moodName == "IDLE" || moodName == "WHITE")
  {
    setLedMood(MOOD_IDLE_WHITE);
    logOut("LED: IDLE_WHITE (Solid ON)");
  }
  else if (moodName == "CURIOSITY_GREEN" || moodName == "CURIOSITY" || moodName == "GREEN")
  {
    setLedMood(MOOD_CURIOSITY_GREEN);
    logOut("LED: CURIOSITY_GREEN (Pulsing Fade)");
  }
  else if (moodName == "FLIRT_PINK" || moodName == "FLIRT" || moodName == "PINK")
  {
    setLedMood(MOOD_FLIRT_PINK);
    logOut("LED: FLIRT_PINK (Double Blink)");
  }
  else if (moodName == "ALERT_RED" || moodName == "ALERT" || moodName == "RED")
  {
    setLedMood(MOOD_ALERT_RED);
    logOut("LED: ALERT_RED (Aggressive Strobe)");
  }
  else if (moodName == "THINKING_BLUE" || moodName == "THINKING" || moodName == "BLUE")
  {
    setLedMood(MOOD_THINKING_BLUE);
    logOut("LED: THINKING_BLUE (Random Flicker)");
  }
  else if (moodName == "SPEAKING" || moodName == "SPEAK")
  {
    setLedMood(MOOD_SPEAKING);
    logOut("LED: SPEAKING (Speech Rhythm)");
  }
  else if (moodName == "OFF" || moodName == "0")
  {
    setLedMood(MOOD_OFF);
    logOut("LED: OFF");
  }
  else
  {
    logOut("ERR: Unknown LED mood: " + moodName);
  }
}

// ============================================================
// LEGACY COMMAND HANDLER (N, S, W, E, NE, NW, SE, SW, CW, CCW, Q)
// ============================================================

void handleLegacyCommand(String command, String source)
{
  command.toUpperCase();

  if (command == "Q" || command == "X" || command == "STOP")
  {
    stopAll();
    motorTimedActive = false;
    logOut("CMD: STOP [" + source + "]");
    return;
  }

  if (command == "N")   { motorTimedActive = false; continuousActive = true; moveForward();           logOut("CMD: FORWARD [" + source + "]"); return; }
  if (command == "S")   { motorTimedActive = false; continuousActive = true; moveBackward();          logOut("CMD: BACKWARD [" + source + "]"); return; }
  if (command == "W")   { motorTimedActive = false; continuousActive = true; strafeLeft();            logOut("CMD: LEFT [" + source + "]"); return; }
  if (command == "E")   { motorTimedActive = false; continuousActive = true; strafeRight();           logOut("CMD: RIGHT [" + source + "]"); return; }
  if (command == "NE")  { motorTimedActive = false; continuousActive = true; moveForwardRight();      logOut("CMD: FORWARD-RIGHT [" + source + "]"); return; }
  if (command == "NW")  { motorTimedActive = false; continuousActive = true; moveForwardLeft();       logOut("CMD: FORWARD-LEFT [" + source + "]"); return; }
  if (command == "SE")  { motorTimedActive = false; continuousActive = true; moveBackwardRight();     logOut("CMD: BACKWARD-RIGHT [" + source + "]"); return; }
  if (command == "SW")  { motorTimedActive = false; continuousActive = true; moveBackwardLeft();      logOut("CMD: BACKWARD-LEFT [" + source + "]"); return; }
  if (command == "CW")  { motorTimedActive = false; continuousActive = true; rotateCW();              logOut("CMD: ROTATE CW [" + source + "]"); return; }
  if (command == "CCW") { motorTimedActive = false; continuousActive = true; rotateCCW();             logOut("CMD: ROTATE CCW [" + source + "]"); return; }

  logOut("ERR: Unrecognized command: " + command);
}

// ============================================================
// UNIFIED SERIAL + BLE LOGGER & NOTIFIER
// ============================================================

void logOut(String msg)
{
  Serial.println(msg);
  bleBroadcast(msg);
}

void bleBroadcast(String msg)
{
  if (bleCentralConnected)
  {
    if (msg.length() > 125)
      msg = msg.substring(0, 125);

    commandChar.writeValue(msg);
  }
}

// ============================================================
// STATUS LED / MOSFET UPDATE (NON-BLOCKING)
// ============================================================

void updateStatusLED()
{
  unsigned long currentMillis = millis();

  switch (currentLedMood)
  {
    case MOOD_STARTUP:
      if (currentMillis - ledPhaseStartMillis >= startupBlinkDuration)
      {
        setLedMood(MOOD_IDLE_WHITE);
      }
      else if (currentMillis - ledPreviousMillis >= blinkInterval)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;

    case MOOD_IDLE_WHITE:
      digitalWrite(LED_PIN, HIGH);
      break;

    case MOOD_OFF:
      digitalWrite(LED_PIN, LOW);
      break;

    case MOOD_CURIOSITY_GREEN:
      if (currentMillis - ledPreviousMillis >= 12)
      {
        ledPreviousMillis = currentMillis;
        if (ledFadeDirection)
        {
          ledBrightness += 2;
          if (ledBrightness >= 255) { ledBrightness = 255; ledFadeDirection = false; }
        }
        else
        {
          ledBrightness -= 2;
          if (ledBrightness <= 10) { ledBrightness = 10; ledFadeDirection = true; }
        }
        analogWrite(LED_PIN, ledBrightness);
      }
      break;

    case MOOD_FLIRT_PINK:
      {
        unsigned long cycleTime = (currentMillis - ledPhaseStartMillis) % 1000;
        if (cycleTime < 80)
          digitalWrite(LED_PIN, HIGH);
        else if (cycleTime < 160)
          digitalWrite(LED_PIN, LOW);
        else if (cycleTime < 240)
          digitalWrite(LED_PIN, HIGH);
        else
          digitalWrite(LED_PIN, LOW);
      }
      break;

    case MOOD_ALERT_RED:
      if (currentMillis - ledPreviousMillis >= 40)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;

    case MOOD_THINKING_BLUE:
      if (currentMillis - ledPreviousMillis >= 25)
      {
        ledPreviousMillis = currentMillis;
        int flickerVal = random(30, 255);
        analogWrite(LED_PIN, flickerVal);
      }
      break;

    case MOOD_SPEAKING:
      if (currentMillis - ledPreviousMillis >= 200)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;
  }
}

// ============================================================
// BASIC MOVEMENT IMPLEMENTATIONS
// ============================================================

void moveForward(int speed) { mixDrive(speed, 0, 0); }
void moveBackward(int speed) { mixDrive(-speed, 0, 0); }
void strafeLeft(int speed) { mixDrive(0, speed, 0); }
void strafeRight(int speed) { mixDrive(0, -speed, 0); }

void moveForwardRight(int speed) { mixDrive(speed, -speed, 0); }
void moveForwardLeft(int speed) { mixDrive(speed, speed, 0); }
void moveBackwardRight(int speed) { mixDrive(-speed, -speed, 0); }
void moveBackwardLeft(int speed) { mixDrive(-speed, speed, 0); }

void rotateCW(int speed) { mixDrive(0, 0, -speed); }
void rotateCCW(int speed) { mixDrive(0, 0, speed); }

void stopAll()
{
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}

// ============================================================
// HOLONOMIC MECANUM MIXING
// ============================================================

void mixDrive(int Vx, int Vy, int W)
{
  // ==========================================================
  // Layout: M1=Front-Left, M2=Back-Left, M3=Back-Right, M4=Front-Right
  // ROS convention: +Vx=forward, +Vy=left, +W=counter-clockwise
  // Standard Mecanum (X-config) mixing - confirmed working with physical tests:
  //   FL(M1) = Vx - Vy - W
  //   BL(M2) = Vx + Vy - W
  //   BR(M3) = Vx - Vy + W
  //   FR(M4) = Vx + Vy + W
  // ==========================================================
  float M1 = Vx - Vy - W;   // Front-Left  (FL)
  float M2 = Vx + Vy - W;   // Back-Left   (BL)
  float M3 = Vx - Vy + W;   // Back-Right  (BR)
  float M4 = Vx + Vy + W;   // Front-Right (FR)


  // Normalization - scale down proportionally if any motor exceeds SPEED_LIMIT
  float maxMag = max(max(fabs(M1), fabs(M2)), max(fabs(M3), fabs(M4)));

  if (maxMag > SPEED_LIMIT)
  {
    float scale = (float)SPEED_LIMIT / maxMag;
    M1 *= scale;
    M2 *= scale;
    M3 *= scale;
    M4 *= scale;
  }

  setMotor(1, (int)M1);
  setMotor(2, (int)M2);
  setMotor(3, (int)M3);
  setMotor(4, (int)M4);

  if (DEBUG)
  {
    static unsigned long lastMixDebugTime = 0;
    if (millis() - lastMixDebugTime > 1000)
    {
      String line = "MIX Vx=" + String(Vx) + " Vy=" + String(Vy) + " W=" + String(W) +
                    " -> M1(FL)=" + String((int)M1) + " M2(BL)=" + String((int)M2) +
                    " M3(BR)=" + String((int)M3) + " M4(FR)=" + String((int)M4);
      logOut(line);
      lastMixDebugTime = millis();
    }
  }
}

// ============================================================
// MOTOR CALIBRATION TOKEN PARSER
// ============================================================

bool parseMotorToken(String token, int &motor, int &speed, bool &reverse)
{
  if (token.length() < 4)
    return false;

  char dirChar = token.charAt(0);
  if (dirChar != 'T' && dirChar != 't')
    return false;

  reverse = (dirChar == 't');
  char motorChar = token.charAt(1);
  if (motorChar < '1' || motorChar > '4')
    return false;

  motor = motorChar - '0';
  if (token.charAt(2) != '_')
    return false;

  String speedStr = token.substring(3);
  if (speedStr.length() == 0)
    return false;

  for (unsigned int i = 0; i < speedStr.length(); i++)
  {
    if (!isDigit(speedStr.charAt(i)))
      return false;
  }

  speed = constrain(speedStr.toInt(), 0, 255);
  return true;
}

void runMotorTokens(String line)
{
  bool used[5] = {false, false, false, false, false};
  int speed[5] = {0, 0, 0, 0, 0};
  bool rev[5] = {false, false, false, false, false};
  String summary = "";
  bool anyValid = false;

  int start = 0;
  while (start < (int)line.length())
  {
    int spaceIdx = line.indexOf(' ', start);
    String token;
    if (spaceIdx == -1)
      token = line.substring(start);
    else
      token = line.substring(start, spaceIdx);

    token.trim();
    if (token.length() > 0)
    {
      int m;
      int spd;
      bool rv;
      if (parseMotorToken(token, m, spd, rv))
      {
        used[m] = true;
        speed[m] = spd;
        rev[m] = rv;
        anyValid = true;
        summary += "M" + String(m) + "=" + (rv ? "-" : "+") + String(spd) + " ";
      }
      else
      {
        logOut("[TEST] Bad token: " + token);
      }
    }

    if (spaceIdx == -1)
      break;
    start = spaceIdx + 1;
  }

  if (!anyValid)
  {
    logOut("[TEST] No valid tokens.");
    return;
  }

  for (int m = 1; m <= 4; m++)
  {
    int s = used[m] ? (rev[m] ? -speed[m] : speed[m]) : 0;
    setMotor(m, s);
  }

  String testLog = "[TEST] " + summary + "(others stopped)";
  logOut(testLog);
}

// ============================================================
// MOTOR INVERSION & RAW PWM (IBT-2 BTS7960)
// ============================================================

void setMotor(int motor, int speed)
{
  switch (motor)
  {
    case 1: if (M1_INVERT) speed = -speed; break;
    case 2: if (M2_INVERT) speed = -speed; break;
    case 3: if (M3_INVERT) speed = -speed; break;
    case 4: if (M4_INVERT) speed = -speed; break;
    default: return;
  }
  setRawMotor(motor, speed);
}

void setRawMotor(int motor, int speed)
{
  int rpwmPin;
  int lpwmPin;

  switch (motor)
  {
    case 1: rpwmPin = M1_RPWM; lpwmPin = M1_LPWM; break;
    case 2: rpwmPin = M2_RPWM; lpwmPin = M2_LPWM; break;
    case 3: rpwmPin = M3_RPWM; lpwmPin = M3_LPWM; break;
    case 4: rpwmPin = M4_RPWM; lpwmPin = M4_LPWM; break;
    default: return;
  }

  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    analogWrite(lpwmPin, 0);
    analogWrite(rpwmPin, speed);
  }
  else if (speed < 0)
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  }
  else
  {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}
