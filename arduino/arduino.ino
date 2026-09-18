#include <ArduinoBLE.h>

// ============================================================
// DHRUVBRAIN — JETSON NANO + ARDUINO UNO R4 HOLONOMIC CONTROLLER
// 4-Wheel Mecanum H-Drive + 12V LED Mood Lighting
//
// TIMED & CONTINUOUS EXECUTION MODE:
//   - If TIME_MS is provided in command (e.g. <FWD,150,500>),
//     motors execute for that duration and then auto-stop.
//   - If TIME_MS is 0 or omitted, motors run continuously until STOP.
//
// WHEEL LAYOUT (Mecanum 4-Wheel Holonomic)
//
//                    FRONT
//                      ↑
//             M1                 M4
//            FRONT-L           FRONT-R
//               \               /
//                \             /
//
//                /             \
//               /               \
//             M2                 M3
//            REAR-L            REAR-R
//                      ↓
//                     REAR
//
// Roller orientation:
//     M1 = "\"       M4 = "/"
//     M2 = "/"       M3 = "\"
//
// ============================================================
//
// BLE UUIDs MATCHING JETSON NANO CENTRAL:
//   Service UUID:        19b10000-e8f2-537e-4f6c-d104768a1214
//   Characteristic UUID: 19b10001-e8f2-537e-4f6c-d104768a1214
//   Device Local Name:   "Dhruv_Arduino"
//
// PAYLOAD FORMAT FROM JETSON NANO:
//   "<MOTOR_DIR,SPEED,TIME_MS>|<LED,MOOD>\n"
//
// Examples:
//   "<FWD,50,500>|<LED,FLIRT_PINK>"     -> Moves forward 500ms then auto-stops + double pink blink
//   "<STRAFE_L,150,500>|<LED,IDLE_WHITE>"-> Strafes left 500ms then auto-stops + white LED
//   "<PIVOT_R,180,300>|<LED,ALERT_RED>"  -> Pivots CW 300ms then auto-stops + alert red strobe
//   "<STOP>|<LED,THINKING_BLUE>"         -> Stops all motors immediately + thinking blue flicker
//
// ============================================================

// ============================================================
// BLE DEFINITIONS (Matching dummy-jetson.ino / DhruvBrain)
// ============================================================

BLEService dhruvService(
  "19b10000-e8f2-537e-4f6c-d104768a1214"
);

BLEStringCharacteristic commandChar(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,
  80
);

bool bleCentralConnected = false;

// ============================================================
// BLE / SERIAL LOGGING
// ============================================================

void bleLog(String msg)
{
  Serial.println(msg);

  if (bleCentralConnected)
  {
    if (msg.length() > 79)
      msg = msg.substring(0, 79);

    commandChar.writeValue(msg);
  }
}

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
// SETTINGS
// ============================================================

const int SPEED_LIMIT = 180;
const bool DEBUG = true;

// Motor direction inversion flags
bool M1_INVERT = false;
bool M2_INVERT = false;
bool M3_INVERT = true;
bool M4_INVERT = true;

bool testModeActive = false;

// ============================================================
// TIMED MOTOR EXECUTION (NON-BLOCKING)
// ============================================================

unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
bool motorTimedActive = false;

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
const unsigned long startupBlinkDuration = 3000;

unsigned long ledPreviousMillis = 0;
unsigned long ledPhaseStartMillis = 0;
int ledBrightness = 0;
bool ledState = LOW;
bool ledFadeDirection = true;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void handleSerialLine(String line);
void parseJetsonCommand(String line);
void parseMotorPart(String str);
void parseLedPart(String str);
void parseColonMotor(String str);
void parseColonLed(String str);
void executeMotorAction(String dir, int speed, unsigned long durationMs);
void setLedMood(LedMood mood);
void setLedMoodByName(String moodName);

void handleLegacyCommand(String command);
void checkMotorTimer();
void updateStatusLED();

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

void setup()
{
  Serial.begin(115200);

  // Motor PWM Pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  pinMode(M3_RPWM, OUTPUT);
  pinMode(M3_LPWM, OUTPUT);
  pinMode(M4_RPWM, OUTPUT);
  pinMode(M4_LPWM, OUTPUT);

  stopAll();

  // Status MOSFET / LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledPhaseStartMillis = millis();

  // BLE Initialization
  if (!BLE.begin())
  {
    Serial.println("ERROR: starting Bluetooth Low Energy failed!");
    while (1)
    {
      stopAll();
      delay(100);
    }
  }

  // Set Local Name & Advertised Service matching Jetson Nano
  BLE.setLocalName("Dhruv_Arduino");
  BLE.setAdvertisedService(dhruvService);

  dhruvService.addCharacteristic(commandChar);
  BLE.addService(dhruvService);

  BLE.advertise();

  Serial.println();
  Serial.println("==================================================");
  Serial.println("   DHRUVBRAIN — JETSON NANO (USB) + PHONE (BLE)");
  Serial.println("==================================================");
  Serial.println("BLE Name:    Dhruv_Arduino");
  Serial.println("Service:     19b10000-e8f2-537e-4f6c-d104768a1214");
  Serial.println("CommandChar: 19b10001-e8f2-537e-4f6c-d104768a1214");
  Serial.println("Waiting for Phone BLE connection...");
  Serial.println("==================================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  updateStatusLED();
  checkMotorTimer();

  // 1. Check Serial input (for USB debugging from Jetson or PC)
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      handleSerialLine(line);
    }
  }

  // 2. Check BLE Central connection
  BLEDevice central = BLE.central();

  if (central)
  {
    bleCentralConnected = true;
    Serial.print("Connected to Phone BLE: ");
    Serial.println(central.address());
    bleLog("Connected: " + central.address());

    while (central.connected())
    {
      updateStatusLED();
      checkMotorTimer();

      // Check incoming BLE commands from Jetson
      if (commandChar.written())
      {
        String value = commandChar.value();
        value.trim();
        if (value.length() > 0)
        {
          handleSerialLine(value);
        }
      }

      // Check Serial input while BLE is connected
      if (Serial.available())
      {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
        {
          handleSerialLine(line);
        }
      }

      BLE.poll();
    }

    // Safety Disconnect: Stop all motors immediately
    stopAll();
    motorTimedActive = false;
    bleCentralConnected = false;
    Serial.println("Phone BLE disconnected. Motors stopped.");
  }
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
      bleLog("MOTOR: Auto-stopped (Duration reached)");
    }
  }
}

// ============================================================
// MAIN COMMAND ROUTER & JETSON PARSER
// ============================================================

void handleSerialLine(String line)
{
  line.trim();
  if (line.length() == 0)
    return;

  // Motor Calibration Token Parser (e.g. T1_150, t2_100)
  char first = line.charAt(0);
  if ((first == 'T' || first == 't') && line.indexOf('_') != -1 && line.indexOf('<') == -1)
  {
    testModeActive = true;
    runMotorTokens(line);
    return;
  }

  // Parse Jetson combined / bracketed / legacy commands
  parseJetsonCommand(line);
  
  // Send ACK back to Jetson to unblock its UART queue instantly
  Serial.println("ACK");
}

// ============================================================
// JETSON NANO COMMAND PARSER
// ============================================================

void parseJetsonCommand(String line)
{
  testModeActive = false;

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
      parseMotorPart(motorPart);

    if (ledPart.length() > 0)
      parseLedPart(ledPart);

    return;
  }

  // ----------------------------------------------------------
  // 2. COLON SEPARATED FORMAT ("MOTOR:..." or "LED:...")
  // ----------------------------------------------------------
  if (line.startsWith("MOTOR:") || line.startsWith("motor:"))
  {
    parseColonMotor(line);
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
      parseMotorPart(line);
    }
    return;
  }

  // ----------------------------------------------------------
  // 4. LEGACY / DIRECT SINGLE CHARACTER / WORD COMMANDS
  // ----------------------------------------------------------
  handleLegacyCommand(line);
}

// ============================================================
// PARSE MOTOR PART: "<DIR,SPEED,TIME_MS>" or "<STOP>"
// ============================================================

void parseMotorPart(String str)
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

  // Split by comma ','
  int firstComma = str.indexOf(',');
  if (firstComma == -1)
  {
    // Single token like "STOP", "FWD", "Q", "WANDER", etc.
    executeMotorAction(str, SPEED_LIMIT, 0);
    return;
  }

  String dirStr = str.substring(0, firstComma);
  dirStr.trim();

  int speed = SPEED_LIMIT;
  unsigned long duration = 0;

  int secondComma = str.indexOf(',', firstComma + 1);
  if (secondComma == -1)
  {
    // Format: <DIR,SPEED>
    String speedStr = str.substring(firstComma + 1);
    speedStr.trim();
    if (speedStr.length() > 0)
      speed = speedStr.toInt();
  }
  else
  {
    // Format: <DIR,SPEED,TIME_MS>
    String speedStr = str.substring(firstComma + 1, secondComma);
    String timeStr = str.substring(secondComma + 1);
    speedStr.trim();
    timeStr.trim();

    if (speedStr.length() > 0)
      speed = speedStr.toInt();
    if (timeStr.length() > 0)
      duration = (unsigned long)timeStr.toInt();
  }

  executeMotorAction(dirStr, speed, duration);
}

// ============================================================
// PARSE LED PART: "<LED,MOOD>" or "<MOOD>"
// ============================================================

void parseLedPart(String str)
{
  str.trim();

  // Strip '<' and '>'
  if (str.startsWith("<"))
    str = str.substring(1);
  if (str.endsWith(">"))
    str = str.substring(0, str.length() - 1);
  str.trim();

  // Strip "LED," or "LED:" if present
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
// PARSE COLON MOTOR: "MOTOR:FWD:255:500" or "MOTOR:STOP"
// ============================================================

void parseColonMotor(String str)
{
  str = str.substring(6);
  str.trim();

  int firstColon = str.indexOf(':');
  if (firstColon == -1)
  {
    executeMotorAction(str, SPEED_LIMIT, 0);
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

  executeMotorAction(dirStr, speed, duration);
}

// ============================================================
// PARSE COLON LED: "LED:FLIRT_PINK"
// ============================================================

void parseColonLed(String str)
{
  str = str.substring(4);
  str.trim();
  setLedMoodByName(str);
}

// ============================================================
// EXECUTE MOTOR ACTION (Mecanum 4-Wheel Direction Mapping)
// ============================================================

void executeMotorAction(String dir, int speed, unsigned long durationMs)
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
    motorDuration = 0;
    bleLog("CMD: STOP");
    return;
  }

  // Setup non-blocking timer
  if (durationMs > 0)
  {
    motorStartTime = millis();
    motorDuration = durationMs;
    motorTimedActive = true;
  }
  else
  {
    motorTimedActive = false;
    motorDuration = 0;
  }

  // Mecanum Direction Mapping
  if (dir == "FWD" || dir == "FORWARD" || dir == "N")
  {
    moveForward(speed);
    bleLog("MOTOR: FWD (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "REV" || dir == "BACK" || dir == "BACKWARD" || dir == "BWD" || dir == "S")
  {
    moveBackward(speed);
    bleLog("MOTOR: REV (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "STRAFE_L" || dir == "LEFT" || dir == "W")
  {
    strafeLeft(speed);
    bleLog("MOTOR: STRAFE_L (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "STRAFE_R" || dir == "RIGHT" || dir == "E")
  {
    strafeRight(speed);
    bleLog("MOTOR: STRAFE_R (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "DIAG_FL" || dir == "FWD_L" || dir == "FORWARD-LEFT" || dir == "NW")
  {
    moveForwardLeft(speed);
    bleLog("MOTOR: DIAG_FL (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "DIAG_FR" || dir == "FWD_R" || dir == "FORWARD-RIGHT" || dir == "NE")
  {
    moveForwardRight(speed);
    bleLog("MOTOR: DIAG_FR (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "DIAG_BL" || dir == "BWD_L" || dir == "BACKWARD-LEFT" || dir == "SW")
  {
    moveBackwardLeft(speed);
    bleLog("MOTOR: DIAG_BL (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "DIAG_BR" || dir == "BWD_R" || dir == "BACKWARD-RIGHT" || dir == "SE")
  {
    moveBackwardRight(speed);
    bleLog("MOTOR: DIAG_BR (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "PIVOT_L" || dir == "CCW" || dir == "ROTATE_CCW" || dir == "ROTATE_L")
  {
    rotateCCW(speed);
    bleLog("MOTOR: PIVOT_L (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "PIVOT_R" || dir == "CW" || dir == "ROTATE_CW" || dir == "ROTATE_R")
  {
    rotateCW(speed);
    bleLog("MOTOR: PIVOT_R (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else if (dir == "WANDER")
  {
    mixDrive(speed, 0, speed / 3);
    bleLog("MOTOR: WANDER (Spd=" + String(speed) + ", Time=" + String(durationMs) + "ms)");
  }
  else
  {
    bleLog("ERR: Unknown direction: " + dir);
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
    bleLog("LED: IDLE_WHITE (Solid ON)");
  }
  else if (moodName == "CURIOSITY_GREEN" || moodName == "CURIOSITY" || moodName == "GREEN")
  {
    setLedMood(MOOD_CURIOSITY_GREEN);
    bleLog("LED: CURIOSITY_GREEN (Pulsing Fade)");
  }
  else if (moodName == "FLIRT_PINK" || moodName == "FLIRT" || moodName == "PINK")
  {
    setLedMood(MOOD_FLIRT_PINK);
    bleLog("LED: FLIRT_PINK (Double Blink)");
  }
  else if (moodName == "ALERT_RED" || moodName == "ALERT" || moodName == "RED")
  {
    setLedMood(MOOD_ALERT_RED);
    bleLog("LED: ALERT_RED (Aggressive Strobe)");
  }
  else if (moodName == "THINKING_BLUE" || moodName == "THINKING" || moodName == "BLUE")
  {
    setLedMood(MOOD_THINKING_BLUE);
    bleLog("LED: THINKING_BLUE (Random Flicker)");
  }
  else if (moodName == "SPEAKING" || moodName == "SPEAK")
  {
    setLedMood(MOOD_SPEAKING);
    bleLog("LED: SPEAKING (Speech Rhythm)");
  }
  else if (moodName == "OFF" || moodName == "0")
  {
    setLedMood(MOOD_OFF);
    bleLog("LED: OFF");
  }
  else
  {
    bleLog("ERR: Unknown LED mood: " + moodName);
  }
}

// ============================================================
// LEGACY COMMAND HANDLER (N, S, W, E, NE, NW, SE, SW, CW, CCW, Q)
// ============================================================

void handleLegacyCommand(String command)
{
  command.toUpperCase();

  if (command == "Q" || command == "X" || command == "STOP")
  {
    stopAll();
    motorTimedActive = false;
    bleLog("CMD: STOP");
    return;
  }

  if (command == "N") { moveForward(); bleLog("CMD: FORWARD"); return; }
  if (command == "S") { moveBackward(); bleLog("CMD: BACKWARD"); return; }
  if (command == "W") { strafeLeft(); bleLog("CMD: LEFT"); return; }
  if (command == "E") { strafeRight(); bleLog("CMD: RIGHT"); return; }
  if (command == "NE") { moveForwardRight(); bleLog("CMD: FORWARD-RIGHT"); return; }
  if (command == "NW") { moveForwardLeft(); bleLog("CMD: FORWARD-LEFT"); return; }
  if (command == "SE") { moveBackwardRight(); bleLog("CMD: BACKWARD-RIGHT"); return; }
  if (command == "SW") { moveBackwardLeft(); bleLog("CMD: BACKWARD-LEFT"); return; }
  if (command == "CW") { rotateCW(); bleLog("CMD: ROTATE CW"); return; }
  if (command == "CCW") { rotateCCW(); bleLog("CMD: ROTATE CCW"); return; }

  bleLog("ERR: Unrecognized command: " + command);
}

// ============================================================
// STATUS LED / MOSFET UPDATE (NON-BLOCKING STATE MACHINE)
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
      // Slow breathing pulsing fade
      if (currentMillis - ledPreviousMillis >= 12)
      {
        ledPreviousMillis = currentMillis;
        if (ledFadeDirection)
        {
          ledBrightness += 2;
          if (ledBrightness >= 255)
          {
            ledBrightness = 255;
            ledFadeDirection = false;
          }
        }
        else
        {
          ledBrightness -= 2;
          if (ledBrightness <= 10)
          {
            ledBrightness = 10;
            ledFadeDirection = true;
          }
        }
        analogWrite(LED_PIN, ledBrightness);
      }
      break;

    case MOOD_FLIRT_PINK:
      // Double blink pattern: ON(80ms) -> OFF(80ms) -> ON(80ms) -> OFF(760ms)
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
      // Fast aggressive strobe: 40ms ON / 40ms OFF
      if (currentMillis - ledPreviousMillis >= 40)
      {
        ledPreviousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
      }
      break;

    case MOOD_THINKING_BLUE:
      // Rapid random flickering (neural pulse)
      if (currentMillis - ledPreviousMillis >= 25)
      {
        ledPreviousMillis = currentMillis;
        int flickerVal = random(30, 255);
        analogWrite(LED_PIN, flickerVal);
      }
      break;

    case MOOD_SPEAKING:
      // Fast 200ms speech rhythm blink
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
  float M1 = Vx - Vy - W;
  float M4 = Vx + Vy + W;
  float M2 = Vx + Vy - W;
  float M3 = Vx - Vy + W;

  // Normalization
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
    String line = "MIX Vx=" + String(Vx) + " Vy=" + String(Vy) + " W=" + String(W) +
                  " -> M1=" + String((int)M1) + " M2=" + String((int)M2) +
                  " M3=" + String((int)M3) + " M4=" + String((int)M4);
    bleLog(line);
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
        bleLog("[TEST] Bad token: " + token);
      }
    }

    if (spaceIdx == -1)
      break;
    start = spaceIdx + 1;
  }

  if (!anyValid)
  {
    bleLog("[TEST] No valid tokens.");
    return;
  }

  for (int m = 1; m <= 4; m++)
  {
    int s = used[m] ? (rev[m] ? -speed[m] : speed[m]) : 0;
    setMotor(m, s);
  }

  bleLog("[TEST] " + summary + "(others stopped)");
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