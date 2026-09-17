#include <ArduinoBLE.h>

// ==========================================
// DhruvBrain Arduino Motor & LED Controller (BLE)
// ==========================================

// BLE Setup
BLEService dhruvService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEStringCharacteristic commandChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLENotify, 50);

// Pin Definitions
const int LED_PIN = 9; // MOSFET triggering the 12V Blue Light

// Motor Pins (Assuming 2x L298N for independent 4-wheel control)
// Front Left
const int IN1_FL = 2;
const int IN2_FL = 3;
// Back Left
const int IN3_BL = 4;
const int IN4_BL = 5;
// Front Right
const int IN5_FR = 6;
const int IN6_FR = 7;
// Back Right
const int IN7_BR = 8;
const int IN8_BR = 10;
String currentMood = "IDLE_WHITE";
unsigned long previousMillis = 0;
bool ledState = LOW;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN3_BL, OUTPUT);
  pinMode(IN4_BL, OUTPUT);
  pinMode(IN5_FR, OUTPUT);
  pinMode(IN6_FR, OUTPUT);
  pinMode(IN7_BR, OUTPUT);
  pinMode(IN8_BR, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  stopMotors();

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  BLE.setLocalName("Dhruv_Arduino");
  BLE.setAdvertisedService(dhruvService);

  dhruvService.addCharacteristic(commandChar);
  BLE.addService(dhruvService);

  BLE.advertise();
  Serial.println("Arduino BLE Peripheral Active. Waiting for Jetson...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      // 1. Check for incoming BLE commands
      if (commandChar.written()) {
        String payload = commandChar.value();
        payload.trim();
        if (payload.length() > 0) {
          parseCommand(payload);
        }
      }

      // 2. Manage LED blinking non-blockingly
      handleLED();
    }
    
    Serial.println("Disconnected from central.");
    stopMotors();
  } else {
    // Keep blinking if not connected
    handleLED();
  }
}

// Parse something like "<FWD,50,500>|<LED,IDLE_WHITE>"
void parseCommand(String payload) {
  int pipeIndex = payload.indexOf('|');
  
  String motorCmd = "";
  String ledCmd = "";

  if (pipeIndex != -1) {
    motorCmd = payload.substring(0, pipeIndex);
    ledCmd = payload.substring(pipeIndex + 1);
  } else {
    // If no pipe, check if it's purely an LED or Motor command
    if (payload.indexOf("<LED") != -1) {
      ledCmd = payload;
    } else {
      motorCmd = payload;
    }
  }

  // Process Motor
  if (motorCmd.length() > 0) {
    if (motorCmd.indexOf("<FWD") != -1) {
      // Implement Forward
    } else if (motorCmd.indexOf("<BWD") != -1) {
      // Implement Backward
    } else if (motorCmd.indexOf("<LEFT") != -1) {
      // Implement Left
    } else if (motorCmd.indexOf("<RIGHT") != -1) {
      // Implement Right
    } else if (motorCmd.indexOf("<STOP") != -1) {
      stopMotors();
    } else if (motorCmd.indexOf("<WANDER") != -1) {
      // Implement Wander sequence
    }
  }

  // Process LED
  if (ledCmd.length() > 0 && ledCmd.startsWith("<LED,") && ledCmd.endsWith(">")) {
    int startIdx = 5; // Length of "<LED,"
    int endIdx = ledCmd.length() - 1;
    String newMood = ledCmd.substring(startIdx, endIdx);
    
    // Only reset timer state if the mood actually changes
    if (newMood != currentMood) {
      currentMood = newMood;
      previousMillis = millis(); 
      ledState = HIGH; 
      digitalWrite(LED_PIN, ledState);
    }
  }
}

void stopMotors() {
  digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW);
  digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, LOW);
  digitalWrite(IN5_FR, LOW); digitalWrite(IN6_FR, LOW);
  digitalWrite(IN7_BR, LOW); digitalWrite(IN8_BR, LOW);
}

void executeMovement(String dir, int speed, int duration) {
  // Mecanum Drive Logic
  if (dir == "FWD") {
    digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
    digitalWrite(IN5_FR, HIGH); digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_BR, HIGH); digitalWrite(IN8_BR, LOW);
  } else if (dir == "REV") {
    digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);
    digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, HIGH);
    digitalWrite(IN5_FR, LOW); digitalWrite(IN6_FR, HIGH);
    digitalWrite(IN7_BR, LOW); digitalWrite(IN8_BR, HIGH);
  } else if (dir == "STRAFE_L") {
    digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);
    digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
    digitalWrite(IN5_FR, HIGH); digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_BR, LOW); digitalWrite(IN8_BR, HIGH);
  } else if (dir == "STRAFE_R") {
    digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, HIGH);
    digitalWrite(IN5_FR, LOW); digitalWrite(IN6_FR, HIGH);
    digitalWrite(IN7_BR, HIGH); digitalWrite(IN8_BR, LOW);
  } else if (dir == "DIAG_FL") {
    digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
    digitalWrite(IN5_FR, HIGH); digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_BR, LOW); digitalWrite(IN8_BR, LOW);
  } else if (dir == "DIAG_FR") {
    digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, LOW);
    digitalWrite(IN5_FR, LOW); digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_BR, HIGH); digitalWrite(IN8_BR, LOW);
  } else if (dir == "PIVOT_L" || dir == "LEFT") {
    digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);
    digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, HIGH);
    digitalWrite(IN5_FR, HIGH); digitalWrite(IN6_FR, LOW);
    digitalWrite(IN7_BR, HIGH); digitalWrite(IN8_BR, LOW);
  } else if (dir == "PIVOT_R" || dir == "RIGHT") {
    digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
    digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
    digitalWrite(IN5_FR, LOW); digitalWrite(IN6_FR, HIGH);
    digitalWrite(IN7_BR, LOW); digitalWrite(IN8_BR, HIGH);
  } else if (dir == "STOP") {
    stopMotors();
    return;
  }
}

void handleLED() {
  unsigned long currentMillis = millis();
  unsigned long interval = 0;

  if (currentMood == "IDLE_WHITE") {
    // Solid ON
    if (ledState == LOW) {
      ledState = HIGH;
      digitalWrite(LED_PIN, ledState);
    }
    return;
  } else if (currentMood == "SPEAKING") {
    interval = 200; // Fast blink
  } else if (currentMood == "THINKING_BLUE") {
    interval = 1000; // Slow blink
  } else if (currentMood == "ALERT_RED") {
    interval = 100; // Rapid strobe
  } else if (currentMood == "CURIOSITY_GREEN" || currentMood == "FLIRT_PINK") {
    interval = 500; // Medium blink
  } else {
    // Default to solid ON
    if (ledState == LOW) {
      ledState = HIGH;
      digitalWrite(LED_PIN, ledState);
    }
    return;
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}
