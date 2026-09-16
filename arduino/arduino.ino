#include <ArduinoBLE.h>

// ==========================================
// DhruvBrain Arduino Motor & LED Controller (BLE)
// ==========================================

// BLE Setup
BLEService dhruvService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEStringCharacteristic commandChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLENotify, 50);

// Pin Definitions
const int LED_PIN = 9; // MOSFET triggering the 12V Blue Light

// Motor Pins (Placeholder for L298N or similar)
const int IN1 = 2;
const int IN2 = 3;
const int IN3 = 4;
const int IN4 = 5;

// State Variables
String currentMood = "IDLE_WHITE";
unsigned long previousMillis = 0;
bool ledState = LOW;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
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
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
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
