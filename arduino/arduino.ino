// ==========================================
// DhruvBrain Arduino Motor & LED Controller
// ==========================================

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
  
  Serial.println("Arduino Initialized");
}

void loop() {
  // 1. Check for incoming Serial commands
  if (Serial.available() > 0) {
    String payload = Serial.readStringUntil('\n');
    payload.trim();
    if (payload.length() > 0) {
      parseCommand(payload);
    }
  }

  // 2. Manage LED blinking non-blockingly
  handleLED();
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
    currentMood = ledCmd.substring(startIdx, endIdx);
    
    // Reset timer state on mood change
    previousMillis = millis(); 
    ledState = HIGH; 
    digitalWrite(LED_PIN, ledState);
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
