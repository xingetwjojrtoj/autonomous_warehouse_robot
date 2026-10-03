#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// --- TLP281 RGB LED Pin Configurations ---
const int RED_PIN = 42;
const int GREEN_PIN = 44;
const int BLUE_PIN = 46;

// --- Pin Configurations ---
const int PIN_ENB = 15;   // Combined Motor Enable Pin
const int PIN_BRK = 14;   // Combined Motor Brake Pin
const int PIN_M1_FR = 2;  // Motor 1 Forward/Reverse Pin
const int PIN_M2_FR = 3;  // Motor 2 Forward/Reverse Pin
const int BUZZER_PIN = 37;
bool buzzerActive = false;
unsigned long previousMillis = 0;
const long interval = 100; // Toggle interval in milliseconds
String command = "Z";

// --- Speed Measurement Interrupt Pins (Arduino Mega) ---
// const int PIN_M1_SPEED = 19;  // Interrupt Pin for Motor 1 (INT2)
// const int PIN_M2_SPEED = 18;  // Interrupt Pin for Motor 2 (INT5)
const int Emergency_button = 30;

// --- Interrupt Pulse Counters ---
// volatile unsigned long m1_pulseCount = 0;
// volatile unsigned long m2_pulseCount = 0;

// --- Motor Specification ---
// Adjust according to your motor's specifications (e.g., 3 * pole pairs)
// const int PULSES_PER_REVOLUTION = 12;
// --- DAC Parameters ---
const uint16_t DAC_MIN = 0;
const uint16_t DAC_BASE_STRAIGHT = 400;  // Base value to immediately jump to on movement
const uint16_t DAC_BASE_TURN = 400; // Base value for Clockwise/Anticlockwise turning
uint16_t DAC_MAX = 900;         // Speed target, adjustable via Serial (0-4095)
const uint16_t DAC_STEP = 100;     // How much the DAC value changes per ramp interval
const uint16_t STEP_DELAY = 30;  // Time (ms) between ramping updates

// --- Control Variables ---
bool motorRunning = false;
char currentDirection = 'S';  // 'F', 'B', 'C', 'A', 'S'
char lastReportedState = ' '; // State tracker to ensure sending only on state change
// bool rampUp = true;
uint16_t dacValue = DAC_MIN;
unsigned long lastRampTime = 0;
unsigned long lastStreamTime = 0;
// unsigned long lastRPMUpdateTime = 0;
// Stream rate for ROS2 (e.g., 50ms = 20Hz publishing rate)
const unsigned long STREAM_INTERVAL = 50;

enum SystemState { STATE_IDLE, STATE_RUNNING, STATE_EMERGENCY };
SystemState lastLedState = STATE_IDLE;

// --- Helper Functions ---

/**
 * Sends current state ONLY when it changes from the previous state.
 */
void checkAndReportState() {
  char currentState;
  if (digitalRead(Emergency_button) == HIGH) {
    currentState = 'E'; // Emergency State
  } else {
    currentState = currentDirection; // 'S', 'F', 'B', 'A', 'C'
  }

  if (currentState != lastReportedState) {
    lastReportedState = currentState;
    Serial.println(currentState);
  }
}

/**
 * Sets the color intensity for Red, Green, and Blue channels.
 * TLP281 + Common-Anode LED: HIGH output from pin pulls LED line LOW to turn it ON.
 */
void setColor(int red, int green, int blue) {
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}

void setSystemStatusColor(SystemState state) {
  if (state == lastLedState) return; // Prevent unnecessary pin writes
  
  switch (state) {
    case STATE_EMERGENCY:
      setColor(0, 255, 255);   // Red
      break;
    case STATE_RUNNING:
      setColor(255, 0, 255);   // Green
      break;
    case STATE_IDLE:
      setColor(255, 255, 0);   // Blue
      break;
  }
  lastLedState = state;
}

void startMotorMotion(uint16_t baseSpeed) {
  motorRunning = true;
  digitalWrite(PIN_BRK, LOW);  // Release Brake
  delay(15);
  digitalWrite(PIN_ENB, LOW);  // Enable Drivers

  // Jump immediately to base speed (2600) without ramping from zero
  dacValue = baseSpeed;
  dac.setVoltage(dacValue, false);
}

void stopMotorMotion() {
  motorRunning = false;
  currentDirection = 'S';
  dacValue = 0;
  dac.setVoltage(0, false);

  digitalWrite(PIN_ENB, HIGH);  // Disable Drivers
  digitalWrite(PIN_BRK, HIGH);  // Engage Brake
}


void setup() {
  Serial.begin(115200);

  // Initialize MCP4725 DAC (Default I2C address 0x60)
  dac.begin(0x60);
  dac.setVoltage(0, false);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  setSystemStatusColor(STATE_IDLE);

  // Configure Control Pins
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_BRK, OUTPUT);
  pinMode(PIN_M1_FR, OUTPUT);
  pinMode(PIN_M2_FR, OUTPUT);
  pinMode(Emergency_button, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  // Set default hardware states (Stopped, Brake active, Forward)
  digitalWrite(PIN_ENB, HIGH);   // Keep motors disabled initially
  digitalWrite(PIN_BRK, HIGH);   // Active Brake (High or Low depends on driver, assuming High=Brake)
  digitalWrite(PIN_M1_FR, LOW);  // Default direction: Forward
  digitalWrite(PIN_M2_FR, LOW);  // Default direction: Forward
}

void loop() {
  // --- Handle Serial Commands ---

  int buttonState = digitalRead(Emergency_button);
  if (buttonState == HIGH) {
    if (motorRunning) {
      stopMotorMotion();
    }
    
    setSystemStatusColor(STATE_EMERGENCY); // Set LED to RED
    checkAndReportState(); // Send 'E' state if changed
    return;
  }

  // --- Update Non-Emergency LED Indicators ---
  if (motorRunning) {
    setSystemStatusColor(STATE_RUNNING); // Set LED to GREEN
  } else {
    setSystemStatusColor(STATE_IDLE);    // Set LED to BLUE
  }


  if (Serial.available() > 0) {
    char peekChar = Serial.peek();

    if (isAlpha(peekChar)) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      cmd.toUpperCase();

      if (cmd == "K" || cmd == "S") {
        stopMotorMotion();
      } else if (cmd == "C" || cmd == "J") {
        currentDirection = 'C';
        digitalWrite(PIN_M1_FR, LOW);
        digitalWrite(PIN_M2_FR, LOW);
        startMotorMotion(DAC_BASE_TURN);
      } else if (cmd == "A" || cmd == "L") {
        currentDirection = 'A';
        digitalWrite(PIN_M1_FR, HIGH);
        digitalWrite(PIN_M2_FR, HIGH);
        startMotorMotion(DAC_BASE_TURN);
      } else if (cmd == "F" || cmd == "I") {
        currentDirection = 'F';
        digitalWrite(PIN_M1_FR, LOW);
        digitalWrite(PIN_M2_FR, HIGH);
        startMotorMotion(DAC_BASE_STRAIGHT);
      } else if (cmd == "B" || cmd == "M") {
        currentDirection = 'B';
        digitalWrite(PIN_M1_FR, HIGH);
        digitalWrite(PIN_M2_FR, LOW);
        startMotorMotion(DAC_BASE_STRAIGHT);
      } else if (cmd == "T" || cmd == "t" || cmd == "Z" || cmd == "z") {
        command = cmd;
      }

      if (command == "T" || command == "t") {
        buzzerActive = true;
      } else {
        buzzerActive = false;
        digitalWrite(BUZZER_PIN, HIGH); // Ensure buzzer turns OFF immediately
      }
    } else if (isDigit(peekChar)) {
      long newMax = Serial.parseInt();
      if (newMax >= 0 && newMax <= 4095) {
        DAC_MAX = (uint16_t)newMax + 10;
      }
    } else {
      Serial.read();  // Discard spaces/newlines
    }
  }

  // Check and report state if updated via serial command
  checkAndReportState();

  if (buzzerActive) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    }
  }

  // --- Speed Management / Ramping Logic ---
  if (!motorRunning) {
    return;  // Do nothing if motors are toggled off
  }

  if (millis() - lastRampTime >= STEP_DELAY) {
    lastRampTime = millis();  // Reset timer

    if (dacValue < DAC_MAX) {
      if (DAC_MAX - dacValue < DAC_STEP) {
        dacValue = DAC_MAX;
      } else {
        dacValue += DAC_STEP;
      }
      dac.setVoltage(dacValue, false);
    } else if (dacValue > DAC_MAX) {
      if (dacValue - DAC_MAX < DAC_STEP) {
        dacValue = DAC_MAX;
      } else {
        dacValue -= DAC_STEP;
      }
      dac.setVoltage(dacValue, false);
    }
  }
}
