// --- Motor 1 Pin Definitions (Arduino Mega) ---
// Pin 2  = PE4 (INT4)
// Pin 3  = PE5 (INT5)
// Pin 21 = PD0 (INT0)
const uint8_t M1_HALL1_PIN = 2;
const uint8_t M1_HALL2_PIN = 3;
const uint8_t M1_HALL3_PIN = 21;

// --- Motor 2 Pin Definitions (Arduino Mega) ---
// Pin 18 = PD3 (INT3)
// Pin 19 = PD2 (INT2)
// Pin 20 = PD1 (INT1)
const uint8_t M2_HALL1_PIN = 18;
const uint8_t M2_HALL2_PIN = 19;
const uint8_t M2_HALL3_PIN = 20;

// Set based on gear ratio and pole pairs (e.g., 10 pole pairs * 6 steps * 27 gear ratio = 1620 or 1704)
const uint32_t PULSES_PER_OUTPUT_REV = 1710; 

// --- Motor 1 Volatile Variables ---
volatile uint8_t m1_lastHallState = 0xFF;
volatile uint32_t m1_lastPulseTime = 0;
volatile uint32_t m1_pulseInterval = 0;
volatile uint64_t m1_totalPulseCount = 0;

// --- Motor 2 Volatile Variables ---
volatile uint8_t m2_lastHallState = 0xFF;
volatile uint32_t m2_lastPulseTime = 0;
volatile uint32_t m2_pulseInterval = 0;
volatile uint64_t m2_totalPulseCount = 0;

uint32_t previousTime = 0;

inline int8_t getBLDCDirection(uint8_t prevState, uint8_t currState) {
  // Standard 6-step Hall state mapping: 1 -> 5 -> 4 -> 6 -> 2 -> 3 -> 1
  switch (prevState) {
    case 1: return (currState == 5) ? 1 : ((currState == 3) ? -1 : 0);
    case 5: return (currState == 4) ? 1 : ((currState == 1) ? -1 : 0);
    case 4: return (currState == 6) ? 1 : ((currState == 5) ? -1 : 0);
    case 6: return (currState == 2) ? 1 : ((currState == 4) ? -1 : 0);
    case 2: return (currState == 3) ? 1 : ((currState == 6) ? -1 : 0);
    case 3: return (currState == 1) ? 1 : ((currState == 2) ? -1 : 0);
    default: return 0;
  }
}

// Fast direct register read for Motor 1

inline uint8_t m1_readHallStateFast() {
  uint8_t h1 = (PINE >> 4) & 0x01;  // Pin 2 (PE4)
  uint8_t h2 = (PINE >> 5) & 0x01;  // Pin 3 (PE5)
  uint8_t h3 = (PIND >> 0) & 0x01;  // Pin 21 (PD0)
  return (h1 << 2) | (h2 << 1) | h3;
}

// Fast direct register read for Motor 2
inline uint8_t m2_readHallStateFast() {
  uint8_t h1 = (PIND >> 3) & 0x01;  // Pin 18 (PD3)
  uint8_t h2 = (PIND >> 2) & 0x01;  // Pin 19 (PD2)
  uint8_t h3 = (PIND >> 1) & 0x01;  // Pin 20 (PD1)
  return (h1 << 2) | (h2 << 1) | h3;
}

// ISR for Motor 1
void m1_hallISR() {
  uint32_t currentTime = micros();
  
  if (currentTime - m1_lastPulseTime < 20) {
    return;
  }

  uint8_t currentState = m1_readHallStateFast();

  if (currentState != m1_lastHallState && currentState != 0 && currentState != 7) {
    int8_t dir = getBLDCDirection(m1_lastHallState, currentState);
    
    // Add direction: +1 if moving forward, -1 if moving backward
    m1_totalPulseCount += dir;
    
    m1_pulseInterval = currentTime - m1_lastPulseTime;
    m1_lastPulseTime = currentTime;
    m1_lastHallState = currentState;
  }
}

// ISR for Motor 2
void m2_hallISR() {
  uint32_t currentTime = micros();
  
  if (currentTime - m2_lastPulseTime < 20) {
    return;
  }

  uint8_t currentState = m2_readHallStateFast();

  if (currentState != m2_lastHallState && currentState != 0 && currentState != 7) {
    // If M2 is mounted opposite to M1 physically, flip direction by swapping sign (-dir)
    int8_t dir = getBLDCDirection(m2_lastHallState, currentState);
    
    m2_totalPulseCount -= dir;
    
    m2_pulseInterval = currentTime - m2_lastPulseTime;
    m2_lastPulseTime = currentTime;
    m2_lastHallState = currentState;
  }
}

void setup() {
  Serial.begin(115200);

  // Configure Motor 1 Pins
  pinMode(M1_HALL1_PIN, INPUT_PULLUP);
  pinMode(M1_HALL2_PIN, INPUT_PULLUP);
  pinMode(M1_HALL3_PIN, INPUT_PULLUP);

  // Configure Motor 2 Pins
  pinMode(M2_HALL1_PIN, INPUT_PULLUP);
  pinMode(M2_HALL2_PIN, INPUT_PULLUP);
  pinMode(M2_HALL3_PIN, INPUT_PULLUP);

  m1_lastHallState = m1_readHallStateFast();
  m2_lastHallState = m2_readHallStateFast();

  // Attach Motor 1 Interrupts
  attachInterrupt(digitalPinToInterrupt(M1_HALL1_PIN), m1_hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_HALL2_PIN), m1_hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_HALL3_PIN), m1_hallISR, CHANGE);

  // Attach Motor 2 Interrupts
  attachInterrupt(digitalPinToInterrupt(M2_HALL1_PIN), m2_hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_HALL2_PIN), m2_hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_HALL3_PIN), m2_hallISR, CHANGE);
}

void loop() {
  uint32_t now = millis();

  if (now - previousTime >= 250) {
    previousTime = now;

    // Safely snapshot Motor 1 interrupt data
    noInterrupts();
    int64_t m1_pulses = m1_totalPulseCount;
    
    
    // Safely snapshot Motor 2 interrupt data
    int64_t m2_pulses = m2_totalPulseCount;
    
    interrupts();

    
    // Output Telemetry for both motors
    Serial.print((long)m1_pulses);
    Serial.print(",");
    Serial.println((long)m2_pulses);
  }
}
