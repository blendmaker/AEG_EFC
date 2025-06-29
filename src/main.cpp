// Ensure you have the 'Arduino Mbed OS RP2040 Boards' or similar RP2040 core installed
// For PlatformIO:
// platform = https://github.com/maxgerhardt/platform-raspberrypi.git#feature/rp2040-earlephilhower-2.x
// board = rpi-pico
// framework = arduino
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_MODE

// --- Pin Definitions ---
// Input pins are configured with INPUT_PULLDOWN, so switches pull HIGH (to 3.3V) when pressed.
const int TRIGGER_PIN = 15;       // Example GPIO14 - Abzug
const int CYCLE_PIN = 14;         // Example GPIO12 - Zyklus-Sensor
const int ANALOG_POT_PIN = 27;    // Example GPIO26 - Analog input for Potentiometer (ADC0)
const int FIRE_SELECTOR_PIN = 26;  // Example GPIO5 - Feuermodus-Schalter
const int MOTOR_OUTPUT_PIN = 28;  // Example GPIO15 - Motorsteuerung

const int ONBOARD_LED_PIN = 16;   // RP2040 Zero Onboard LED (GPIO25 on Pico/Zero)
Adafruit_NeoPixel pixel;

// --- Fire Mode Definitions ---
enum FireMode {
  SINGLE_SHOT = 0,
  BURST_3_SHOT = 1,
  FULL_AUTO = 2
};

const char* FIRE_MODE_NAMES[] = {"Single", "Burst", "Full Auto"};
FireMode currentFireMode = SINGLE_SHOT; // Default: Single shot (resets on power cycle)

// --- Volatile Variables for Shoot Control (retain values in Sleep Mode) ---
volatile int shotsToFire = 0;
volatile bool triggerDetected = false;
volatile bool cycleDetected = false;
volatile bool fireSelectorDetected = false;

bool motorRunning = false;
unsigned long motorStartTime = 0;
const long MAX_SHOT_DURATION_MS = 100; // Max motor run duration for safety

// --- Motor Delay from Potentiometer ---
int postCycleMotorDelayMs = 1; // Default value, will be updated by pot
const int MIN_DELAY_MS = 1;
const int MAX_DELAY_MS = 100;

// --- Function Forward Declarations ---
void startMotor();
void stopMotor(int delayMs = 0);
void indicateFireMode(FireMode mode);
void updateMotorDelayFromPot();
void SerialPrintLn(String s);
void SerialPrint(String s);

// --- Interrupt Service Routines (ISRs) ---
// ISRs should be minimal: just set a flag to be handled in loop().
void handleTriggerInterrupt() {
  static unsigned long lastInterruptTime = 0;
  if (millis() - lastInterruptTime > 30) { // 50ms debounce
    if (digitalRead(TRIGGER_PIN) == HIGH) { // Trigger pressed (pulls HIGH)
      triggerDetected = true;
    }
    lastInterruptTime = millis();
  }
}

void handleCycleInterrupt() {
  static unsigned long lastInterruptTime = 0;
  if (millis() - lastInterruptTime > 10) { // 20ms debounce
    if (digitalRead(CYCLE_PIN) == HIGH) { // Cycle sensor active (pulls HIGH)
      cycleDetected = true;
    }
    lastInterruptTime = millis();
  }
}

void handleFireSelectorInterrupt() {
  static unsigned long lastInterruptTime = 0;
  if (millis() - lastInterruptTime > 200) { // 200ms debounce for mode switch
    if (digitalRead(FIRE_SELECTOR_PIN) == HIGH) { // Switch pressed (pulls HIGH)
      fireSelectorDetected = true;
    }
    lastInterruptTime = millis();
  }
}


void setup() {
  #ifdef DEBUG_MODE
    Serial.begin(115200);
    Serial.println("\nRP2040 Zero AEG Controller Starting...");
  #endif

  pixel = Adafruit_NeoPixel(1, ONBOARD_LED_PIN, NEO_GRB + NEO_KHZ800);
  pixel.begin();
  pixel.clear();

  // Initialize Pins with INPUT_PULLDOWN, as switches pull HIGH
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  pinMode(CYCLE_PIN, INPUT_PULLDOWN);
  pinMode(FIRE_SELECTOR_PIN, INPUT_PULLDOWN);
  pinMode(MOTOR_OUTPUT_PIN, OUTPUT);
  pinMode(ONBOARD_LED_PIN, OUTPUT);

  digitalWrite(MOTOR_OUTPUT_PIN, LOW); // Motor initially off
  digitalWrite(ONBOARD_LED_PIN, LOW);  // LED initially off (HIGH for ON)

  // Initialize analog pin
  analogReadResolution(10); // 10-bit resolution for ADC (0-1023)

  // Attach Interrupts - RP2040 allows GPIO wake-up from Sleep Mode
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleTriggerInterrupt, RISING); // Trigger on HIGH edge (button press)
  attachInterrupt(digitalPinToInterrupt(CYCLE_PIN), handleCycleInterrupt, RISING);     // Cycle sensor active (on HIGH edge)
  attachInterrupt(digitalPinToInterrupt(FIRE_SELECTOR_PIN), handleFireSelectorInterrupt, RISING); // Mode switch (on HIGH edge)

  indicateFireMode(currentFireMode); // Show initial mode

  updateMotorDelayFromPot(); // Read initial delay from pot
}


void loop() {
  // Update motor delay from potentiometer regularly
  updateMotorDelayFromPot();

  // --- Process Interrupt Flags ---
  if (triggerDetected) {
    triggerDetected = false; // Reset flag
    SerialPrintLn("Trigger detected (via Interrupt).");
    if (!motorRunning) {
      if (currentFireMode == SINGLE_SHOT && shotsToFire == 0) {
        shotsToFire = 1;
        startMotor();
      } else if (currentFireMode == BURST_3_SHOT && shotsToFire == 0) {
        shotsToFire = 3;
        startMotor();
      } else if (currentFireMode == FULL_AUTO) {
        shotsToFire = 1; // Always 1, will be continuously requested
        startMotor();
      }
    }
  }

  if (cycleDetected) {
    cycleDetected = false; // Reset flag
    SerialPrintLn("Cycle detected (via Interrupt).");
    if (motorRunning) {
      if (currentFireMode == FULL_AUTO && digitalRead(TRIGGER_PIN) == HIGH) { // Trigger still pressed
        shotsToFire = 1; // Request next shot
      } else {
        shotsToFire = max(0, shotsToFire - 1);
      }

      if (shotsToFire == 0) {
        stopMotor(postCycleMotorDelayMs); // Stop motor after delay
      } else {
        SerialPrintLn("Next shot requested. Motor stays ON.");
      }
    }
  }

  if (fireSelectorDetected) {
    fireSelectorDetected = false; // Reset flag
    SerialPrintLn("Fire Mode change detected (via Interrupt).");
    currentFireMode = (FireMode)((currentFireMode + 1) % 3);
    indicateFireMode(currentFireMode);
    SerialPrint("Fire Mode changed to: ");
    SerialPrintLn(FIRE_MODE_NAMES[currentFireMode]);
  }

  // --- Logic for Trigger Release in Full Auto (Polling, as ISR only on RISING) ---
  // If trigger is released (pin goes from HIGH to LOW) and Full Auto is active
  if (currentFireMode == FULL_AUTO && digitalRead(TRIGGER_PIN) == LOW && motorRunning && shotsToFire > 0) {
      SerialPrintLn("Full Auto: Trigger released. Stopping next shots.");
      shotsToFire = 0; // Stops further shot requests
      stopMotor();
  }

  // --- Safety Timeout for Motor ---
  if (motorRunning && currentFireMode != FULL_AUTO) {
    if (millis() - motorStartTime >= (MAX_SHOT_DURATION_MS * shotsToFire))
    {
      SerialPrintLn("Safety timeout: stopping motor.");
      shotsToFire = 0; // Reset counter
      stopMotor();
    }
  }

  // --- Sleep Mode Logic ---
  // If no motor running and no shots pending, go into sleep mode
  if (!motorRunning && shotsToFire == 0) {
    // RP2040 Sleep Mode keeps RAM contents and wakes from GPIO.
    // The `sleep_ms(0)` effectively yields control and allows the system to enter a low-power state.
    // It's less aggressive than `Dormant` mode (which resets the chip).
    // Serial.println("Entering RP2040 Sleep Mode...");
    // Serial.flush(); // Ensure serial output is sent
    // The `tight_loop_contents()` often includes `yield()` or similar for power saving.
    // Explicitly calling `sleep_ms()` for a short duration or `__WFI()` (Wait For Interrupt)
    // is common in Pico SDK, but Arduino loop's implicit yield/idle management can be enough.
    // For a simple loop that enters sleep when idle, just let the loop continue with `delay(1)`
    // and rely on the core's power management, or use a more explicit sleep like this:
    // sleep_ms(0); // Effectively yields and can enter low power if no tasks are pending.
    // __WFI(); // Wait For Interrupt - a direct way to halt CPU until an interrupt occurs.
    // This is often within the `yield()` function or core's idle loop.
    // For Arduino, just keeping the loop minimal and letting `delay(1)` handle yielding is typical.
    // If you need more aggressive sleep:
    // sleep_run_from_rosc(); // If no precise timing needed during sleep (RTC accurate clock needed otherwise)
    // sleep_goto_sleep_until(timer_us_32() + 1000); // Wake up in 1ms
    // For this simple application, relying on the background power management and interrupt wake is often sufficient.
    delay(1); // Small delay to allow background tasks and system to go idle
  } else {
    delay(1); // Standard delay when active
  }
}

// --- Function Implementations ---

void startMotor() {
  if (!motorRunning) {
    digitalWrite(MOTOR_OUTPUT_PIN, HIGH);
    motorRunning = true;
    motorStartTime = millis();
    SerialPrintLn("Motor ON.");
  }
}

void stopMotor(int delayMs) {
  SerialPrint("Delay to stop motor: ");
  SerialPrintLn(String(delayMs));
  if (delayMs > 0) {
    unsigned long stopTime = millis() + delayMs;
    // Wait for delay, but continuously check trigger for full auto
    while(millis() < stopTime) {
        if (currentFireMode == FULL_AUTO && digitalRead(TRIGGER_PIN) == HIGH) { // If trigger re-pressed
            SerialPrintLn("Trigger re-pressed during stop delay (Full Auto). Motor stays ON.");
            return; // Motor stays on, stop request ignored
        }
        yield(); // Allow other tasks/interrupts
    }
  }

  if (motorRunning) { // Only stop if it's currently running
    digitalWrite(MOTOR_OUTPUT_PIN, LOW);
    motorRunning = false;
    shotsToFire = 0; // Reset after stopping
    SerialPrintLn("Motor OFF.");
  }
}

void indicateFireMode(FireMode mode) {
  int blinks = 0;
  switch (mode) {
    case SINGLE_SHOT: blinks = 1; break;
    case BURST_3_SHOT: blinks = 2; break;
    case FULL_AUTO: blinks = 6; break;
  }

  SerialPrint("Indicating Fire Mode: ");
  SerialPrintLn(FIRE_MODE_NAMES[mode]);

  for (int i = 0; i < blinks; i++) {
    pixel.clear();
    pixel.fill(pixel.Color(128, 128, 128));
    pixel.show();
    // digitalWrite(ONBOARD_LED_PIN, HIGH); // LED ON
    delay(200);
    // digitalWrite(ONBOARD_LED_PIN, LOW);  // LED OFF
    pixel.clear();
    pixel.fill(pixel.Color(0, 0, 0));
    pixel.show();
    delay(100);
  }
  // digitalWrite(ONBOARD_LED_PIN, LOW); // Ensure LED is off at the end
  pixel.clear();
}

void updateMotorDelayFromPot() {
  int analogValue = map(analogRead(ANALOG_POT_PIN), 0, 1023, MIN_DELAY_MS, MAX_DELAY_MS);
  if (analogValue != postCycleMotorDelayMs) {
    SerialPrint("Motor delay input: ");
    SerialPrintLn(String(postCycleMotorDelayMs));
    postCycleMotorDelayMs = analogValue;
  }
  
  // Map analog value (0-1023 for 10-bit ADC) to desired delay range (1-100 ms)

  // Optional: Serial.printf("Pot Value: %d -> Delay: %d ms\n", analogValue, postCycleMotorDelayMs);
}

void SerialPrintLn(String s) {
  SerialPrint(s);
  SerialPrint("\n");
}

void SerialPrint(String s) {
  #ifdef DEBUG_MODE
    Serial.print(s);
  #endif
}
