#include <AccelStepper.h>

// --- Pin Definitions (Based on your Schematic) ---
const int ROTARY_STEP_PIN = 5;
const int ROTARY_DIR_PIN  = 6;
const int ROTARY_ENA_PIN  = 7;
const int LIMIT_SWITCH_PIN = 2;

// --- System Settings ---
// 1.8 degree motor = 200 steps/rev. 
// TB6600 set to 1/8 Microstepping = 1600 steps/rev.
const int STEPS_PER_REV = 1600; 
const int SYRINGE_COUNT = 10;
const int STEPS_PER_INDEX = STEPS_PER_REV / SYRINGE_COUNT; // 160 steps

// --- Variables ---
int currentSyringeIndex = 0; // Tracks which syringe (0-9) is active

// Define stepper using Driver mode (Step & Direction)
AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // 1. Setup Pins
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP); // HIGH when open, LOW when pressed
  
  // 2. Enable Motor (TB6600: Usually LOW = Enable, HIGH = Disable)
  digitalWrite(ROTARY_ENA_PIN, LOW); 

  // 3. Configure Speed (Steps per second)
  rotaryStepper.setMaxSpeed(800.0);
  rotaryStepper.setAcceleration(400.0);

  // 4. Run Homing Sequence
  Serial.println("System Start. Homing Rotary Disk...");
  homeRotaryDisk();
}

void loop() {
  // --- Test Interface ---
  // Type 'n' in Serial Monitor to move to the next syringe
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'n') {
      nextSyringe();
    }
  }

  // Critical: Must call run() frequently to process movement
  rotaryStepper.run();
}

// --- Helper Functions ---

void homeRotaryDisk() {
  // Move CCW slowly until switch is hit
  rotaryStepper.setSpeed(-200); // Negative speed for homing direction
  
  // Keep moving as long as Switch is NOT pressed (HIGH)
  while (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    rotaryStepper.runSpeed(); // Blocking movement at constant speed
  }
  
  // Stop immediately when Switch is pressed (LOW)
  rotaryStepper.setSpeed(0);
  rotaryStepper.setCurrentPosition(0); // Reset internal counter to Zero
  
  // Optional: Back off slightly to release switch
  rotaryStepper.runToNewPosition(20); 
  rotaryStepper.setCurrentPosition(0); // Re-zero
  
  currentSyringeIndex = 0;
  Serial.println("Homing Complete. Disk at Position 0.");
}

void nextSyringe() {
  if (rotaryStepper.distanceToGo() == 0) { // Only accept command if idle
    
    if (currentSyringeIndex < SYRINGE_COUNT - 1) {
      long targetPos = (currentSyringeIndex + 1) * STEPS_PER_INDEX;
      
      Serial.print("Moving to Syringe ");
      Serial.println(currentSyringeIndex + 1);
      
      rotaryStepper.moveTo(targetPos);
      currentSyringeIndex++;
      
    } else {
      Serial.println("Sequence Complete: All 10 syringes filled.");
      // Optional: Reset to 0? 
      // rotaryStepper.moveTo(0); 
      // currentSyringeIndex = 0;
    }
  }
}