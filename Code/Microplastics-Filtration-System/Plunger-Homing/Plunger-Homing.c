#include <AccelStepper.h>

// --- Pin Definitions ---
// Right TB6600 Driver (Lead Screw / Plunger)
const int PLUNGER_STEP_PIN = 2;  // Diagram trace: PUL+
const int PLUNGER_DIR_PIN  = 3;  // Diagram trace: DIR+
const int PLUNGER_ENA_PIN  = 4;  // Diagram trace: ENA+

// Redefined Limit Switch Pin (Moved from Pin 2 to avoid conflict)
const int PLUNGER_LIMIT_PIN = 10; 

// --- System Settings ---
// NEMA 17 + T8 Lead Screw Physics
// 1.8 deg motor = 200 steps/rev
// 1/8 Microstepping = 1600 steps/rev
// T8 Lead Screw Pitch = 8mm travel per rev
// Result: 1600 steps = 8mm -> 200 steps = 1mm
const int STEPS_PER_MM = 200; 

// Define stepper (Driver Mode)
AccelStepper plungerStepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // 1. Setup Pins
  pinMode(PLUNGER_ENA_PIN, OUTPUT);
  pinMode(PLUNGER_LIMIT_PIN, INPUT_PULLUP); // Switch: LOW = Pressed, HIGH = Open
  
  // 2. Enable Motor (Active LOW)
  digitalWrite(PLUNGER_ENA_PIN, LOW); 

  // 3. Configure Dynamics (Slower for linear mechanics to prevent binding)
  plungerStepper.setMaxSpeed(1000.0);
  plungerStepper.setAcceleration(500.0);

  // 4. Start Homing Routine
  Serial.println("System Powered. Homing Plunger (Retracting)...");
  homePlunger();
}

void loop() {
  // Test: Type 'p' to simulate a "Press" (Move down 50mm)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'p') {
      Serial.println("Pressing Syringe (50mm)...");
      plungerStepper.moveTo(50 * STEPS_PER_MM); // Move to absolute position 50mm down
    }
    if (cmd == 'h') {
      homePlunger(); // Re-home if needed
    }
  }
  
  plungerStepper.run();
}

// --- Homing Function ---
void homePlunger() {
  // 1. Retract (Move UP) towards the switch
  // Direction depends on wiring. If this moves DOWN, change to positive speed.
  Serial.println("  > Moving to Limit Switch...");
  plungerStepper.setSpeed(-400); 

  // 2. Move until Switch is Pressed (LOW)
  while (digitalRead(PLUNGER_LIMIT_PIN) == HIGH) {
    plungerStepper.runSpeed();
  }

  // 3. Stop immediately
  plungerStepper.setSpeed(0);
  plungerStepper.setCurrentPosition(0); 
  
  Serial.println("  > Switch Hit. Backing off...");

  // 4. Back off 2mm to release the switch state
  // blocking move used here for safety during calibration
  plungerStepper.runToNewPosition(2 * STEPS_PER_MM); 
  
  // 5. Re-Zero: This 2mm point is now our logical "Home" (Top clearance)
  plungerStepper.setCurrentPosition(0);
  
  Serial.println("Plunger Homing Complete. System Ready.");
}