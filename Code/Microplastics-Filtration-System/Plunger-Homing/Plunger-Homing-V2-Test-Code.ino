#include <AccelStepper.h>

// --- Configurable Pin Definitions ---
#define PLUNGER_STEP_PIN 7   // To TB6600 PUL+
#define PLUNGER_DIR_PIN 8    // To TB6600 DIR+
#define PLUNGER_ENA_PIN 9    // To TB6600 ENA+
#define SS5GL_LIMIT_PIN 3    // To SS-5GL 'NO' Terminal (COM to GND)

// --- System Physics (T8 Lead Screw & NEMA 17) ---
// 1.8 deg motor = 200 steps/rev. 1/8 Microstepping = 1600 steps/rev.
// T8 Lead Screw Pitch = 8mm linear travel per revolution.
// 1600 steps / 8mm = 200 steps per 1mm of linear travel.
const float STEPS_PER_MM = 200.0; 

// Initialize Stepper (Driver Mode)
AccelStepper plungerStepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // 1. Hardware Pin Configuration
  pinMode(PLUNGER_ENA_PIN, OUTPUT);
  pinMode(SS5GL_LIMIT_PIN, INPUT_PULLUP); // HIGH by default, drops to LOW when pressed
  
  digitalWrite(PLUNGER_ENA_PIN, LOW); // Enable motor driver (Active LOW)

  // 2. High Torque Dynamics Configuration
  // Low Max Speed ensures the motor stays in its highest torque band
  plungerStepper.setMaxSpeed(400.0);      // Max steps/sec (approx 2mm/sec)
  plungerStepper.setAcceleration(200.0);  // Gradual ramp-up prevents stalling under load

  // 3. System Calibration
  Serial.println("System Initializing: Homing Plunger (Retracting)...");
  homePlunger();
}

void loop() {
  // --- Test Interface ---
  // Send 'P' via Serial Monitor to simulate a high-torque push (e.g., pressing the syringe)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'P' || cmd == 'p') {
      pushSyringe(50.0); // Push 50mm down
    }
  }

  // Critical non-blocking engine
  plungerStepper.run(); 
}

// --- Core Functions ---

void homePlunger() {
  Serial.print("Retracting UP to find SS-5GL Limit Switch... ");
  
  // Set a steady, slow speed for homing (- value assumes UP, depending on wiring)
  plungerStepper.setSpeed(-200); 
  
  // Poll the switch. runSpeed() moves at a constant rate without acceleration.
  while (digitalRead(SS5GL_LIMIT_PIN) == HIGH) {
    plungerStepper.runSpeed(); 
  }
  
  // Switch triggered! (Reads LOW)
  plungerStepper.setSpeed(0);
  plungerStepper.setCurrentPosition(0); // This top position is now Absolute 0
  
  Serial.println("Limit Hit.");
  
  // Back off 2mm to release the switch lever so it isn't crushed
  Serial.println("Backing off 2mm to clear switch...");
  plungerStepper.runToNewPosition(2 * STEPS_PER_MM); 
  
  // Re-zero at this safe clearance height
  plungerStepper.setCurrentPosition(0); 
  Serial.println("Homing Complete. Plunger Ready at Z = 0.");
}

void pushSyringe125mm() {
  // 12.5 cm = 125 mm
  const float PUSH_DISTANCE_MM = 125.0; 
  
  if (plungerStepper.distanceToGo() == 0) {
    long targetSteps = PUSH_DISTANCE_MM * STEPS_PER_MM; // 25,000 steps
    
    Serial.print("Initiating High-Torque Push: ");
    Serial.print(PUSH_DISTANCE_MM);
    Serial.println(" mm...");
    
    // Plunger moves DOWN (assuming positive steps move away from the home switch)
    plungerStepper.moveTo(targetSteps);
  }
}

// --- Retract Function ---
void retractPlunger() {
  if (plungerStepper.distanceToGo() == 0) {
    Serial.println("Retracting Plunger to Home Position...");
    // Move back to the absolute zero position established during homing
    plungerStepper.moveTo(0); 
  }
}