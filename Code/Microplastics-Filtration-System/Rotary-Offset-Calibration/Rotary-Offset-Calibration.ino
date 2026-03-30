#include <AccelStepper.h>

// ==========================================
// PIN DEFINITIONS (From V6 Architecture)
// ==========================================
#define ROTARY_STEP_PIN 4
#define ROTARY_DIR_PIN 5
#define ROTARY_ENA_PIN 6
#define ROTARY_LIMIT_PIN 2 // SS-5GL 'NO' Pin

// Initialize Stepper Object
AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);

// Track position so we only print when the motor stops at a new location
long lastPrintedPosition = 0;

void setup() {
  Serial.begin(115200);
  
  // Hardware Setup
  pinMode(ROTARY_LIMIT_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  digitalWrite(ROTARY_ENA_PIN, LOW);  // Enable Stepper Driver (Active Low)

  // Motor Dynamics (Safe speeds for jogging)
  rotaryStepper.setMaxSpeed(200.0);      
  rotaryStepper.setAcceleration(100.0);  

  Serial.println("==========================================");
  Serial.println("  ROTARY OFFSET CALIBRATION TOOL");
  Serial.println("==========================================");
  
  homeRotary();

  Serial.println("\n=== Homing Complete. Position is ZERO ===");
  Serial.println("CONTROLS:");
  Serial.println(" [1] Jog +1 Step  (CW)");
  Serial.println(" [2] Jog -1 Step  (CCW)");
  Serial.println(" [q] Jog +10 Steps (Fast CW)");
  Serial.println(" [w] Jog -10 Steps (Fast CCW)");
  Serial.println("==========================================");
}

void loop() {
  // 1. Listen for Serial Commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == '1') {
      rotaryStepper.move(1);   // Relative move +1 step
    } 
    else if (cmd == '2') {
      rotaryStepper.move(-1);  // Relative move -1 step
    }
    else if (cmd == 'q' || cmd == 'Q') {
      rotaryStepper.move(10);  // Macro move +10 steps
    }
    else if (cmd == 'w' || cmd == 'W') {
      rotaryStepper.move(-10); // Macro move -10 steps
    }
  }

  // 2. Execute movement (Non-blocking)
  rotaryStepper.run();

  // 3. Print the absolute position once the motor stops moving
  if (rotaryStepper.distanceToGo() == 0 && rotaryStepper.currentPosition() != lastPrintedPosition) {
    lastPrintedPosition = rotaryStepper.currentPosition();
    
    Serial.print(">>> Current Offset: ");
    Serial.println(lastPrintedPosition);
  }
}

// ==========================================
// HOMING FUNCTION
// ==========================================
void homeRotary() {
  Serial.print("Homing Rotary Disk (CCW)... ");
  rotaryStepper.setSpeed(-112); // Negative = CCW
  
  while (digitalRead(ROTARY_LIMIT_PIN) == HIGH) {
    rotaryStepper.runSpeed();
  }
  
  // Stop and set to 0
  rotaryStepper.setSpeed(0);
  rotaryStepper.setCurrentPosition(0);
  
  // Back off slightly to clear the physical switch
  rotaryStepper.runToNewPosition(-10);
  
  // Re-Zero at this safe clearance point
  rotaryStepper.setCurrentPosition(0);
  Serial.println("Done.");
}