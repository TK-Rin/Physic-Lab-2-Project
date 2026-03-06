#include <AccelStepper.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
// Rotary Disk (Carousel) Pins
#define ROTARY_STEP_PIN 4
#define ROTARY_DIR_PIN 5
#define ROTARY_ENA_PIN 6
#define ROTARY_LIMIT_PIN 2 // SS-5GL 'NO' Pin

// Plunger (Linear Lead Screw) Pins
#define PLUNGER_STEP_PIN 7
#define PLUNGER_DIR_PIN 8
#define PLUNGER_ENA_PIN 9
#define PLUNGER_LIMIT_PIN 3 // SS-5GL 'NO' Pin

// ==========================================
// PHYSICAL CONSTANTS & KINEMATICS
// ==========================================
// Rotary Math (1/8 Microstepping)
const int TOTAL_SAMPLES = 10;
const int ROTARY_STEPS_PER_REV = 1600; 
const int STEPS_PER_INDEX = ROTARY_STEPS_PER_REV / TOTAL_SAMPLES; // 160 steps

// Plunger Math (T8 Lead Screw, 8mm pitch, 1/8 Microstepping)
const float PLUNGER_STEPS_PER_MM = 200.0; // 1600 steps / 8mm
const float PUSH_DISTANCE_MM = 125.0;     // 12.5 cm target distance

// State Variables
int currentSampleCount = 0; 

// Stepper Objects
AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);
AccelStepper plungerStepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

// ==========================================
// INITIALIZATION
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Configure Switch Pins (Internal Pullups prevent floating noise)
  pinMode(ROTARY_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PLUNGER_LIMIT_PIN, INPUT_PULLUP);
  
  // Configure Enable Pins
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  pinMode(PLUNGER_ENA_PIN, OUTPUT);
  digitalWrite(ROTARY_ENA_PIN, LOW);   // Enable Driver
  digitalWrite(PLUNGER_ENA_PIN, LOW);  // Enable Driver

  // --- Dynamics Tuning ---
  // Rotary: Slow to prevent sloshing the 8cm radius water samples
  rotaryStepper.setMaxSpeed(100.0);      
  rotaryStepper.setAcceleration(50.0);   
  
  // Plunger: Slow for maximum torque through the 5-micron filter
  plungerStepper.setMaxSpeed(400.0);     
  plungerStepper.setAcceleration(200.0); 

  // --- Homing Sequence ---
  Serial.println("System Powered. Initiating Homing Sequence...");
  homePlunger(); // Home Z-axis first for safety clearance
  homeRotary();  // Then home the carousel
  
  Serial.println("System Ready. Waiting for Serial Commands (N, P, R).");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'N' || cmd == 'n') {
      advanceRotary();
    } 
    else if (cmd == 'P' || cmd == 'p') {
      pushPlunger();
    }
    else if (cmd == 'R' || cmd == 'r') {
      retractPlunger();
    }
  }

  // Critical engines for non-blocking movement
  rotaryStepper.run();
  plungerStepper.run();
}

// ==========================================
// CORE FUNCTIONS
// ==========================================

void homePlunger() {
  Serial.print("  > Homing Plunger (Retracting UP)... ");
  plungerStepper.setSpeed(-200); // Negative = UP (Depends on wiring)
  
  while (digitalRead(PLUNGER_LIMIT_PIN) == HIGH) {
    plungerStepper.runSpeed();
  }
  
  plungerStepper.setSpeed(0);
  plungerStepper.setCurrentPosition(0);
  
  // Back off to clear the switch
  plungerStepper.runToNewPosition(2 * PLUNGER_STEPS_PER_MM); 
  plungerStepper.setCurrentPosition(0); // Set Absolute 0
  Serial.println("Done.");
}

void homeRotary() {
  Serial.print("  > Homing Rotary Disk (CCW)... ");
  rotaryStepper.setSpeed(-150); 
  
  while (digitalRead(ROTARY_LIMIT_PIN) == HIGH) {
    rotaryStepper.runSpeed();
  }
  
  rotaryStepper.setSpeed(0);
  rotaryStepper.setCurrentPosition(0);
  
  // Back off slightly to clear switch
  rotaryStepper.runToNewPosition(10);
  rotaryStepper.setCurrentPosition(0);
  
  currentSampleCount = 0;
  Serial.println("Done.");
}

void advanceRotary() {
  if (rotaryStepper.distanceToGo() == 0 && plungerStepper.currentPosition() == 0) {
    if (currentSampleCount < TOTAL_SAMPLES - 1) {
      currentSampleCount++;
      long target = currentSampleCount * STEPS_PER_INDEX;
      Serial.print("Indexing to Sample ");
      Serial.println(currentSampleCount + 1);
      rotaryStepper.moveTo(target);
    } else {
      Serial.println("Disk Full: 10 samples reached.");
    }
  } else {
    Serial.println("Error: Plunger must be fully retracted before rotating.");
  }
}

void pushPlunger() {
  if (plungerStepper.distanceToGo() == 0 && rotaryStepper.distanceToGo() == 0) {
    long targetSteps = PUSH_DISTANCE_MM * PLUNGER_STEPS_PER_MM; // 125mm * 200
    Serial.print("Pushing Plunger 125mm with High Torque...");
    plungerStepper.moveTo(targetSteps);
  }
}

void retractPlunger() {
  if (plungerStepper.distanceToGo() == 0) {
    Serial.println("Retracting Plunger to Zero...");
    plungerStepper.moveTo(0);
  }
}