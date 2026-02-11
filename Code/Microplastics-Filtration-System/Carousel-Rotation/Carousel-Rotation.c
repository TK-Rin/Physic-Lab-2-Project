#include <AccelStepper.h>

// --- Pin Definitions ---
// TBB6600 Driver Pins to Arduino UNO R3
#define ROTARY_STEP_PIN 8
#define ROTARY_DIR_PIN 9
#define ROTARY_EN_PIN 10     // Optional: Enable pin to turn off motor holding torque if needed

// Micro Electronic Switch for Homing
#define ROTARY_LIMIT_PIN 3   

// --- System Configurations ---
const int TOTAL_SAMPLES = 10;
const int STEPS_PER_REV = 1600; // Assuming 1/8 Microstepping on TBB6600
const int STEPS_PER_INDEX = STEPS_PER_REV / TOTAL_SAMPLES; // 160 steps

int currentSyringe = 0;

// Initialize Stepper (Type 1 means a stepper driver with Step and Direction pins)
AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);

void setup() {
  Serial.begin(9600);
  
  pinMode(ROTARY_EN_PIN, OUTPUT);
  pinMode(ROTARY_LIMIT_PIN, INPUT_PULLUP); // Use internal pull-up for the limit switch
  
  digitalWrite(ROTARY_EN_PIN, LOW); // Enable the TBB6600 driver (usually active LOW)

  // Configure Motor Dynamics
  rotaryStepper.setMaxSpeed(500.0);      // Max steps per second
  rotaryStepper.setAcceleration(200.0);  // Steps per second squared for smooth start/stop

  Serial.println("System Powered. Starting Carousel Homing Sequence...");
  homeCarousel();
}

void loop() {
  // For testing: Send 'n' over Serial Monitor to index to the next syringe
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'n' || cmd == 'N') {
      moveToNextSyringe();
    }
  }

  // AccelStepper requires this to be called constantly in the loop
  // It only moves the motor if distanceToGo() != 0
  rotaryStepper.run(); 
}

// --- Functions ---

void homeCarousel() {
  Serial.println("Homing...");
  
  // Set a slow speed for homing to prevent crashing hard into the switch
  rotaryStepper.setSpeed(-150); 
  
  // Rotate backward until the limit switch is triggered (reads LOW)
  while (digitalRead(ROTARY_LIMIT_PIN) == HIGH) {
    rotaryStepper.runSpeed(); // runSpeed() moves at a constant speed, bypassing acceleration
  }
  
  // Once triggered, set this exact position as Zero
  rotaryStepper.setCurrentPosition(0);
  rotaryStepper.setSpeed(0);
  currentSyringe = 1; // Mark as Position 1
  
  Serial.println("Homing Complete. At Syringe Position 1.");
}

void moveToNextSyringe() {
  if (currentSyringe >= TOTAL_SAMPLES) {
    Serial.println("Warning: All 10 samples utilized. Cannot index further.");
    return;
  }

  // Calculate the target absolute position
  // Example: Moving to syringe 2 = 1 * 160 = absolute position 160
  long targetPosition = currentSyringe * STEPS_PER_INDEX;
  
  Serial.print("Indexing to Syringe ");
  Serial.print(currentSyringe + 1);
  Serial.print(" (Target Step: ");
  Serial.print(targetPosition);
  Serial.println(")");

  rotaryStepper.moveTo(targetPosition);
  currentSyringe++;
}