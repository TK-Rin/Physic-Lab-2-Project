#include <AccelStepper.h>

// --- Configurable Pin Definitions ---
#define ROTARY_STEP_PIN 4    // To TB6600 PUL+(Step)
#define ROTARY_DIR_PIN 5     // To TB6600 DIR+(Direction)
#define ROTARY_ENA_PIN 6     // To TB6600 ENA+(Enable)
#define SS5GL_LIMIT_PIN 2    // To SS-5GL 'NO' Terminal (COM to GND)

// --- System Constants ---
const int TOTAL_SAMPLES = 10;
const int STEPS_PER_REV = 1600; // TB6600 set to 1/8 Microstepping
const int STEPS_PER_INDEX = STEPS_PER_REV / TOTAL_SAMPLES; // 160 Steps

int currentSampleCount = 0; // Tracks filled syringes (0 to 10)

// Initialize Stepper (Type 1 = Dedicated Driver Board)
AccelStepper carouselStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  // 1. Hardware Pin Configuration
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  pinMode(SS5GL_LIMIT_PIN, INPUT_PULLUP); // HIGH by default, LOW when triggered
  
  digitalWrite(ROTARY_ENA_PIN, LOW); // Enable TB6600 (active LOW standard)

  // 2. Motor Dynamics Configuration
  carouselStepper.setMaxSpeed(100.0);      // Max speed in steps/sec
  carouselStepper.setAcceleration(50.0);  // Smooth acceleration/deceleration curve

  // 3. System Calibration
  Serial.println("System Initializing: Homing Carousel...");
  homeCarousel();
}

void loop() {
  // --- Test Interface ---
  // Send 'N' via Serial Monitor to simulate loading the next syringe
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'N' || cmd == 'n') {
      advanceToNextSyringe();
    }
  }

  // Critical: Must be called continuously to process non-blocking movement
  carouselStepper.run(); 
}

// --- Core Functions ---

void homeCarousel() {
  Serial.print("Rotating CCW to seek zero position... ");
  
  // Move in negative direction (Counter-Clockwise) at a safe, constant speed
  carouselStepper.setSpeed(-150); 
  
  // Poll the SS-5GL limit switch
  while (digitalRead(SS5GL_LIMIT_PIN) == HIGH) {
    carouselStepper.runSpeed(); // Blocking move bypassing acceleration
  }
  
  // Switch triggered (LOW)
  carouselStepper.setSpeed(0);
  carouselStepper.setCurrentPosition(0); // Set this physical location as Absolute 0
  
  currentSampleCount = 0; // Ready at Position 1
  Serial.println("Homing Complete. At Syringe 1.");
}

void advanceToNextSyringe() {
  // Prevent commands while the motor is already in motion
  if (carouselStepper.distanceToGo() != 0) {
    return; 
  }

  if (currentSampleCount < TOTAL_SAMPLES - 1) {
    currentSampleCount++;
    
    // Calculate the absolute step position for the target syringe
    long targetAbsolutePosition = currentSampleCount * STEPS_PER_INDEX;
    
    Serial.print("Indexing to Sample ");
    Serial.print(currentSampleCount + 1);
    Serial.print(" | Target Step: ");
    Serial.println(targetAbsolutePosition);
    
    carouselStepper.moveTo(targetAbsolutePosition);
  } else {
    Serial.println("Action Denied: All 10 samples collected. Carousel full.");
  }
}