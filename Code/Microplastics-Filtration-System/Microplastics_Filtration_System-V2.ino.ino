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

// Water Pump & Navigation Pins
#define PUMP_MOSFET_PIN 10     // MOSFET Gate pin
#define PIXHAWK_SIGNAL_PIN 11  // Input from Pixhawk (Requires 10k Pull-Down resistor to GND)

// ==========================================
// PHYSICAL CONSTANTS & KINEMATICS
// ==========================================
// Rotary Math 
const int TOTAL_SAMPLES = 10;
const int CYCLES_PER_SYRINGE = 10;
const long ROTARY_STEPS_PER_REV = 800; 
const long STEPS_PER_INDEX = ROTARY_STEPS_PER_REV / TOTAL_SAMPLES; // 80 steps

// Plunger Math 
const float PLUNGER_STEPS_PER_MM = 200.0; 
const float PUSH_DISTANCE_MM = 125.0;     

// System Offsets & Timers
const long PUMP_POSITION_OFFSET = 0;     
const long PLUNGER_POSITION_OFFSET = 40; 
const unsigned long PUMP_TIME_MS = 4500; 

// ==========================================
// FSM STATE VARIABLES
// ==========================================
enum SystemState {
  STATE_INIT,
  STATE_WAIT_PIXHAWK,        // NEW: Wait for boat to anchor at waypoint
  STATE_MOVE_TO_PUMP,
  STATE_WAIT_ROTARY_PUMP,
  STATE_PUMPING,
  STATE_WAIT_PUMP,
  STATE_MOVE_TO_PLUNGER,
  STATE_WAIT_ROTARY_PLUNGER,
  STATE_PLUNGE_DOWN,
  STATE_WAIT_PLUNGE_DOWN,
  STATE_PLUNGE_UP,
  STATE_WAIT_PLUNGE_UP,
  STATE_EVALUATE_LOOPS,
  STATE_DONE
};

SystemState currentState = STATE_INIT;
int currentSyringe = 0; 
int currentCycle = 0;   

unsigned long pumpStartTime = 0;
bool manualOverride = false; // Flag for Serial Monitor override

// Stepper Objects
AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);
AccelStepper plungerStepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

// ==========================================
// INITIALIZATION
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Configure Input Pins 
  pinMode(ROTARY_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PLUNGER_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PIXHAWK_SIGNAL_PIN, INPUT); // Expects external Pull-Down resistor
  
  // Configure Output Pins
  pinMode(PUMP_MOSFET_PIN, OUTPUT);
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  pinMode(PLUNGER_ENA_PIN, OUTPUT);
  
  digitalWrite(PUMP_MOSFET_PIN, LOW);
  digitalWrite(ROTARY_ENA_PIN, LOW);   
  digitalWrite(PLUNGER_ENA_PIN, LOW);  

  // --- Dynamics Tuning ---
  rotaryStepper.setMaxSpeed(112.0);      
  rotaryStepper.setAcceleration(500.0);   
  
  // Plunger Speed Increased (Watch for torque loss!)
  plungerStepper.setMaxSpeed(2000.0);     
  plungerStepper.setAcceleration(8000.0); 

  // --- Homing Sequence ---
  Serial.println("System Powered. Initiating Homing Sequence...");
  homePlunger(); 
  homeRotary();  
  
  Serial.println("\n=== Homing Complete. ===");
  currentState = STATE_WAIT_PIXHAWK; // Send to waiting state
}

// ==========================================
// HELPER: STRICT COUNTER-CLOCKWISE MOVEMENT
// ==========================================
// Calculates backward (CCW) steps required to hit a specific rotational 
// angle, ensuring the motor NEVER turns forward (CW).
void moveToTargetCCW(long targetModulo) {
  long currentMod = rotaryStepper.currentPosition() % ROTARY_STEPS_PER_REV;
  
  // Handle negative modulo in C++ to normalize between 0 and 799
  if (currentMod < 0) {
    currentMod += ROTARY_STEPS_PER_REV;
  }
  
  long stepsToMove = currentMod - targetModulo;
  
  // If target is mathematically "ahead" of us, complete the rotation backward
  if (stepsToMove <= 0) {
    stepsToMove += ROTARY_STEPS_PER_REV;
  }
  
  // Move in the negative direction for CCW
  rotaryStepper.move(-stepsToMove); 
}

// ==========================================
// MAIN LOOP (Autonomous FSM Engine)
// ==========================================
void loop() {
  
  // --- Check for Serial Override ---
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'G' || cmd == 'g') {
      manualOverride = true;
      Serial.println(">>> SERIAL OVERRIDE RECEIVED <<<");
    }
  }

  switch (currentState) {
    
    // --- 0. WAIT FOR PIXHAWK / BOAT ANCHOR ---
    case STATE_WAIT_PIXHAWK:
      // Print waiting message once per syringe
      if (currentCycle == 0 && !manualOverride && digitalRead(PIXHAWK_SIGNAL_PIN) == LOW) {
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 2000) {
          Serial.print("Waiting for Pixhawk Signal (Pin 11) or 'G' via Serial to start Syringe ");
          Serial.println(currentSyringe + 1);
          lastPrint = millis();
        }
      }

      // Check Triggers (Pixhawk HIGH or Serial 'G')
      if (digitalRead(PIXHAWK_SIGNAL_PIN) == HIGH || manualOverride) {
        manualOverride = false; // Reset override flag
        Serial.println("\n>>> SIGNAL RECEIVED. STARTING SAMPLE SEQUENCE <<<");
        currentState = STATE_MOVE_TO_PUMP;
      }
      break;

    // --- 1. ALIGN SYRINGE WITH WATER PUMP ---
    case STATE_MOVE_TO_PUMP:
      {
        Serial.print("\n[Syringe "); Serial.print(currentSyringe + 1); 
        Serial.print("/10 | Cycle "); Serial.print(currentCycle + 1); Serial.println("/10]");
        Serial.println("  > Rotating CCW to Water Pump...");
        
        long syringeBasePosition = currentSyringe * STEPS_PER_INDEX;
        long pumpTarget = (syringeBasePosition + PUMP_POSITION_OFFSET) % ROTARY_STEPS_PER_REV;
        
        moveToTargetCCW(pumpTarget);
        currentState = STATE_WAIT_ROTARY_PUMP;
      }
      break;

    case STATE_WAIT_ROTARY_PUMP:
      if (rotaryStepper.distanceToGo() == 0) currentState = STATE_PUMPING;
      break;

    // --- 2. FILL SYRINGE ---
    case STATE_PUMPING:
      Serial.println("  > Pumping 100ml water...");
      digitalWrite(PUMP_MOSFET_PIN, HIGH);
      pumpStartTime = millis(); 
      currentState = STATE_WAIT_PUMP;
      break;

    case STATE_WAIT_PUMP:
      if (millis() - pumpStartTime >= PUMP_TIME_MS) {
        digitalWrite(PUMP_MOSFET_PIN, LOW); // Turn off Pump
        Serial.println("  > Pump OFF.");
        currentState = STATE_MOVE_TO_PLUNGER;
      }
      break;

    // --- 3. ALIGN SYRINGE WITH PLUNGER ---
    case STATE_MOVE_TO_PLUNGER:
      {
        Serial.println("  > Rotating CCW to Plunger...");
        long syringeBasePosition = currentSyringe * STEPS_PER_INDEX;
        long plungerTarget = (syringeBasePosition + PLUNGER_POSITION_OFFSET) % ROTARY_STEPS_PER_REV;
        
        moveToTargetCCW(plungerTarget);
        currentState = STATE_WAIT_ROTARY_PLUNGER;
      }
      break;

    case STATE_WAIT_ROTARY_PLUNGER:
      if (rotaryStepper.distanceToGo() == 0) currentState = STATE_PLUNGE_DOWN;
      break;

    // --- 4. PLUNGE ACTION (PUSH) ---
    case STATE_PLUNGE_DOWN:
      Serial.println("  > Pushing Plunger Down...");
      plungerStepper.moveTo(PUSH_DISTANCE_MM * PLUNGER_STEPS_PER_MM);
      currentState = STATE_WAIT_PLUNGE_DOWN;
      break;

    case STATE_WAIT_PLUNGE_DOWN:
      if (plungerStepper.distanceToGo() == 0) currentState = STATE_PLUNGE_UP;
      break;

    // --- 5. PLUNGE ACTION (RETRACT) ---
    case STATE_PLUNGE_UP:
      Serial.println("  > Retracting Plunger to Zero...");
      plungerStepper.moveTo(0); // Return to absolute zero (top limit)
      currentState = STATE_WAIT_PLUNGE_UP;
      break;

    case STATE_WAIT_PLUNGE_UP:
      if (plungerStepper.distanceToGo() == 0) currentState = STATE_EVALUATE_LOOPS;
      break;

    // --- 6. NESTED LOOP EVALUATION ---
    case STATE_EVALUATE_LOOPS:
      currentCycle++; // Cycle finished
      
      if (currentCycle < CYCLES_PER_SYRINGE) {
        // Still have cycles left for THIS syringe. Loop back to pump alignment.
        // It bypasses STATE_WAIT_PIXHAWK because the boat is still anchored.
        currentState = STATE_MOVE_TO_PUMP; 
      } else {
        // 10 cycles finished. Reset cycle counter and move to NEXT syringe.
        currentCycle = 0;
        currentSyringe++;
        
        if (currentSyringe < TOTAL_SAMPLES) {
          // Send back to the Wait state for the boat to travel to the next waypoint
          currentState = STATE_WAIT_PIXHAWK; 
        } else {
          currentState = STATE_DONE; // 10 syringes x 10 cycles finished
        }
      }
      break;

    // --- 7. MISSION COMPLETE ---
    case STATE_DONE:
      digitalWrite(ROTARY_ENA_PIN, HIGH);  
      digitalWrite(PLUNGER_ENA_PIN, HIGH);
      Serial.println("\n=============================================");
      Serial.println("MISSION SUCCESS: 10 Syringes Fully Processed.");
      Serial.println("System Halted.");
      while(true); // Trap loop indefinitely 
      break;
      
    case STATE_INIT:
    default:
      break;
  }

  // Critical background tasks (pulses the motors if distanceToGo != 0)
  rotaryStepper.run();
  plungerStepper.run();
}

// ==========================================
// HOMING FUNCTIONS 
// ==========================================
void homePlunger() {
  plungerStepper.runToNewPosition(10 * PLUNGER_STEPS_PER_MM); 
  Serial.print("  > Homing Plunger (Retracting UP)... ");
  plungerStepper.setSpeed(-1000); 
  
  while (digitalRead(PLUNGER_LIMIT_PIN) == HIGH) {
    plungerStepper.runSpeed();
  }
  
  plungerStepper.setSpeed(0);
  plungerStepper.setCurrentPosition(0);
  plungerStepper.runToNewPosition(15 * PLUNGER_STEPS_PER_MM); 
  plungerStepper.setCurrentPosition(0); 
  Serial.println("Done.");
}

void homeRotary() {
  Serial.print("  > Homing Rotary Disk (CCW)... ");
  rotaryStepper.setSpeed(-112); // Negative = CCW
  
  while (digitalRead(ROTARY_LIMIT_PIN) == HIGH) {
    rotaryStepper.runSpeed();
  }
  
  rotaryStepper.setSpeed(0);
  rotaryStepper.setCurrentPosition(0);
  rotaryStepper.runToNewPosition(-10); // Back off switch CCW
  rotaryStepper.setCurrentPosition(0);
  Serial.println("Done.");
}