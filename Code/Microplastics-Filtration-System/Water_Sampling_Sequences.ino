#include <AccelStepper.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
#define ROTARY_STEP_PIN 4
#define ROTARY_DIR_PIN 5
#define ROTARY_ENA_PIN 6
#define ROTARY_LIMIT_PIN 2

#define PLUNGER_STEP_PIN 7
#define PLUNGER_DIR_PIN 8
#define PLUNGER_ENA_PIN 9
#define PLUNGER_LIMIT_PIN 3

#define PUMP_MOSFET_PIN 10   // IRF520 'SIG' pin

// ==========================================
// SYSTEM CONSTANTS & CALIBRATION
// ==========================================
const int TOTAL_SAMPLES = 10;
const int STEPS_PER_INDEX = 1600 / TOTAL_SAMPLES; // 160 steps
const float PLUNGER_STEPS_PER_MM = 200.0;         // T8 lead screw
const float PUSH_DISTANCE_MM = 125.0;             // 12.5 cm target

// Pump Timer: Adjust so exactly 100ml is dispensed
const unsigned long PUMP_TIME_MS = 4500; 

// ==========================================
// STATE MACHINE VARIABLES
// ==========================================
enum SystemState {
  STATE_INIT,
  STATE_FILL_SYRINGE,
  STATE_WAIT_FILL,
  STATE_PUSH_PLUNGER,
  STATE_WAIT_PUSH,
  STATE_RETRACT_PLUNGER,
  STATE_WAIT_RETRACT,
  STATE_ROTATE_DISK,
  STATE_WAIT_ROTATE,
  STATE_COMPLETE
};

SystemState currentState = STATE_INIT;
int completedSamples = 0; 
unsigned long stateTimer = 0;

AccelStepper rotaryStepper(AccelStepper::DRIVER, ROTARY_STEP_PIN, ROTARY_DIR_PIN);
AccelStepper plungerStepper(AccelStepper::DRIVER, PLUNGER_STEP_PIN, PLUNGER_DIR_PIN);

// ==========================================
// SETUP & HOMING
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(ROTARY_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PLUNGER_LIMIT_PIN, INPUT_PULLUP);
  
  pinMode(PUMP_MOSFET_PIN, OUTPUT);
  digitalWrite(PUMP_MOSFET_PIN, LOW); 
  
  pinMode(ROTARY_ENA_PIN, OUTPUT);
  pinMode(PLUNGER_ENA_PIN, OUTPUT);
  digitalWrite(ROTARY_ENA_PIN, LOW);   
  digitalWrite(PLUNGER_ENA_PIN, LOW);  

  rotaryStepper.setMaxSpeed(100.0);      
  rotaryStepper.setAcceleration(50.0);   
  plungerStepper.setMaxSpeed(400.0);     
  plungerStepper.setAcceleration(200.0); 

  Serial.println("System Power On. Homing Hardware...");
  homePlunger(); 
  homeRotary();  
  
  Serial.println("Homing Complete. Starting autonomous sequence in 5 seconds...");
  delay(5000); // Blocking delay is safe here before the loop starts
  
  currentState = STATE_FILL_SYRINGE; // Kick off the sequence
}

// ==========================================
// MAIN LOOP (Autonomous Engine)
// ==========================================
void loop() {
  
  // 1. Execute the State Machine Logic
  switch (currentState) {
    
    case STATE_FILL_SYRINGE:
      Serial.print("--- Sample ");
      Serial.print(completedSamples + 1);
      Serial.println(" ---");
      Serial.println("  > Pumping 100ml water...");
      digitalWrite(PUMP_MOSFET_PIN, HIGH);
      stateTimer = millis(); 
      currentState = STATE_WAIT_FILL;
      break;

    case STATE_WAIT_FILL:
      if (millis() - stateTimer >= PUMP_TIME_MS) {
        digitalWrite(PUMP_MOSFET_PIN, LOW);
        Serial.println("  > Pump OFF.");
        currentState = STATE_PUSH_PLUNGER;
      }
      break;

    case STATE_PUSH_PLUNGER:
      Serial.println("  > Pushing Plunger 125mm...");
      plungerStepper.moveTo(PUSH_DISTANCE_MM * PLUNGER_STEPS_PER_MM);
      currentState = STATE_WAIT_PUSH;
      break;

    case STATE_WAIT_PUSH:
      if (plungerStepper.distanceToGo() == 0) {
        Serial.println("  > Push Complete.");
        currentState = STATE_RETRACT_PLUNGER;
      }
      break;

    case STATE_RETRACT_PLUNGER:
      Serial.println("  > Retracting Plunger...");
      plungerStepper.moveTo(0); // Return to absolute zero
      currentState = STATE_WAIT_RETRACT;
      break;

    case STATE_WAIT_RETRACT:
      if (plungerStepper.distanceToGo() == 0) {
        completedSamples++;
        Serial.println("  > Sample Complete.");
        
        if (completedSamples >= TOTAL_SAMPLES) {
          currentState = STATE_COMPLETE;
        } else {
          currentState = STATE_ROTATE_DISK;
        }
      }
      break;

    case STATE_ROTATE_DISK:
      Serial.println("  > Rotating to next syringe...");
      rotaryStepper.moveTo(completedSamples * STEPS_PER_INDEX);
      currentState = STATE_WAIT_ROTATE;
      break;

    case STATE_WAIT_ROTATE:
      if (rotaryStepper.distanceToGo() == 0) {
        currentState = STATE_FILL_SYRINGE; // Loop back to the start
      }
      break;

    case STATE_COMPLETE:
      // Turn off steppers to save battery and prevent heat buildup
      digitalWrite(ROTARY_ENA_PIN, HIGH); 
      digitalWrite(PLUNGER_ENA_PIN, HIGH);
      Serial.println("=================================");
      Serial.println("MISSION SUCCESS: 10 Samples Collected.");
      Serial.println("System Halted.");
      while(true); // Trap the Arduino here forever until manually reset
      break;
      
    default:
      break;
  }

  // 2. Keep the motors pulsing in the background
  rotaryStepper.run();
  plungerStepper.run();
}

// ==========================================
// HOMING FUNCTIONS 
// ==========================================
void homePlunger() {
  plungerStepper.setSpeed(-200); 
  while (digitalRead(PLUNGER_LIMIT_PIN) == HIGH) plungerStepper.runSpeed();
  plungerStepper.setSpeed(0);
  plungerStepper.setCurrentPosition(0);
  plungerStepper.runToNewPosition(2 * PLUNGER_STEPS_PER_MM); // Back off switch
  plungerStepper.setCurrentPosition(0); 
}

void homeRotary() {
  rotaryStepper.setSpeed(-150); 
  while (digitalRead(ROTARY_LIMIT_PIN) == HIGH) rotaryStepper.runSpeed();
  rotaryStepper.setSpeed(0);
  rotaryStepper.setCurrentPosition(0);
  rotaryStepper.runToNewPosition(10); // Back off switch
  rotaryStepper.setCurrentPosition(0);
}