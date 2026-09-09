#include <Arduino.h>

#include "config.h"
#include "motor.h"

#define START_STOP_BTN_PIN 40
#define RESET_BTN_PIN 42
#define E_STOP_BTN_PIN 37

#define SPEED_READ_INTERVAL_MS 1000
#define PULSES_PER_REV 30
#define DEBOUNCE_DELAY_MS 50

enum class AgvState {
  IDLE,
  MOVING_FORWARD,  // Sub-state dari Moving
  MOVING_ROTATE,   // Sub-state dari Moving
  LOADING,
  UNLOADING,
  OBSTACLE,
  ERROR,
  ESTOP
};

// Global Variables
AgvState currentState = AgvState::IDLE;
unsigned long stateStartTime = 0;

// State durations (simulasi waktu proses)
const unsigned long LOADING_DURATION_MS = 3000;
const unsigned long UNLOADING_DURATION_MS = 3000;
const unsigned long OBSTACLE_TIMEOUT_MS = 5000; // Jika > 5 detik, masuk ke ERROR

// Button states
bool eStopActive = false;
bool lastStartBtnState = HIGH;
unsigned long lastStartDebounceTime = 0;
bool lastResetBtnState = HIGH;
unsigned long lastResetDebounceTime = 0;

// Speed reading
unsigned long lastSpeedReadTime = 0;


// Declared Functions
const char* getStateName(AgvState);

// Motor Instance
Motor motorRight(R_START_STOP, R_RUN_BRAKE, R_DIR, R_INT_VR, R_PWM, R_SPEED_OUT);
Motor motorLeft(L_START_STOP, L_RUN_BRAKE, L_DIR, L_INT_VR, L_PWM, L_SPEED_OUT);

// --- Helper Functions ---
void changeState(AgvState newState) {
    if (currentState != newState) {
        currentState = newState;
        stateStartTime = millis();

        Serial.print("STATE CHANGE --> ");
        Serial.println(getStateName(currentState));

        if (currentState == AgvState::ESTOP || currentState == AgvState::ERROR) {
            motorRight.stop();
            motorLeft.stop();
        }
    }
}

const char* getStateName(AgvState state) {
  switch (state) {
    case AgvState::IDLE: return "IDLE";
    case AgvState::MOVING_FORWARD: return "MOVING_FORWARD";
    case AgvState::MOVING_ROTATE: return "MOVING_ROTATE";
    case AgvState::LOADING: return "LOADING";
    case AgvState::UNLOADING: return "UNLOADING";
    case AgvState::OBSTACLE: return "OBSTACLE";
    case AgvState::ERROR: return "ERROR";
    case AgvState::ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(START_STOP_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(E_STOP_BTN_PIN, INPUT_PULLUP);

  motorRight.begin();
  motorLeft.begin();

  changeState(AgvState::IDLE);
  Serial.println("AGV FSM Initialized. Ready.");
  Serial.println("Kirim via Serial Monitor: 'T'=Task, 'L'=Arrive Load, 'D'=Arrive Drop, 'O'=Obstacle, 'C'=Clear Obs");
}

void loop()
{
  // --- 1. E-Stop Check ---
  if (digitalRead(E_STOP_BTN_PIN) == HIGH)
  {
    if (!eStopActive) {
      eStopActive = true;
      motorRight.stop();
      motorLeft.stop();
      changeState(AgvState::ESTOP);
      Serial.println("E-STOP ACTIVATED !!!");
    }
    return;
  }

  // --- 3. Reset Button ---
  int resetReading = digitalRead(RESET_BTN_PIN);
  if (resetReading != lastResetBtnState) {
    lastResetDebounceTime = millis();
  }

  if ((millis() - lastResetDebounceTime) > DEBOUNCE_DELAY_MS) {
    static bool currentResetState = HIGH;
    if (resetReading != currentResetState) {
      currentResetState = resetReading;
      if (currentResetState == LOW) {
        if (currentState == AgvState::ESTOP || currentState == AgvState::ERROR) {
          eStopActive = false;
          changeState(AgvState::IDLE); // UML: EStop/Error --> Idle
          Serial.println("System Reset. E-Stop Cleared.");
        }
      }
    }
  }
  lastResetBtnState = resetReading;

  // --- 4. Start/Stop Button ---
  int startReading = digitalRead(START_STOP_BTN_PIN);
  if (startReading != lastStartBtnState) {
    lastStartDebounceTime = millis();
  }

  if ((millis() - lastStartDebounceTime) > DEBOUNCE_DELAY_MS) {
    static bool currentStartState = HIGH;
    if (startReading != currentStartState) {
      currentStartState = startReading;
      if (currentStartState == LOW) {
        // START
        if (currentState == AgvState::IDLE) {
          changeState(AgvState::MOVING_FORWARD);
        }
        // STOP
        else if (currentState == AgvState::MOVING_FORWARD ||
                  currentState == AgvState::MOVING_ROTATE ||
                  currentState == AgvState::LOADING ||
                  currentState == AgvState::UNLOADING ||
                  currentState == AgvState::OBSTACLE) {

          changeState(AgvState::IDLE);
        }
      }
    }
  }
  lastStartBtnState = startReading;

  // -- Simulasi Input Sensor via Serial Monitor (Untuk Testing FSM) --
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'T' && currentState == AgvState::IDLE) changeState(AgvState::MOVING_FORWARD);
    if (cmd == 'L' && currentState == AgvState::MOVING_FORWARD) changeState(AgvState::LOADING); // UML: Moving --> Loading
    if (cmd == 'R' && currentState == AgvState::MOVING_FORWARD) changeState(AgvState::MOVING_ROTATE); // Sub-state
    if (cmd == 'F' && currentState == AgvState::MOVING_ROTATE) changeState(AgvState::MOVING_FORWARD); // UML: Rotate --> Forward
    if (cmd == 'D' && (currentState == AgvState::MOVING_FORWARD || currentState == AgvState::MOVING_ROTATE)) changeState(AgvState::UNLOADING); // UML: Moving --> Unloading
    if (cmd == 'O' && currentState == AgvState::MOVING_FORWARD) changeState(AgvState::OBSTACLE); // UML: Moving --> Obstacle
    if (cmd == 'C' && currentState == AgvState::OBSTACLE) changeState(AgvState::MOVING_FORWARD); // UML: Obstacle --> Moving
  }


  // FSM LOGIC
  unsigned long currentTime = millis();

  switch (currentState) {
    case AgvState::IDLE:
      // Menunggu tugas. Motor mati.
      motorRight.stop();
      motorLeft.stop();
      break;

    case AgvState::MOVING_FORWARD:
      motorRight.set(DIR_CW, 50);
      motorLeft.set(DIR_CCW, 50);
      // CATATAN: Di sini nanti tambahkan logika: if (sensorGarisBelok) changeState(MOVING_ROTATE);
      break;

    case AgvState::MOVING_ROTATE:
      // Contoh: Putar di tempat
      motorRight.set(DIR_CW, 15);
      motorLeft.set(DIR_CW, 15);
      // CATATAN: Nanti tambahkan: if (imuAngleReached) changeState(MOVING_FORWARD);
      break;

    case AgvState::LOADING:
      motorRight.stop();
      motorLeft.stop();
      // Loading --> Moving (Goods loaded)
      if (currentTime - stateStartTime >= LOADING_DURATION_MS) {
        Serial.println("Goods loaded successfully.");
        changeState(AgvState::MOVING_FORWARD);
      }
      break;

    case AgvState::UNLOADING:
      motorRight.stop();
      motorLeft.stop();
      // Unloading --> Idle
      if (currentTime - stateStartTime >= UNLOADING_DURATION_MS) {
        Serial.println("Goods unloaded. Task completed.");
        changeState(AgvState::IDLE);
      }
      break;

    case AgvState::OBSTACLE:
      motorRight.stop();
      motorLeft.stop();
      // Obstacle --> Moving (Cleared) ATAU Obstacle --> Error (Timeout)
      if (currentTime - stateStartTime >= OBSTACLE_TIMEOUT_MS) {
        Serial.println("Obstacle timeout! Path blocked.");
        changeState(AgvState::ERROR); // UML: Obstacle --> Error
      }
      break;

    case AgvState::ERROR:
      motorRight.stop();
      motorLeft.stop();
      // Menunggu tombol RESET ditekan
      break;

    case AgvState::ESTOP:
      // Ditangani di bagian 1
      break;
  }


  // Speed Reading
  if (currentState == AgvState::MOVING_FORWARD || currentState == AgvState::MOVING_ROTATE) {
    if (currentTime - lastSpeedReadTime >= SPEED_READ_INTERVAL_MS) {
      unsigned long r_pulses = motorRight.getAndResetPulses();
      unsigned long l_pulses = motorLeft.getAndResetPulses();

      float intervalSec = SPEED_READ_INTERVAL_MS / 1000.0;
      float r_rpm = (r_pulses / intervalSec / PULSES_PER_REV) * 60.0;
      float l_rpm = (l_pulses / intervalSec / PULSES_PER_REV) * 60.0;

      Serial.print("R RPM: "); Serial.print(r_rpm / 15);
      Serial.print("  |  L RPM: "); Serial.println(l_rpm / 15);

      lastSpeedReadTime = currentTime;
    }
  }
}
