#include "Receiver.h"
#include "Config.h"
#include "Debug.h"
#include "Utils.h"

// Implementazione di setupRC
void setupRC(Drone *drone) {
  Serial.print("RC setup starting.\n");

  pinMode(RC_INPUT_THROTTLE_PIN, INPUT);
  pinMode(RC_INPUT_ROLL_PIN, INPUT);
  pinMode(RC_INPUT_PITCH_PIN, INPUT);
  pinMode(RC_INPUT_HEAD_PIN, INPUT);
  
  drone->RC_INPUT.HEAD = 0;

  Serial.print("RC setup completed.\n");
}

// Implementazione di readRC
void readRC(Drone *drone) {
  // Lettura dei valori RC
  float THR_PulseIn = pulseIn(RC_INPUT_THROTTLE_PIN, HIGH, 30000);
  float ROL_PulseIn = pulseIn(RC_INPUT_ROLL_PIN, HIGH, 30000);
  float PIT_PulseIn = pulseIn(RC_INPUT_PITCH_PIN, HIGH, 30000);
  float HEAD_PulseIn = pulseIn(RC_INPUT_HEAD_PIN, HIGH, 30000);

  // Controllo validità
  if (THR_PulseIn == 0 || ROL_PulseIn == 0 || PIT_PulseIn == 0 || HEAD_PulseIn == 0) {
    Serial.print("RC input is invalid.\n");
    drone->STATUS.FAILSAFE = RC;
  }

  // Arm e Disarm
  if (THR_PulseIn < (IO_MIN + ARMING_RANGE) &&
      HEAD_PulseIn < (IO_MIN + ARMING_RANGE) &&
      PIT_PulseIn < (IO_MIN + ARMING_RANGE) &&
      ROL_PulseIn > (IO_MAX - ARMING_RANGE)) {
    Serial.print("Drone status: armed.\n");
    drone->STATUS.isArmed = true;
  } else if (THR_PulseIn < (IO_MIN + ARMING_RANGE) &&
             PIT_PulseIn < (IO_MAX + ARMING_RANGE) &&
             HEAD_PulseIn > (IO_MAX - ARMING_RANGE) &&
             ROL_PulseIn < (IO_MIN + ARMING_RANGE)) {
    Serial.print("Drone status: unarmed\n");
    drone->STATUS.isArmed = false;
  }

  // Conversione e mappatura
  ROL_PulseIn = constrain(mapfloat(ROL_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL), -MAX_RC_INPUT_ROLL, MAX_RC_INPUT_ROLL);
  PIT_PulseIn = constrain(mapfloat(PIT_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH), -MAX_RC_INPUT_PITCH, MAX_RC_INPUT_PITCH);
  HEAD_PulseIn = constrain(mapfloat(HEAD_PulseIn, IO_MIN, IO_MAX, -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD), -MAX_RC_INPUT_HEAD, MAX_RC_INPUT_HEAD);
  THR_PulseIn = constrain(mapfloat(THR_PulseIn, IO_MIN, IO_MAX, 0, MAX_RC_INPUT_THROTTLE), 0, MAX_RC_INPUT_THROTTLE);

  // Smoothing
  if (abs(ROL_PulseIn) < MIN_RC_INPUT_ROLL) ROL_PulseIn = 0;
  if (abs(PIT_PulseIn) < MIN_RC_INPUT_PITCH) PIT_PulseIn = 0;
  if (abs(HEAD_PulseIn) < MIN_RC_INPUT_HEAD) HEAD_PulseIn = 0;
  if (abs(THR_PulseIn) < MIN_RC_INPUT_THROTTLE) THR_PulseIn = 0;

  // Mappatura finale
  if (hypot(ROL_PulseIn, PIT_PulseIn) > hypot(MAX_RC_INPUT_ROLL, MAX_RC_INPUT_PITCH)) {
    ROL_PulseIn *= MAX_RC_INPUT_ROLL / hypot(ROL_PulseIn, PIT_PulseIn);
    PIT_PulseIn *= MAX_RC_INPUT_PITCH / hypot(ROL_PulseIn, PIT_PulseIn);
  }

  // Assegnamento valori
  drone->RC_INPUT.ROL = ROL_PulseIn;
  drone->RC_INPUT.PIT = PIT_PulseIn;
  drone->RC_INPUT.HEAD = HEAD_PulseIn;
  drone->RC_INPUT.THR = THR_PulseIn;

  // Stampa i valori
  #if RC_DEBUG
    DEBUG_PRINT("\nInput Roll -> ");
    DEBUG_PRINT(drone->RC_INPUT.ROL);
    DEBUG_PRINT("Input Pitch -> ");
    DEBUG_PRINT(drone->RC_INPUT.PIT);
    DEBUG_PRINT("Input Heading -> ");
    DEBUG_PRINT(drone->RC_INPUT.HEAD);
  #endif

  #if RC_GRAPH_DEBUG
    // Debug grafico per RC
    DEBUG_PRINT("RC_Throttle:"); DEBUG_PRINT(drone->RC_INPUT.THR);
    DEBUG_PRINT(",RC_Roll:"); DEBUG_PRINT(drone->RC_INPUT.ROL);
    DEBUG_PRINT(",RC_Pitch:"); DEBUG_PRINT(drone->RC_INPUT.PIT);
    DEBUG_PRINT(",RC_Head:"); DEBUG_PRINT(drone->RC_INPUT.HEAD);
    DEBUG_PRINTLN();
  #endif


  if (drone->STATUS.FAILSAFE == RC)
    Serial.print("FAILSAFE - RC ERROR\n");
}
