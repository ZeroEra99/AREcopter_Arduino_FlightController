#include "Control.h"
#include "Config.h"
#include "Utils.h"
#include "Debug.h"

// Definizioni dei PID per Roll, Pitch e Yaw
const pid PID_VALUE_ROLL = {
  .P = 0.8,
  .I = 0,
  .D = 0.05
};
const pid PID_VALUE_PITCH = {
  .P = 0.8,
  .I = 0,
  .D = 0.05
};
const pid PID_VALUE_YAW = {
  .P = 0.8,
  .I = 0,
  .D = 0.05
};

float time, timePrev, elapsedTime;

// Variabili per le differenze di input e l'ultimo input
ControlInput DIFF_INPUT;
ControlInput LAST_DIFF_INPUT;

// Funzione per il controllo PID
void PID(Drone *drone, float elapsedTime) {

  pid PID_OFFSET_ROLL;
  pid PID_OFFSET_PITCH;
  pid PID_OFFSET_YAW;
  ControlInput ESC_OFFSET;
  ESC_output ESC_OFFSET_SUM;
  ESC_output ESC_OUTPUT;

  // Calcola le differenze tra i valori di input della IMU e del RC (constrained)
  DIFF_INPUT.ROL = constrain(drone->IMU_INPUT.ROL - drone->RC_INPUT.ROL, -MAX_DIFF_INPUT_ROLL, MAX_DIFF_INPUT_ROLL);
  DIFF_INPUT.PIT = constrain(drone->IMU_INPUT.PIT - drone->RC_INPUT.PIT, -MAX_DIFF_INPUT_PITCH, MAX_DIFF_INPUT_PITCH);

  // Calcolo della parte proporzionale (P) del PID
  PID_OFFSET_ROLL.P = PID_VALUE_ROLL.P * DIFF_INPUT.ROL;
  PID_OFFSET_PITCH.P = PID_VALUE_PITCH.P * DIFF_INPUT.PIT;
/*
  // Parte integrale (I) del PID
  if (-PID_INTEGRAL_ERROR < DIFF_INPUT.ROL && DIFF_INPUT.ROL < PID_INTEGRAL_ERROR)
    PID_OFFSET_ROLL.I += (PID_VALUE_ROLL.I * DIFF_INPUT.ROL);
  if (-PID_INTEGRAL_ERROR < DIFF_INPUT.PIT && DIFF_INPUT.PIT < PID_INTEGRAL_ERROR)
    PID_OFFSET_PITCH.I += (PID_VALUE_PITCH.I * DIFF_INPUT.PIT);
*/
  // Calcolo della parte derivativa (D) del PID
  PID_OFFSET_ROLL.D = PID_VALUE_ROLL.D * ((DIFF_INPUT.ROL - LAST_DIFF_INPUT.ROL) / elapsedTime);
  PID_OFFSET_PITCH.D = PID_VALUE_PITCH.D * ((DIFF_INPUT.PIT - LAST_DIFF_INPUT.PIT) / elapsedTime);

  // Limitazione della grandezza delle modifiche
  ESC_OFFSET.ROL = constrain(PID_OFFSET_ROLL.P + PID_OFFSET_ROLL.I + PID_OFFSET_ROLL.D, -MAX_OFFSET, MAX_OFFSET);
  ESC_OFFSET.PIT = constrain(PID_OFFSET_PITCH.P + PID_OFFSET_PITCH.I + PID_OFFSET_PITCH.D, -MAX_OFFSET, MAX_OFFSET);

  // Offset proporzionale al THR
  ESC_OFFSET.ROL *= drone->RC_INPUT.THR / 100;
  ESC_OFFSET.PIT *= drone->RC_INPUT.THR / 100;
  ESC_OFFSET.HEAD = 0;

  // Calcolo della somma degli offset per gli ESC (motori)
  ESC_OFFSET_SUM.FRL = constrain((drone->RC_INPUT.THR - ESC_OFFSET.ROL + ESC_OFFSET.PIT + ESC_OFFSET.HEAD), 0, MAX_RC_INPUT_THROTTLE);
  ESC_OFFSET_SUM.FRR = constrain((drone->RC_INPUT.THR + ESC_OFFSET.ROL + ESC_OFFSET.PIT - ESC_OFFSET.HEAD), 0, MAX_RC_INPUT_THROTTLE);
  ESC_OFFSET_SUM.BKL = constrain((drone->RC_INPUT.THR - ESC_OFFSET.ROL - ESC_OFFSET.PIT - ESC_OFFSET.HEAD), 0, MAX_RC_INPUT_THROTTLE);
  ESC_OFFSET_SUM.BKR = constrain((drone->RC_INPUT.THR + ESC_OFFSET.ROL - ESC_OFFSET.PIT + ESC_OFFSET.HEAD), 0, MAX_RC_INPUT_THROTTLE);

  // Conversione da thrust a PWM per i motori
  drone->ESC_OUTPUT.FRL = mapfloat(ESC_OFFSET_SUM.FRL, 0, MAX_RC_INPUT_THROTTLE, IO_MIN, IO_MAX);
  drone->ESC_OUTPUT.FRR = mapfloat(ESC_OFFSET_SUM.FRR, 0, MAX_RC_INPUT_THROTTLE, IO_MIN, IO_MAX);
  drone->ESC_OUTPUT.BKL = mapfloat(ESC_OFFSET_SUM.BKL, 0, MAX_RC_INPUT_THROTTLE, IO_MIN, IO_MAX);
  drone->ESC_OUTPUT.BKR = mapfloat(ESC_OFFSET_SUM.BKR, 0, MAX_RC_INPUT_THROTTLE, IO_MIN, IO_MAX);

  // Controllo failsafe/shutdown
  if (!drone->STATUS.isArmed || drone->STATUS.FAILSAFE != NONE) {
    drone->ESC_OUTPUT.FRL = IO_MIN;
    drone->ESC_OUTPUT.FRR = IO_MIN;
    drone->ESC_OUTPUT.BKL = IO_MIN;
    drone->ESC_OUTPUT.BKR = IO_MIN;
    return;
  }

  // Stampa i valori
  #if PID_DEBUG
    // Valori calcolati per PID Roll (P, I, D)
    DEBUG_PRINT("PID Roll P: ");
    DEBUG_PRINT(PID_OFFSET_ROLL.P);
    DEBUG_PRINT("  PID Roll I: ");
    DEBUG_PRINT(PID_OFFSET_ROLL.I);
    DEBUG_PRINT("  PID Roll D: ");
    DEBUG_PRINT(PID_OFFSET_ROLL.D);

    // Valori calcolati per PID Pitch (P, I, D)
    DEBUG_PRINT("  PID Pitch P: ");
    DEBUG_PRINT(PID_OFFSET_PITCH.P);
    DEBUG_PRINT("  PID Pitch I: ");
    DEBUG_PRINT(PID_OFFSET_PITCH.I);
    DEBUG_PRINT("  PID Pitch D: ");
    DEBUG_PRINT(PID_OFFSET_PITCH.D);

    // Offset finali per i motori
    DEBUG_PRINT("  PID ESC Roll Offset: ");
    DEBUG_PRINT(ESC_OFFSET.ROL);
    DEBUG_PRINT("  PID ESC Pitch Offset: ");
    DEBUG_PRINTLN(ESC_OFFSET.PIT);
  #endif

  #if PID_GRAPH_DEBUG
    // Debug grafico per PID
    DEBUG_PRINT("PID_Roll_P:"); DEBUG_PRINT(PID_OFFSET_ROLL.P);
    DEBUG_PRINT(",PID_Roll_I:"); DEBUG_PRINT(PID_OFFSET_ROLL.I);
    DEBUG_PRINT(",PID_Roll_D:"); DEBUG_PRINT(PID_OFFSET_ROLL.D);
    DEBUG_PRINT(",PID_Roll_Offset:"); DEBUG_PRINT(ESC_OFFSET.ROL);

    DEBUG_PRINT(",PID_Pitch_P:"); DEBUG_PRINT(PID_OFFSET_PITCH.P);
    DEBUG_PRINT(",PID_Pitch_I:"); DEBUG_PRINT(PID_OFFSET_PITCH.I);
    DEBUG_PRINT(",PID_Pitch_D:"); DEBUG_PRINT(PID_OFFSET_PITCH.D);
    DEBUG_PRINT(",PID_Pitch_Offset:"); DEBUG_PRINT(ESC_OFFSET.PIT);

    DEBUG_PRINTLN();
  #endif

  // Memorizza le differenze per l'uso futuro nel calcolo PID
  LAST_DIFF_INPUT.ROL = DIFF_INPUT.ROL;
  LAST_DIFF_INPUT.PIT = DIFF_INPUT.PIT;
}
