#include "Arduino.h"
#include "Control.h"
#include "Config.h"
#include "Utils.h"
#include "Debug.h"

// Variabili per le differenze di input e l'ultimo input
ControlInput DIFF_INPUT;
ControlInput LAST_DIFF_INPUT;


// Variabili statiche per l'accumulo della componente integrale
static float integralRoll = 0;
static float integralPitch = 0;
static float integralHead = 0;


unsigned long lastTime = 0;  // Variabile per il tempo dell'ultimo ciclo


// Funzione per il controllo PID
void PID(Drone *drone) {
  pid PID_OFFSET_ROLL;
  pid PID_OFFSET_PITCH;
  pid PID_OFFSET_HEAD;
  ControlInput ESC_OFFSET;
  ESC_output ESC_OFFSET_SUM;
  ESC_output ESC_OUTPUT;

  unsigned long currentTime = millis();  // Ottieni il tempo corrente in millisecondi
  float elapsedTime = (currentTime - lastTime) / 1000.0;  // Calcola il tempo trascorso in secondi
  
  // Aggiorna lastTime per il prossimo ciclo
  lastTime = currentTime;
  
  // Normalizzazione dell'angolo di Heading
  //float headingDiff = normalizeAngle(drone->RC_INPUT.HEAD - drone->IMU_INPUT.HEAD);
  // Calcola le differenze tra i valori di input della IMU e del RC (constrained)
  DIFF_INPUT.ROL = drone->IMU_INPUT.ROL - drone->RC_INPUT.ROL;
  DIFF_INPUT.PIT = drone->IMU_INPUT.PIT - drone->RC_INPUT.PIT;
  DIFF_INPUT.HEAD = drone->RC_INPUT.HEAD;//constrain(headingDiff, -MAX_DIFF_INPUT_HEAD, MAX_DIFF_INPUT_HEAD);

  // Calcolo della parte proporzionale (P) del PID
  PID_OFFSET_ROLL.P = PID_VALUE_ROLL_P * DIFF_INPUT.ROL;
  PID_OFFSET_PITCH.P = PID_VALUE_PITCH_P * DIFF_INPUT.PIT;
  PID_OFFSET_HEAD.P = PID_VALUE_HEAD_P * DIFF_INPUT.HEAD;

  // Calcolo della parte integrale (I) del PID
  if(drone->RC_INPUT.THR >= 20){
  integralRoll += DIFF_INPUT.ROL * elapsedTime;
  integralPitch += DIFF_INPUT.PIT * elapsedTime;
  integralHead += DIFF_INPUT.HEAD * elapsedTime;
  PID_OFFSET_ROLL.I = PID_VALUE_ROLL_I * integralRoll;
  PID_OFFSET_PITCH.I = PID_VALUE_PITCH_I * integralPitch;
  PID_OFFSET_HEAD.I = PID_VALUE_HEAD_I * integralHead;
  }else{
    integralRoll = 0;
    integralPitch = 0;
    integralHead = 0;
  }
  // Calcolo della parte derivativa (D) del PID
  PID_OFFSET_ROLL.D = PID_VALUE_ROLL_D * ((DIFF_INPUT.ROL - LAST_DIFF_INPUT.ROL) / elapsedTime);
  PID_OFFSET_PITCH.D = PID_VALUE_PITCH_D * ((DIFF_INPUT.PIT - LAST_DIFF_INPUT.PIT) / elapsedTime);
  PID_OFFSET_HEAD.D = PID_VALUE_HEAD_D * ((DIFF_INPUT.HEAD - LAST_DIFF_INPUT.HEAD) / elapsedTime);

  // Somma 
  ESC_OFFSET.ROL = PID_OFFSET_ROLL.P + PID_OFFSET_ROLL.I + PID_OFFSET_ROLL.D;
  ESC_OFFSET.PIT = PID_OFFSET_PITCH.P + PID_OFFSET_PITCH.I + PID_OFFSET_PITCH.D;
  ESC_OFFSET.HEAD = PID_OFFSET_HEAD.P + PID_OFFSET_HEAD.I + PID_OFFSET_HEAD.D;

  // Offset proporzionale al THR
  ESC_OFFSET.ROL *= drone->RC_INPUT.THR / 100;
  ESC_OFFSET.PIT *= drone->RC_INPUT.THR / 100;
  ESC_OFFSET.HEAD *= drone->RC_INPUT.THR / 100;

  // Calcolo della somma degli offset per gli ESC (motori)
  ESC_OFFSET_SUM.FRL = drone->RC_INPUT.THR - ESC_OFFSET.ROL + ESC_OFFSET.PIT - ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.FRR = drone->RC_INPUT.THR + ESC_OFFSET.ROL + ESC_OFFSET.PIT + ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.BKL = drone->RC_INPUT.THR - ESC_OFFSET.ROL - ESC_OFFSET.PIT + ESC_OFFSET.HEAD;
  ESC_OFFSET_SUM.BKR = drone->RC_INPUT.THR + ESC_OFFSET.ROL - ESC_OFFSET.PIT - ESC_OFFSET.HEAD;

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
    //DEBUG_PRINT("  PID Pitch P: ");
    //DEBUG_PRINT(PID_OFFSET_PITCH.P);
    //DEBUG_PRINT("  PID Pitch I: ");
    //DEBUG_PRINT(PID_OFFSET_PITCH.I);
    //DEBUG_PRINT("  PID Pitch D: ");
    //DEBUG_PRINT(PID_OFFSET_PITCH.D);
    //Serial.print("P  ID_OFFSET_HEAD.P: ");
    //Serial.print(PID_OFFSET_HEAD.P);
    //Serial.print("P  ID_OFFSET_HEAD.I: ");
    //Serial.print(PID_OFFSET_HEAD.I);
    //Serial.print("P  ID_OFFSET_HEAD.D: ");
    //Serial.print(PID_OFFSET_HEAD.D);
    // Offset finali per i motori
    DEBUG_PRINT("  PID ESC Roll Offset: ");
    DEBUG_PRINT(ESC_OFFSET.ROL);
    //DEBUG_PRINT("  PID ESC Pitch Offset: ");
    //DEBUG_PRINT(ESC_OFFSET.PIT);
    //Serial.print("  PID ESC Head Offset: ");
    //Serial.print(ESC_OFFSET.HEAD);
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
  LAST_DIFF_INPUT.HEAD = DIFF_INPUT.HEAD;
}
